"""Single-file DimOS module that polls an RFID scanner HTTP API.

Asynchronously polls a local RFID API every `interval` seconds and prints the
active tag count, EPC, and RSSI to the console.

Ways to run it natively (all bypass the `dimos` CLI daemon):

    python rfid_module.py                # in-process  -> RECOMMENDED for the debugger
    python rfid_module.py --coordinator  # via ModuleCoordinator (forks a worker process)
    python rfid_module.py --ui           # opens the DimOS (Rerun) viewer and shows tags

The `--ui` mode runs the module through DimOS's ModuleCoordinator and opens the
same viewer `dimos run` uses (`dimos-viewer`), displaying the live tag list as a
text panel. The module owns the viewer in its own process so the panel always
renders (no cross-process Rerun recording mismatch).

Why the default is in-process
-----------------------------
`ModuleCoordinator.build()` deploys each module into a *forkserver child
process* (DimOS always runs a worker pool; there is no in-process worker mode).
On Python 3.12 (this workspace), debugpy has known bugs where stepping
(F10/F11) inside fork/forkserver child processes behaves like "continue", so
breakpoints in the async loop are unreliable there.

Running in-process instead executes the async loop in a background *thread* of
this same process, which the debugger handles perfectly: breakpoints, stepping,
and variable inspection in `_poll_loop` / `_print_tags` all work.

Notes on this DimOS build (see the workspace `.venv`)
----------------------------------------------------
- The lifecycle hook the coordinator calls is `start()`, not `setup()`. To keep
  the requested `setup()` + `self.spawn()` shape, `start()` calls `self.setup()`.
- DimOS builds a module's config from field kwargs (`config_type(**kwargs)` with
  `extra="forbid"`). `__init__` below accepts a `config=` object and expands it
  into fields so `RFIDModule.blueprint(config=RFIDConfig())` works as requested.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import threading
from concurrent.futures import Future
from typing import Any

import httpx
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_URL = "http://10.42.200.240:8765/api/v1/tags/active"


class RFIDConfig(ModuleConfig):
    """Configuration for :class:`RFIDModule`.

    `ModuleConfig` is a pydantic model, so these are two plain pydantic fields
    added on top of the framework's config fields.
    """

    url: str = Field(default=DEFAULT_URL, description="RFID active-tags endpoint.")
    interval: float = Field(default=0.5, gt=0, description="Poll interval in seconds.")
    rerun: bool = Field(
        default=False,
        description="Also display the live tag list in the DimOS (Rerun) viewer.",
    )


# Rerun entity path for the RFID text panel.
RERUN_ENTITY = "rfid"


class RFIDModule(Module):
    """Polls the RFID API in an async background loop and prints results."""

    config: RFIDConfig

    # `spawn()` schedules onto the module loop via run_coroutine_threadsafe,
    # which returns a concurrent.futures.Future (not an asyncio.Task).
    _poll_task: Future[Any] | None = None
    _stop_flag: threading.Event | None = None
    _rerun_ready: bool = False

    def __init__(self, **kwargs: Any) -> None:
        # Support `RFIDModule.blueprint(config=RFIDConfig())` and direct
        # construction with a config object. DimOS builds config from field
        # kwargs, so expand the object into its fields (existing kwargs win).
        cfg = kwargs.pop("config", None)
        if cfg is not None:
            for field_name in type(cfg).model_fields:
                kwargs.setdefault(field_name, getattr(cfg, field_name))
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self.setup()

    def setup(self) -> None:
        """Kick off the async polling loop on the module event loop."""
        logger.info("RFIDModule polling %s every %.2fs", self.config.url, self.config.interval)
        if self.config.rerun:
            self._init_rerun()
        self._stop_flag = threading.Event()
        self._poll_task = self.spawn(self._poll_loop())

    def _init_rerun(self) -> None:
        """Open the DimOS (Rerun) viewer and lay out a single RFID text panel."""
        try:
            import rerun as rr
            import rerun.blueprint as rrb
            from dimos.visualization.rerun.bridge import RERUN_GRPC_PORT

            rr.init("dimos")
            try:
                # Prefer the DimOS-branded viewer (same one `dimos run` opens).
                import rerun_bindings

                rerun_bindings.spawn(
                    port=RERUN_GRPC_PORT,
                    executable_name="dimos-viewer",
                    memory_limit="25%",
                )
                rr.connect_grpc(f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy")
            except Exception:
                # Fall back to the stock Rerun viewer if dimos-viewer is absent.
                rr.spawn(connect=True)

            view = (
                rrb.TextDocumentView(origin=RERUN_ENTITY, name="RFID tags")
                if hasattr(rrb, "TextDocumentView")
                else rrb.TextLogView(origin=RERUN_ENTITY, name="RFID tags")
            )
            rr.send_blueprint(rrb.Blueprint(view, rrb.TimePanel(state="collapsed")))
            self._rerun_ready = True
            logger.info("RFIDModule: DimOS viewer ready — tags will appear in the 'RFID tags' panel")
        except Exception as exc:  # noqa: BLE001 - UI is best-effort; keep polling either way
            logger.warning("RFIDModule: could not open the Rerun viewer: %s", exc)
            self._rerun_ready = False

    async def _poll_loop(self) -> None:
        """Continuously fetch active tags without blocking the event loop.

        Good place for a breakpoint: step through a request/response cycle and
        inspect `payload`, `tags`, `epc`, `rssi`.
        """
        stop = self._stop_flag
        assert stop is not None
        async with httpx.AsyncClient(timeout=1.5) as client:
            while not stop.is_set():
                try:
                    response = await client.get(self.config.url)
                    response.raise_for_status()
                    payload = response.json()
                    self._print_tags(payload)
                    if self._rerun_ready:
                        self._log_rerun(payload)
                except httpx.HTTPError as exc:
                    logger.warning("RFID poll failed (%s): %s", self.config.url, exc)
                except Exception as exc:  # noqa: BLE001 - keep the loop alive
                    logger.warning("RFID poll error: %s", exc)
                await asyncio.sleep(self.config.interval)

    @staticmethod
    def _print_tags(payload: dict) -> None:
        count = payload.get("count", 0)
        tags = payload.get("tags", []) or []
        if not tags:
            print(f"[RFID] {count} tag(s) in range")
            return
        print(f"[RFID] {count} tag(s) in range:")
        for tag in tags:
            epc = tag.get("epc", "?")
            rssi = tag.get("rssi_dbm")
            rssi_s = f"{rssi} dBm" if rssi is not None else "unknown RSSI"
            print(f"    EPC={epc}  RSSI={rssi_s}")

    @staticmethod
    def _tags_markdown(payload: dict) -> str:
        count = payload.get("count", 0)
        tags = payload.get("tags", []) or []
        lines = [f"# RFID — {count} tag(s) in range", ""]
        if not tags:
            lines.append("_No tags in range._")
        else:
            lines.append("| EPC | RSSI |")
            lines.append("|-----|------|")
            for tag in tags:
                epc = tag.get("epc", "?")
                rssi = tag.get("rssi_dbm")
                rssi_s = f"{rssi} dBm" if rssi is not None else "—"
                lines.append(f"| `{epc}` | {rssi_s} |")
        return "\n".join(lines)

    def _log_rerun(self, payload: dict) -> None:
        """Push the current tag list to the viewer as a text panel."""
        try:
            import rerun as rr

            md = self._tags_markdown(payload)
            try:
                rr.log(RERUN_ENTITY, rr.TextDocument(md, media_type=rr.MediaType.MARKDOWN))
            except (AttributeError, TypeError):
                rr.log(RERUN_ENTITY, rr.TextLog(md))
        except Exception as exc:  # noqa: BLE001 - never let UI logging break polling
            logger.debug("RFIDModule: rerun log failed: %s", exc)

    @rpc
    def stop(self) -> None:
        if self._stop_flag is not None:
            self._stop_flag.set()
        task = self._poll_task
        if task is not None:
            # Wait for the loop to exit on its own so the future resolves
            # normally (no CancelledError surfaced at shutdown).
            try:
                task.result(timeout=self.config.interval + 2.0)
            except BaseException:
                pass
            self._poll_task = None
        super().stop()


def run_in_process(config: RFIDConfig) -> None:
    """Run the module in THIS process (no worker fork) for IDE debugging.

    The async loop runs in a background thread of this process, so debugger
    breakpoints and stepping work reliably.
    """
    module = RFIDModule(config=config)
    module.start()
    logger.info("RFIDModule running in-process (pid=%d). Press Ctrl-C to stop.", os.getpid())
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        module.stop()


def run_via_coordinator(config: RFIDConfig) -> None:
    """Run the blueprint via ModuleCoordinator (deploys into a worker process)."""
    blueprint = autoconnect(RFIDModule.blueprint(config=config))
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        coordinator.loop()
    except KeyboardInterrupt:
        coordinator.stop()


def run_with_ui(config: RFIDConfig) -> None:
    """Open the DimOS (Rerun) viewer and show the live tag list in a text panel.

    Runs in-process (no forkserver worker) so it works identically from a plain
    terminal and from the IDE debugger (F5). Under the debugger, a forked worker
    plus the native viewer is unreliable on Python 3.12, so we avoid it here.
    """
    config = config.model_copy(update={"rerun": True})
    logger.info("Starting DimOS viewer + RFID module (Ctrl-C to stop)...")
    run_in_process(config)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the RFID DimOS module natively.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--ui",
        action="store_true",
        help="Open the DimOS (Rerun) viewer and display the tags in a text panel.",
    )
    group.add_argument(
        "--coordinator",
        action="store_true",
        help="Run via ModuleCoordinator (forks a worker process; stepping in the "
        "child is unreliable on Python 3.12). Default runs in-process for debugging.",
    )
    args = parser.parse_args()

    if args.ui:
        run_with_ui(RFIDConfig())
    elif args.coordinator:
        run_via_coordinator(RFIDConfig())
    else:
        run_in_process(RFIDConfig())
