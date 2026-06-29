# Copyright 2026. RFID DimOS integration.
#
# Draws RFID tag indicators on the Go2 camera view in Rerun.

from __future__ import annotations

import time
from dataclasses import dataclass, field

from reactivex.disposable import Disposable

import rerun as rr

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image

from dimos_rfid.msgs import RfidTag, RfidTagArray


@dataclass
class RfidCameraOverlay:
    """Tag markers to render on top of the camera feed."""

    tags: list[RfidTag] = field(default_factory=list)
    width: int = 1280
    height: int = 720
    ts: float = 0.0

    def to_rerun(self) -> list[tuple[str, rr.Archetype]]:
        """Log Points2D + labels under world/color_image/rfid."""
        active = [t for t in self.tags if t.in_range]
        if not active:
            return [
                (
                    "world/color_image/rfid",
                    rr.Clear(recursive=True),
                )
            ]

        w, h = self.width, self.height
        n = len(active)
        positions: list[list[float]] = []
        radii: list[float] = []
        colors: list[list[int]] = []
        labels: list[str] = []

        for i, tag in enumerate(active):
            # Spread along lower third of image (no bearing yet — see README).
            x = w * (0.12 + 0.76 * (i / max(n - 1, 1)))
            y = h * 0.78
            positions.append([x, y])

            rssi = tag.rssi_dbm if tag.rssi_dbm is not None else -75
            # Stronger signal → larger dot (-40 dBm big, -80 dBm small).
            radii.append(float(max(10.0, min(36.0, (-rssi - 35) * 0.55)))

            strength = max(0.0, min(1.0, (rssi + 85) / 35.0))
            colors.append([int(80 + 175 * strength), int(255 - 80 * strength), 80])

            short = tag.name or f"…{tag.epc[-8:].upper()}"
            labels.append(f"{short}\n{tag.rssi_dbm} dBm")

        return [
            (
                "world/color_image/rfid",
                rr.Points2D(
                    positions=positions,
                    radii=radii,
                    colors=colors,
                    labels=labels,
                    show_labels=True,
                ),
            )
        ]


class RfidOverlayModule(Module):
    """Subscribe to camera + RFID tags; publish overlay for RerunBridge."""

    color_image: In[Image]
    rfid_tags: In[RfidTagArray]
    rfid_overlay: Out[RfidCameraOverlay]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._width = 1280
        self._height = 720
        self._latest_tags: list[RfidTag] = []

    @rpc
    def start(self) -> None:
        super().start()

        def on_image(img: Image) -> None:
            self._width = int(img.width) if img.width else self._width
            self._height = int(img.height) if img.height else self._height

        def on_tags(msg: RfidTagArray) -> None:
            self._latest_tags = list(msg.tags)
            self._publish_overlay()

        unsub_img = self.color_image.subscribe(on_image)
        unsub_tags = self.rfid_tags.subscribe(on_tags)
        self.register_disposable(Disposable(unsub_img) if callable(unsub_img) else unsub_img)
        self.register_disposable(Disposable(unsub_tags) if callable(unsub_tags) else unsub_tags)

    def _publish_overlay(self) -> None:
        self.rfid_overlay.publish(
            RfidCameraOverlay(
                tags=self._latest_tags,
                width=self._width,
                height=self._height,
                ts=time.time(),
            )
        )
