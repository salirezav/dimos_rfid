# Copyright 2026. RFID DimOS integration.

"""Run RFID blueprints without registering them in the dimos CLI.

Examples:
    python -m dimos_rfid demo
    python -m dimos_rfid go2
    python -m dimos_rfid go2-dataset
    python -m dimos_rfid go2-dataset-rounds
    python -m dimos_rfid go2-agentic
    python -m dimos_rfid semantic
"""

from __future__ import annotations

import argparse
import sys


def main(argv: list[str] | None = None) -> None:
    # Load repo-root .env first so ROBOT_IP / RFID_* defaults apply unless
    # already exported in the shell (shell exports always win).
    from dimos_rfid.env import load_repo_dotenv

    load_repo_dotenv(override=False)

    parser = argparse.ArgumentParser(description="Run DimOS RFID blueprints")
    parser.add_argument(
        "blueprint",
        choices=[
            "demo",
            "go2",
            "go2-dataset",
            "go2-dataset-rounds",
            "go2-agentic",
            "semantic",
        ],
        help="Which blueprint to run",
    )
    args = parser.parse_args(argv)

    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    if args.blueprint == "demo":
        from dimos_rfid.demo_blueprint import rfid_demo

        blueprint = rfid_demo
    elif args.blueprint == "go2":
        from dimos_rfid.go2_blueprints import unitree_go2_rfid

        blueprint = unitree_go2_rfid
    elif args.blueprint == "go2-dataset":
        from dimos_rfid.go2_blueprints import unitree_go2_rfid_dataset

        blueprint = unitree_go2_rfid_dataset
    elif args.blueprint == "go2-dataset-rounds":
        from dimos_rfid.dataset_rounds_blueprints import unitree_go2_rfid_dataset_rounds

        blueprint = unitree_go2_rfid_dataset_rounds
    elif args.blueprint == "semantic":
        from dimos_rfid.semantic_rfid_blueprints import unitree_go2_rfid_semantic

        blueprint = unitree_go2_rfid_semantic
    else:
        from dimos_rfid.go2_agentic_blueprints import unitree_go2_rfid_agentic

        blueprint = unitree_go2_rfid_agentic

    ModuleCoordinator.build(blueprint).loop()


if __name__ == "__main__":
    main(sys.argv[1:])
