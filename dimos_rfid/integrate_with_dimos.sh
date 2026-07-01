#!/usr/bin/env bash
# Register dimos_rfid inside the installed dimos package.
# Re-run after upgrading dimos: ./integrate_with_dimos.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
RFID_PKG="dimos.hardware.sensors.rfid"

if ! command -v uv &>/dev/null; then
    echo "Install uv first: https://docs.astral.sh/uv/" >&2
    exit 1
fi

cd "${PROJECT_ROOT}"

echo "Syncing project environment with uv..."
uv sync --extra unitree --frozen

DIMOS_PKG="$(uv run python -c "import dimos, os; print(os.path.dirname(dimos.__file__))")"
RFID_DIR="${DIMOS_PKG}/hardware/sensors/rfid"

echo "Vendoring RFID module into ${RFID_DIR}..."
mkdir -p "${RFID_DIR}"

for f in bridge.py demo_blueprint.py go2_blueprints.py go2_agentic_blueprints.py msgs.py rfid_module.py rfid_rerun.py _backend.py; do
    cp "${SCRIPT_DIR}/${f}" "${RFID_DIR}/${f}"
done

# Blueprint stubs in dimos/ must import from dimos.hardware.sensors.rfid, not dimos_rfid.
for f in demo_blueprint.py go2_blueprints.py go2_agentic_blueprints.py rfid_module.py rfid_rerun.py; do
    sed -i \
        -e "s/from dimos_rfid\./from ${RFID_PKG}./g" \
        -e "s/from dimos_rfid import/from ${RFID_PKG} import/g" \
        "${RFID_DIR}/${f}"
done

sed -i "s/from dimos_rfid\.go2_blueprints/from ${RFID_PKG}.go2_blueprints/g" \
    "${RFID_DIR}/go2_agentic_blueprints.py"

echo "Regenerating dimos blueprint registry..."
uv run python - <<'PY'
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.robot.test_all_blueprints_generation import (
    _generate_all_blueprints_content,
    _scan_for_blueprints,
)

root = DIMOS_PROJECT_ROOT / "dimos"
blueprints, modules = _scan_for_blueprints(root)
out = root / "robot" / "all_blueprints.py"
out.write_text(_generate_all_blueprints_content(blueprints, modules))
print(f"Updated {out}")
PY

echo ""
echo "Verifying imports..."
uv run python - <<PY
from dimos.robot.get_all_blueprints import get_blueprint_by_name
for name in ("rfid-demo", "unitree-go2-rfid"):
    get_blueprint_by_name(name)
    print(f"  {name}: OK")
PY

echo ""
echo "Registered RFID blueprints:"
uv run dimos list | grep -i rfid || true

echo ""
echo "Done. Example:"
echo "  export ROBOT_IP=<go2-wifi-ip>"
echo "  export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1"
echo "  uv run dimos run unitree-go2-rfid"
