# Focused RFID finder agent

This is a deliberately small first agent. It reads the EPCs or EPC suffixes in
`rfid_module/rfid_focus.txt` and reuses the standalone RFID module's robust
5–10-pose localizer.

## Run

From the repository root:

```bash
export ROBOT_IP=<go2-ip>
python Agent/rfid_finder_agent.py --go2
```

The agent prints one of:

- `SEARCHING`: the focused EPC has not been detected.
- `COLLECTING`: the tag is detected but fewer than five finalized poses exist.
- `REFINING`: five or more poses exist, but confidence is still low.
- `FOUND`: a usable robust world-frame position is available.

At each observation pose, stop long enough to collect at least three RFID
samples. Move at least 0.3 m before the next stop. Surrounding the suspected tag
from different directions is substantially better than collecting poses along
one straight line.

The finder intentionally does not move the robot on its own. This keeps the
first version safe and makes localization behavior easy to validate.

## Optional MCP skill

Run the agentic composition:

```bash
python Agent/rfid_finder_agent.py --go2 --agentic
```

The method `find_focused_rfid_tag()` is marked as a DimOS agent skill. The
machine-readable RPC is `get_focused_tag_location()`.
