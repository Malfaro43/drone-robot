# Agent guide: drone-robot

DJI RoboMaster TT (Tello Talent) and an XGO Lite dog, both driven from the laptop. The drone maps a safe route to a target mission pad; the dog walks it. Start with `README.md` (repository map, how it fits together); `docs/HARDWARE_DAY.md` is the runbook for real hardware.

## Safety (non-negotiable)

- Never run a flight script without the user confirming the drone is in a clear space. `--check` style connect-only runs and `calibrate.py show` are fine.
- Every flight script must land in a `finally` block and treat Ctrl+C as "land now". Every dog-moving script must stop the dog in a `finally`.
- Keep indoor test moves small (`go`/`move_*` minimum is 20 cm; the scripts cap climbs at 50 cm).
- The dog only walks routes with `route_checked: true`. Don't weaken that check, the clearance (`dog_width/2 + margin + CELL/2`), or the rule that only *stopped* readings confirm a square, to make a number look better.
- While the dog moves, resend its command at least every 0.5 s: the Pi stops it after 1 s of silence (`pi_xgo_bridge.CommandWatchdog`).

## Environment

- Use `.venv/bin/python` (Python 3.13). Python 3.14 is the system default, but don't use it for this project. Setup: `.venv/bin/pip install -r requirements-dev.txt`.
- The drone is at `192.168.10.1` in AP mode. Commands go over UDP 8889, state comes back on 8890, and video on 11111. Run drone scripts from **Apple Terminal**: Ghostty has no macOS Local Network permission and never gets the drone's replies.
- The dog's Pi is reached over Tailscale (`http://100.74.30.90:8080`); see the XGO repo's `AGENTS.md` before touching it.

## Tests

- `.venv/bin/python -m pytest` runs every suite (search, dog, calibration, bridge; ~3 s, no hardware). Run it before every commit.
- `scripts/search/stress.py --rooms 500` scores the mission over random rooms against the real layout. **0 unsafe checked routes** is the invariant; the unit tests run 150 rooms and fail otherwise. When a change trades one metric for another, report both numbers.
- CI (`.github/workflows/tests.yml`) also runs the Pi-side code on Python 3.9.

## Conventions and gotchas

- **Simulation never applies `calibration.json`** (tests must not depend on local measurements). Only the real-hardware entry points call `calibration.apply_to_flight` / read dog values.
- **Calibrated signs live in modules:** `tracker.PAD_X_SIGN/PAD_Y_SIGN/YAW_CW_POSITIVE` and `mission.GO_Y_SIGN` (not `find_pad`, which only re-exports the mission).
- **`pi_xgo_bridge/remote_command.py` has an identical copy** in the XGO repo at `scripts/xgo_robot/remote_command.py`. Change both, keep them byte-identical, and keep them Python 3.9 compatible (the Pi runs 3.9.2).
- **The XGO repo's `scripts/xgo_robot/http_stream.py` uses CRLF line endings.** Editing it with tools that rewrite newlines turns a small change into a whole-file diff; preserve CRLF.
- **djitellopy doesn't serialise commands** and matches replies by arrival order. Use the bridge's `bridge.tello.drone` (command lock) when anything else might talk to the drone; state getters (`get_*`) are safe from any thread.
- **Dog `walk`/`yaw` are xgolib speed values** (Lite ±25 / ±100), not mm or degrees.
- `scripts/link/` and its README section are the user's older, uncommitted link test: leave them out of commits unless asked.
- In the umbrella repo, commit inside the submodule first, then `git add drone-robot` in the parent to bump its pointer.
