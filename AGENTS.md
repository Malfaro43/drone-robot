# Agent guide: drone-robot

DJI RoboMaster TT (Tello Talent) controlled from the laptop over Wi-Fi with `djitellopy`.

- Use `.venv/bin/python` (Python 3.13). Python 3.14 is the system default, but don't use it for this project.
- The drone is at `192.168.10.1` in AP mode. Commands go over UDP 8889, state comes back on 8890, and video on 11111.
- Never run a flight script without the user confirming the drone is in a clear space. `--check` style connect-only runs are fine.
- Every flight script must land in a `finally` block and treat Ctrl+C as "land now".
- Keep indoor test moves small (`move_*` minimum is 20 cm; the scripts cap climbs at 50 cm).
