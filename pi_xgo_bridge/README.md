# Pi-side bridge files

`remote_command.py` validates the laptop's `{"walk": speed, "yaw": speed}` commands (XGO Lite limits ±25 and ±100, xgolib speed units) and applies them through `CommandWatchdog`, which **stops the dog if no command arrives for 1 s**, so a crashed laptop or dropped network can't leave it walking.

The XGO repo (`ChampPhil/xgo_robot_kit_playing_fetch`, `scripts/xgo_robot/`) already has a copy of `remote_command.py` and the `POST /command` handler in `http_stream.py`. On the Pi, pull that repo and run `python3 main.py stream` (telemetry on): it serves `GET /stats` and `POST /command` for `tello_xgo_bridge`.

If you change `remote_command.py` here, copy it to `scripts/xgo_robot/remote_command.py` in the XGO repo too (the files must stay identical).