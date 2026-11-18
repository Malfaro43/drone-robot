# Pi-side bridge files

Copy these into your **`xgo_robot`** package on the Raspberry Pi (same folder as `http_stream.py`).

1. Copy `remote_command.py` into `~/xgo_robot/remote_command.py`.
2. In `http_stream.py`:
   - Add `from .remote_command import apply_parsed_command, parse_remote_command`
   - Add `do_POST` for `POST /command` and helper `_write_json` (see the Advanced CS Topics `xgo_robot/http_stream.py` in the coursework repo, or re-copy that file from a Pi-synced tree).

After that, `python3 main.py stream` serves `GET /stats` and `POST /command` for `tello_xgo_bridge` on the laptop.