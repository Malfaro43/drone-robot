# Dog route follower (`scripts/dog/`)

Walks the XGO dog along the route in the drone's `map.json`, steering it from the laptop through `tello_xgo_bridge` (`xgo.write(walk, yaw)` speed commands, `xgo.read()` for the gyro heading). Nothing is installed on the dog beyond the bridge's `POST /command` handler.

For each leg the dog turns on the spot until the gyro says it faces the next waypoint, then walks for distance ÷ calibrated speed, steering back onto the line if it veers. Before moving it stands still for 5 s to measure gyro drift (subtracted afterwards), then makes a short left turn to learn which way the gyro counts.

```bash
.venv/bin/python scripts/dog/follow_route.py map.json --sim                      # simulated dog
.venv/bin/python scripts/dog/follow_route.py map.json --pi http://100.74.30.90:8080 \
    --start 0,30 --walk-speed-cm 6.5                                          # real dog
.venv/bin/python -m unittest discover -s scripts/dog
```

`--start` is where the dog stands, in cm from pad A (+x the way pad A's rocket points, +y left), facing +x.

## Stops the dog when

- the route isn't marked `route_checked` (unless `--allow-unchecked`);
- the gyro drifts more than 4°/s standing still, or the test turn barely moves it;
- the heading can't be read, a command is refused, `--max-seconds` runs out, or Ctrl+C.

## Before the first real walk

- **Measure the walking speed.** `--walk-speed-cm` is cm/s at walk value 8 and has never been measured. Every 10% it's off puts the dog roughly 10% of the distance off.
- **Stay by the dog.** If the laptop or network drops mid-walk, the Pi keeps running the last command; the bridge's `/command` handler has no timeout (the teleop receiver uses 0.5 s).
- The minimum turn speed that actually moves the legs (`MIN_TURN = 6`) is assumed, not measured.
- Route waypoint 1 is pad A, where the drone lands. Clear the drone off the pad first, or start the dog elsewhere.

## Simulated dog (`sim_dog.py`)

Same `read`/`write` interface as the bridge's `XgoLink`. Uses the measured start-up delay (~0.6 s) and turn rates (~19°/s at 20, ~32°/s at 40); the walking speed (0.8 cm/s per walk unit), veer and gyro drift are guesses you can change.
