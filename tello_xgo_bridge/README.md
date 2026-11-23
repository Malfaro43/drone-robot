# Tello and XGO bridge

This package runs on **your laptop**. It is one Python object that can **read** and **write** both the Tello drone and the XGO dog. The dog is still reached over **Tailscale**. The drone is still reached over **its own Wi-Fi**.

## The Python interface (start here)

```python
from tello_xgo_bridge import TelloXgoBridge

bridge = TelloXgoBridge(pi_url="http://100.x.x.x:8080")  # Pi Tailscale IP
bridge.open()
bridge.tello.read()                 # battery, attitude, height, …
bridge.tello.write("battery?")      # one Tello SDK text command
bridge.xgo.read()                   # same JSON as the browser stats panel
bridge.xgo.write(walk=0, yaw=0)     # stop the dog (or set walk/yaw speed values, below)
bridge.tello.drone                  # the connected djitellopy.Tello, for mission pads, go moves …
bridge.close()
```

- **`read`** returns a `dict` with an `"ok"` flag.
- **`write`** on the Tello takes one command **string**. **`write`** on the XGO takes **`walk`** and **`yaw`** numbers.
- XGO **`walk`** and **`yaw`** are xgolib **speed values**, not millimetres or degrees: the dog keeps moving at that speed until the next command. XGO Lite ranges: walk −25…25 (`chase.py` uses 8), yaw −100…100, positive turns left (about 19°/s at 20 on our dog). Values outside the range are refused, on the laptop and on the Pi.
- **`with TelloXgoBridge(...) as bridge:`** opens both and, however the block ends (error, Ctrl+C), stops the dog and closes the drone session. `bridge.close()` also stops the dog. Neither lands the drone: flight code still lands in its own `finally`.
- **Drone commands are serialised.** `djitellopy` matches replies to commands only by arrival order, so two threads sending at once can swap replies. The bridge puts one lock around every SDK command, including code that uses `bridge.tello.drone` directly. `bridge.tello.read()` uses only the drone's state stream; `read(wifi=True)` adds the Wi-Fi signal, which is a command and waits its turn. `baro_m` is in metres.
- **The Pi stops the dog if no command arrives for 1 s** (`CommandWatchdog` in `pi_xgo_bridge/remote_command.py`). While the dog moves, resend the command several times a second; if your program crashes or the network drops, the dog stops by itself.
- Only **one** program may control the Tello at a time. Do not run `tello_camera_stream.py` while `bridge.open()` is active.
- On the Pi, start the HTTP server first: `python3 main.py stream` (port 8080).

### Commands

| Command | What it does |
|--------|----------------|
| `python -m tello_xgo_bridge` | Prints the API reminder above |
| `python -m tello_xgo_bridge routes` | Shows whether Wi-Fi and Ethernet are split correctly |
| `python -m tello_xgo_bridge routes apply` | Fixes routing (Windows: Administrator; macOS: `sudo`) |

## What you buy (both Windows laptop and school MacBook)

1. **USB Ethernet adapter** with **USB-C** (MacBook) and **USB-A** (Windows), or one USB-C adapter plus a small USB-A adapter. Common chipsets (Realtek RTL8153 class) work **without** extra drivers on Windows and macOS.
2. **Ethernet patch cable** (Cat6) from the adapter to a **wall jack or router** that actually has **internet**.
3. You **do not** need a USB Wi-Fi dongle for the shared setup. On Windows it can work; on current school MacBooks it usually **never** appears as a second Wi-Fi network.

The wire does **not** plug into the dog or the drone. Only the laptop uses it.

## Why two paths exist

Think of two delivery rules on the laptop:

| Destination | Example packet | Should leave through |
|-------------|----------------|----------------------|
| Drone | Talk to `192.168.10.1` | **Built-in Wi-Fi** (you joined `TELLO-…`) |
| Everything else | Tailscale to the Pi at `100.x.x.x` | **USB Ethernet** → internet |

**Before** you join Tello Wi-Fi, Tailscale and the internet usually go out your normal school or home path.

**After** you join Tello Wi-Fi, many laptops make the drone Wi-Fi the **default route**. Then packets to the Pi try to leave through the drone. The drone is not an internet router, so the dog disappears.

`routes apply` puts the default route back on **Ethernet** while keeping the automatic rule for `192.168.10.x` on **Wi-Fi**.

### Terms in plain English

- **NIC** — Any network port the OS lists: Wi-Fi, Ethernet, Tailscale virtual adapter, etc.
- **IP address** — A number like `192.168.10.2` on your laptop while on the drone network.
- **Subnet / prefix** — `192.168.10.0/24` means “all addresses starting with 192.168.10.”
- **Default route** — The rule for addresses that are not on a more specific local subnet. The public internet and Tailscale use this rule.
- **Tailscale** — An app on real computers. The Pi has it. The Tello **cannot** install apps; it only speaks the Tello SDK over Wi-Fi.

### Trinity Guest Wi-Fi

Guest Wi-Fi **isolates** devices from each other. That is why the Pi is reached at a **Tailscale `100.x` address**, not by pinging its school Wi-Fi IP. The drone cannot join Guest in a useful way (no Tailscale, no school login portal). The Tello stays on **its own Wi-Fi**.

### School wall jack caveat

Ethernet is only helpful if that jack gives your laptop **internet**. Some jacks are locked down. If `routes` shows an uplink but Tailscale still fails, the jack may not be the path you need.

## Session checklist

1. Power on the Tello; join its Wi-Fi on the laptop.
2. Plug USB Ethernet + cable into a live jack.
3. `python -m tello_xgo_bridge routes` → expect Tello on Wi-Fi, uplink on Ethernet.
4. `python -m tello_xgo_bridge routes apply` if the plan says the Tello stole the default route.
5. On the Pi: `python3 main.py stream`.
6. Use `TelloXgoBridge` in your script.

## Tests

From the course repo root:

```bash
python -m pytest tello_xgo_bridge/tests
```
