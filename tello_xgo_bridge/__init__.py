"""
Laptop bridge: one Python object for the Tello and the XGO.

First-day orientation / Purpose:
    The drone and the dog are on different networks. Your laptop sits in the
    middle. ``TelloXgoBridge`` is the single import you use in homework code:
    it exposes ``.tello`` and ``.xgo`` as peers. Each peer has ``read()`` for
    facts and ``write()`` for one instruction. You do not call ``djitellopy``,
    ``urllib``, or shell route commands from homework; this package hides them.

    Before Python can reach both machines at once, the laptop needs two paths:
    built-in Wi-Fi on the Tello, and USB Ethernet into a jack with internet.
    See ``README.md`` for why a Wi-Fi dongle is not the shared solution on
    school MacBooks. The ``routes`` subcommand fixes Windows/macOS routing.

Core loop / mental model:
    1. Hardware: Tello Wi-Fi + USB Ethernet cable to a live jack.
    2. ``python -m tello_xgo_bridge routes apply`` (Admin / sudo) when needed.
    3. On the Pi: ``python3 main.py stream`` so ``:8080`` serves ``/stats`` and ``/command``.
    4. In Python on the laptop::

           bridge = TelloXgoBridge(pi_url="http://100.x.x.x:8080")
           bridge.open()
           bridge.tello.read()
           bridge.xgo.write(walk=0, yaw=0)
           bridge.close()

    5. Do not run ``tello_camera_stream.py`` at the same time (one Tello controller).

Key terms / vocabulary:
    - Bridge: This laptop process, not the drone radio and not Tailscale itself.
    - Tailscale: A private mesh VPN. The Pi has it; the Tello cannot install it.
    - ``pi_url``: Base URL for the dog's HTTP server, usually a ``100.x`` Tailscale IP.
    - ``read`` / ``write``: Shared verbs on both peers; payloads differ by robot.
    - Route split: Making drone traffic use Wi-Fi and internet traffic use Ethernet.

Important invariants / boundaries:
    - Gait hardware is on the Pi. ``xgo.write`` POSTs JSON; it does not load ``xgolib`` on Windows.
    - Camera MJPEG is **not** relayed here (YAGNI).
    - Route tools live in ``tello_xgo_bridge.routes``; they are not re-exported here.

Read order:
    1. ``README.md`` — interface first, then the network lesson.
    2. ``robot_bridge.py`` — ``TelloXgoBridge`` session.
    3. ``tello_link.py`` / ``xgo_link.py`` — per-robot I/O.
    4. ``routes/`` — only when both links must work simultaneously.

Exports:
    - TelloXgoBridge: Public session with ``.tello`` and ``.xgo``.
"""

from .robot_bridge import TelloXgoBridge

__all__ = ["TelloXgoBridge"]
