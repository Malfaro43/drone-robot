"""
Route helper: keep Tello Wi-Fi and the uplink alive at the same time.

First-day orientation / Purpose:
    Your laptop is the bridge between two worlds. The Tello only exists on its
    own little Wi-Fi island ``192.168.10.0/24``. The XGO dog lives on Tailscale,
    which rides on normal internet. When you join the drone Wi-Fi, Windows and
    macOS often make that Wi-Fi the "default route," meaning *all* traffic,
    including Tailscale, tries to leave through the drone. That cannot work.
    The drone is not a router to the internet.

    This subdirectory is **not** the whole bridge package. It is the network
    fix you run once per session: ``python -m tello_xgo_bridge routes apply``.
    The main package face is ``TelloXgoBridge`` in the parent directory, which
    talks to both robots after the paths are sane.

Core loop / mental model:
    1. Join the Tello Wi-Fi on built-in Wi-Fi.
    2. Plug USB Ethernet into a jack that has internet (see README).
    3. Run ``python -m tello_xgo_bridge routes`` to print the plan.
    4. Run ``python -m tello_xgo_bridge routes apply`` as Admin (Windows) or sudo (macOS).
    5. Import ``TelloXgoBridge`` and call ``tello.read`` / ``xgo.read`` from Python.

Key terms / vocabulary:
    - NIC: Network Interface Card. Any adapter the OS lists: ``Wi-Fi``, ``Ethernet``, etc.
    - Default route: The rule for "everyone else" that is not on a local subnet.
      Tailscale and the public internet use this path.
    - Connected route: Automatic rule for the subnet you joined. Tello traffic should
      use this on the Wi-Fi adapter without stealing the default route.
    - Interface metric (Windows): Priority number; **lower wins** the default route.
    - Service order (macOS): The first network service in the list wins the default route.
    - Uplink: USB Ethernet to a wall jack or router, not school Guest Wi-Fi on a second radio.

Important invariants / boundaries:
    - This code never joins Wi-Fi for you and never installs dongle drivers.
    - Robot gait lives in ``tello_link`` / ``xgo_link`` in the parent package.
    - Do not import route helpers from the public ``tello_xgo_bridge`` barrel.

Read order:
    1. ``../README.md`` — full network lesson and shopping list.
    2. ``split_route_nic.py`` — data shapes.
    3. ``split_default_route.py`` — pure planner.
    4. ``windows_interface_metric.py`` or ``macos_service_order.py`` — your OS.
    5. ``run_split_route.py`` — status / apply orchestration.

Exports:
    - Nic, RoutePlan: Adapter snapshot and plan.
    - plan_routes: Pure planner from NIC list.
    - DEFAULT_TELLO_PREFIX, DEFAULT_TELLO_HOST: Drone LAN defaults.
"""

from .split_default_route import plan_routes
from .split_route_nic import (
    DEFAULT_TELLO_HOST,
    DEFAULT_TELLO_PREFIX,
    Nic,
    RoutePlan,
)

__all__ = [
    "DEFAULT_TELLO_HOST",
    "DEFAULT_TELLO_PREFIX",
    "Nic",
    "RoutePlan",
    "plan_routes",
]
