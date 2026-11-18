"""
Console dispatch for the bridge package.

Purpose:
    Separate tiny entry logic from ``__main__.py`` so tests can call ``main``
    without the ``SystemExit`` wrapper.

Exports:
    - main: Print usage or run the routes subcommand.

"""

from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
    """Print the Python API hint or run ``routes``.

    :param argv: Arguments after the script name (default ``sys.argv[1:]``).
    :returns: Exit code.
    """

    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] == "routes":
        from .routes.split_route_cli import main as routes_main

        return routes_main(args[1:])

    print(
        "Tello + XGO bridge (Python API)\n"
        "\n"
        "  from tello_xgo_bridge import TelloXgoBridge\n"
        "  bridge = TelloXgoBridge(pi_url='http://100.x.x.x:8080')\n"
        "  bridge.open()\n"
        "  bridge.tello.read()\n"
        "  bridge.tello.write('battery?')\n"
        "  bridge.xgo.read()\n"
        "  bridge.xgo.write(walk=0, yaw=0)\n"
        "  bridge.close()\n"
        "\n"
        "Network helper (after joining Tello Wi-Fi and plugging USB Ethernet):\n"
        "  python -m tello_xgo_bridge routes\n"
        "  python -m tello_xgo_bridge routes apply\n"
        "\n"
        "Read tello_xgo_bridge/README.md for the full network lesson.\n"
    )
    return 0
