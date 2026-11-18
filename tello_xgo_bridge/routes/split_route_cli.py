"""
Argparse front end for the ``routes`` subcommand.

Purpose:
    Parse ``status`` (default) and ``apply``, plus optional overrides for the
    Tello subnet prefix. Delegates all work to ``run_split_route``.

Key terms:
    - Prefix: CIDR string for the drone LAN, default ``192.168.10.0/24``.

Exports:
    - main: Entry used by ``tello_xgo_bridge.__main__`` when argv contains ``routes``.

"""

from __future__ import annotations

import argparse
import sys

from .run_split_route import run_apply, run_status
from .split_route_nic import DEFAULT_TELLO_PREFIX


def main(argv: list[str] | None = None) -> int:
    """Run ``routes`` or ``routes apply``.

    :param argv: Arguments after the word ``routes``. None uses ``sys.argv``.
    :returns: Exit code.
    """

    parser = argparse.ArgumentParser(
        prog="python -m tello_xgo_bridge routes",
        description="Show or fix the split between Tello Wi-Fi and the uplink (USB Ethernet).",
    )
    parser.add_argument(
        "action",
        nargs="?",
        default="status",
        choices=("status", "apply"),
        help="status prints the plan (default); apply changes metrics (Admin/sudo).",
    )
    parser.add_argument(
        "--prefix",
        default=DEFAULT_TELLO_PREFIX,
        help=f"Tello IPv4 network (default {DEFAULT_TELLO_PREFIX}).",
    )
    args = parser.parse_args(argv)
    if args.action == "apply":
        return run_apply(prefix=args.prefix)
    return run_status(prefix=args.prefix)


if __name__ == "__main__":
    raise SystemExit(main())
