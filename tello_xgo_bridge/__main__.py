"""
``python -m tello_xgo_bridge`` entry point.

Purpose:
    With no arguments, print how to import ``TelloXgoBridge`` and when to run
    the ``routes`` helper. With ``routes``, delegate to the route CLI.

Key terms:
    - Module run: ``python -m package`` executes this file.

Exports:
    - main: Dispatch argv.
"""

from __future__ import annotations

from .main_entry import main

raise SystemExit(main())
