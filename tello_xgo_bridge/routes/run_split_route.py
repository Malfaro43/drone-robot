"""
Orchestrates ``routes status`` and ``routes apply`` on the host OS.

Purpose:
    Pick the Windows or macOS collector, build a ``RoutePlan``, print it for
    humans, and optionally apply metrics or service order. Re-read adapters
    after apply to report whether the default route moved off the Tello NIC.

Key terms:
    - Status: Read-only planning. Never changes the route table.
    - Apply: Mutates metrics or service order, then collects again.

Exports:
    - run_status: Print the plan for the current adapters.
    - run_apply: Apply the plan when possible, then verify.

"""

from __future__ import annotations

import sys

from .split_default_route import plan_routes
from .split_route_nic import DEFAULT_TELLO_PREFIX, RoutePlan


def _platform_module():
    """Import the collector for this operating system.

    :returns: Module with ``collect``, ``apply_plan``, and ``platform_name``.
    :raises RuntimeError: On unsupported OS.
    """

    if sys.platform == "win32":
        from . import windows_interface_metric as mod

        return mod
    if sys.platform == "darwin":
        from . import macos_service_order as mod

        return mod
    raise RuntimeError(f"routes helper supports Windows and macOS only (not {sys.platform!r}).")


def _print_plan(plan: RoutePlan, prefix: str) -> int:
    """Write a human-readable summary to stdout.

    :param plan: Planned split.
    :param prefix: Tello CIDR shown in the header.
    :returns: Process exit code (0 ok, 1 failure).
    """

    print(f"Tello prefix: {prefix}")
    if not plan.ok:
        print(f"Status: cannot bridge — {plan.error or plan.summary}")
        return 1
    print(f"Status: {plan.summary}")
    if plan.needs_metric_adjustment:
        print(
            "Fix needed: demote the Tello adapter so the uplink keeps the default route. "
            "Run: python -m tello_xgo_bridge routes apply"
        )
    else:
        print("Fix needed: no (default route already looks correct).")
    return 0


def run_status(prefix: str = DEFAULT_TELLO_PREFIX) -> int:
    """Collect NICs and print the route plan without changing anything.

    :param prefix: Drone IPv4 network.
    :returns: Exit code.
    """

    mod = _platform_module()
    print(f"Platform: {mod.platform_name()}")
    nics = mod.collect()
    plan = plan_routes(nics, prefix=prefix)
    return _print_plan(plan, prefix)


def run_apply(prefix: str = DEFAULT_TELLO_PREFIX) -> int:
    """Apply metrics or service order when the plan says the Tello NIC stole default.

    :param prefix: Drone IPv4 network.
    :returns: Exit code.
    """

    mod = _platform_module()
    print(f"Platform: {mod.platform_name()}")
    before = mod.collect()
    plan = plan_routes(before, prefix=prefix)
    code = _print_plan(plan, prefix)
    if not plan.ok:
        return code

    if not plan.needs_metric_adjustment:
        print("Apply: skipped (no change required).")
        return 0

    try:
        mod.apply_plan(plan)
    except RuntimeError as exc:
        print(f"Apply failed: {exc}", file=sys.stderr)
        return 1

    after = mod.collect()
    verify = plan_routes(after, prefix=prefix)
    if not verify.ok:
        print("Apply: finished, but verification could not re-plan routes.")
        return 1
    # Success means the Tello adapter no longer owns the default route.
    tello_default = bool(verify.tello_nic and verify.tello_nic.is_default)
    if tello_default:
        print("Apply: finished, but the Tello adapter still owns the default route.")
        return 1
    print("Apply: finished. Default route should now use the uplink adapter.")
    print(f"After: {verify.summary}")
    return 0
