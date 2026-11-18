"""
Pure planning: which adapter owns the drone and which owns the internet.

Purpose:
    This file never talks to the operating system. Given a list of ``Nic``
    snapshots, it decides whether the laptop can bridge the Tello and the Pi
    at the same time and whether the Tello Wi-Fi incorrectly holds the default
    route.

Key terms:
    - Connected route: Traffic for ``192.168.10.x`` should leave through the
      adapter that joined the drone. The OS usually creates this automatically.
    - Stolen default route: The Tello Wi-Fi became the preferred path for all
      internet traffic, which breaks Tailscale until metrics are corrected.

Exports:
    - plan_routes: Build a ``RoutePlan`` from current NIC snapshots.

"""

from __future__ import annotations

import ipaddress
from typing import Iterable

from .split_route_nic import (
    DEFAULT_TELLO_METRIC_AFTER_APPLY,
    DEFAULT_UPLINK_METRIC_AFTER_APPLY,
    Nic,
    RoutePlan,
)


def _ipv4_in_prefix(addresses: Iterable[str], network: ipaddress.IPv4Network) -> bool:
    """Return True when any address sits inside ``network``.

    :param addresses: Dotted-quad strings from ``Nic.addresses``.
    :param network: Tello subnet, usually ``192.168.10.0/24``.
    :returns: True if at least one address is in the subnet.
    """

    for raw in addresses:
        try:
            addr = ipaddress.IPv4Address(raw)
        except ValueError:
            continue
        if addr in network:
            return True
    return False


def _pick_tello_nic(nics: list[Nic], network: ipaddress.IPv4Network) -> Nic | None:
    """Choose the adapter that has an address on the drone LAN.

    :param nics: All adapters from ``collect``.
    :param network: Drone prefix.
    :returns: The matching ``Nic``, or None.
    """

    matches = [n for n in nics if _ipv4_in_prefix(n.addresses, network)]
    if not matches:
        return None
    # Prefer Wi-Fi when multiple NICs match (unusual, but deterministic).
    wifi = [n for n in matches if n.kind == "wifi"]
    return wifi[0] if wifi else matches[0]


def _pick_uplink_nic(nics: list[Nic], tello: Nic | None) -> Nic | None:
    """Choose the adapter that should keep the default route.

    :param nics: All adapters from ``collect``.
    :param tello: The drone adapter, excluded from candidacy.
    :returns: Uplink ``Nic``, or None.
    """

    def is_uplink_candidate(n: Nic) -> bool:
        if tello is not None and n.name == tello.name and n.index == tello.index:
            return False
        if not n.addresses:
            return False
        return True

    with_default = [n for n in nics if n.is_default and is_uplink_candidate(n)]
    if with_default:
        ethernet = [n for n in with_default if n.kind == "ethernet"]
        return ethernet[0] if ethernet else with_default[0]

    # No explicit default on uplink yet: still pick a non-Tello adapter with an IP.
    candidates = [n for n in nics if is_uplink_candidate(n)]
    ethernet = [n for n in candidates if n.kind == "ethernet"]
    if ethernet:
        return ethernet[0]
    return candidates[0] if candidates else None


def plan_routes(nics: list[Nic], prefix: str = "192.168.10.0/24") -> RoutePlan:
    """Decide how to split default routing for live Tello + Tailscale use.

    :param nics: Adapter snapshots from the platform ``collect`` function.
    :param prefix: Drone IPv4 network in CIDR form.
    :returns: A ``RoutePlan`` with ``ok`` False when bridging is impossible.
    """

    try:
        network = ipaddress.IPv4Network(prefix, strict=False)
    except ValueError as exc:
        return RoutePlan(
            ok=False,
            error=f"Invalid prefix {prefix!r}: {exc}",
            summary="Could not parse the Tello subnet.",
        )

    if not nics:
        return RoutePlan(
            ok=False,
            error="No network adapters reported.",
            summary="The OS did not return any NICs.",
        )

    tello = _pick_tello_nic(nics, network)
    if tello is None:
        return RoutePlan(
            ok=False,
            error="No adapter has an address on the Tello subnet. Join the drone Wi-Fi first.",
            summary="Missing Tello path (192.168.10.x).",
        )

    uplink = _pick_uplink_nic(nics, tello)
    if uplink is None:
        return RoutePlan(
            ok=False,
            error="No second adapter with an IP address. Plug in USB Ethernet to a live jack.",
            summary="Missing uplink path (Ethernet or other default route).",
            tello_nic=tello,
        )

    needs_fix = tello.is_default

    summary = (
        f"Tello traffic via {tello.name!r} ({', '.join(tello.addresses) or 'no IP'}). "
        f"Internet/Tailscale via {uplink.name!r} ({', '.join(uplink.addresses) or 'no IP'})."
    )
    if needs_fix:
        summary += " The Tello adapter currently owns the default route; apply will demote it."

    return RoutePlan(
        ok=True,
        tello_nic=tello,
        uplink_nic=uplink,
        needs_metric_adjustment=needs_fix,
        tello_metric=DEFAULT_TELLO_METRIC_AFTER_APPLY if needs_fix else None,
        uplink_metric=DEFAULT_UPLINK_METRIC_AFTER_APPLY if needs_fix else None,
        summary=summary,
    )
