"""
Records what the operating system reports about one network adapter.

Purpose:
    Before this package can fix routes, it must see the laptop the same way
    Windows or macOS does. Each ``Nic`` is one adapter: a name humans see
    in settings, the IPv4 addresses currently assigned, whether this adapter
    owns the default route, and an interface metric on Windows.

    ``RoutePlan`` is not a live route table edit. It is the decision: which
    adapter should carry drone traffic, which should carry everything else,
    and whether the Tello adapter stole the default route and needs a worse
    metric.

Key terms:
    - NIC (Network Interface Card): Any network port the OS lists, including
      built-in Wi-Fi, USB Ethernet, or Tailscale's virtual adapter.
    - Default route: The rule that sends traffic to addresses that are not on
      a more specific local subnet. Tailscale and the public internet use this.
    - Interface metric: On Windows, a priority number where a lower value wins
      when two adapters could both claim the default route.

Exports:
    - Nic: One adapter snapshot.
    - RoutePlan: Planned split between Tello and uplink adapters.
    - DEFAULT_TELLO_PREFIX: Classic Tello drone LAN ``192.168.10.0/24``.
    - DEFAULT_TELLO_HOST: Drone address used in docs and optional CLI checks.

"""

from __future__ import annotations

from dataclasses import dataclass, field

DEFAULT_TELLO_PREFIX = "192.168.10.0/24"
DEFAULT_TELLO_HOST = "192.168.10.1"

# When the Tello NIC stole the default route, push it above a typical Ethernet metric.
DEFAULT_TELLO_METRIC_AFTER_APPLY = 75
DEFAULT_UPLINK_METRIC_AFTER_APPLY = 10


@dataclass(frozen=True)
class Nic:
    """One network adapter as reported by the host OS.

    :param name: Human label such as ``Wi-Fi`` or ``Ethernet``.
    :param index: OS interface index when available; else -1.
    :param addresses: IPv4 addresses in dotted-quad form, no prefix length.
    :param is_default: True when this adapter currently owns the default route.
    :param metric: Windows interface metric when known; else None.
    :param kind: Short hint such as ``wifi``, ``ethernet``, or ``other``.
    """

    name: str
    index: int
    addresses: tuple[str, ...] = field(default_factory=tuple)
    is_default: bool = False
    metric: int | None = None
    kind: str = "other"


@dataclass(frozen=True)
class RoutePlan:
    """How traffic should be split after ``routes apply``.

    :param ok: False when the laptop cannot bridge yet (missing adapter).
    :param tello_nic: Adapter with an address on the drone subnet.
    :param uplink_nic: Adapter that should keep the default route.
    :param tello_metric: Target metric for the Tello adapter when a fix is needed.
    :param uplink_metric: Target metric for the uplink when a fix is needed.
    :param needs_metric_adjustment: True when the Tello NIC must be demoted.
    :param summary: Plain-English explanation for CLI output.
    :param error: When ``ok`` is False, why planning stopped.
    """

    ok: bool
    tello_nic: Nic | None = None
    uplink_nic: Nic | None = None
    tello_metric: int | None = None
    uplink_metric: int | None = None
    needs_metric_adjustment: bool = False
    summary: str = ""
    error: str | None = None
