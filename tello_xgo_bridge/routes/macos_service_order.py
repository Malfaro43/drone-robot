"""
macOS-only: read adapters and prefer Ethernet for the default route.

Purpose:
    On macOS, service order decides which adapter wins the default route when
    several are connected. This module lists hardware ports and, when needed,
    puts Ethernet ahead of Wi-Fi so Tailscale stays up while Wi-Fi talks to
    the Tello.

Key terms:
    - Network service: The name in System Settings, such as ``Wi-Fi`` or
      ``USB 10/100/1000 LAN``, tied to a device ``en0``, ``en5``, etc.
    - Default route: Shown by ``route -n get default`` with an ``interface`` line.

Exports:
    - is_elevated: Whether the process is root (sudo).
    - collect: Build ``Nic`` list from ``ifconfig`` and ``route``.
    - apply_plan: Reorder services when the plan requires it.

"""

from __future__ import annotations

import subprocess

from .split_route_nic import Nic, RoutePlan


def is_elevated() -> bool:
    """Return True when running as root (typical after ``sudo``).

    :returns: True if service order can be changed.
    """

    try:
        import os

        return os.geteuid() == 0
    except AttributeError:
        return False


def _run(cmd: list[str]) -> str:
    """Run a command and return stdout.

    :param cmd: argv list.
    :returns: Stripped stdout.
    :raises RuntimeError: On non-zero exit.
    """

    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"{' '.join(cmd)} failed")
    return result.stdout


def _parse_hardware_ports(text: str) -> list[tuple[str, str]]:
    """Parse ``networksetup -listallhardwareports`` into (port, device) pairs.

    :param text: Command output.
    :returns: Pairs like ``("Wi-Fi", "en0")``.
    """

    ports: list[tuple[str, str]] = []
    current_port: str | None = None
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("Hardware Port:"):
            current_port = line.split(":", 1)[1].strip()
        elif line.startswith("Device:") and current_port:
            device = line.split(":", 1)[1].strip()
            ports.append((current_port, device))
            current_port = None
    return ports


def _ifconfig_addresses(device: str) -> tuple[str, ...]:
    """Read IPv4 addresses for one ``en*`` device.

    :param device: Interface name such as ``en0``.
    :returns: Dotted-quad addresses.
    """

    try:
        text = _run(["ifconfig", device])
    except RuntimeError:
        return ()
    addrs: list[str] = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("inet ") and "127.0.0.1" not in line:
            parts = line.split()
            if len(parts) >= 2:
                addrs.append(parts[1])
    return tuple(addrs)


def _default_interface() -> str | None:
    """Return the interface name that owns the default route, if any.

    :returns: ``en0`` style name, or None.
    """

    try:
        text = _run(["route", "-n", "get", "default"])
    except RuntimeError:
        return None
    for line in text.splitlines():
        if line.strip().startswith("interface:"):
            return line.split(":", 1)[1].strip()
    return None


def _kind_for_port(port_name: str) -> str:
    """Guess adapter kind from the hardware port label.

    :param port_name: Hardware port string from ``networksetup``.
    :returns: ``wifi``, ``ethernet``, or ``other``.
    """

    lower = port_name.lower()
    if "wi-fi" in lower or "wifi" in lower or "airport" in lower:
        return "wifi"
    if "ethernet" in lower or "lan" in lower or "usb" in lower:
        return "ethernet"
    return "other"


def collect() -> list[Nic]:
    """Snapshot IPv4 adapters the way macOS sees them.

    :returns: List of ``Nic`` records.
    """

    try:
        hardware = _run(["networksetup", "-listallhardwareports"])
    except RuntimeError:
        return []

    default_dev = _default_interface()
    nics: list[Nic] = []
    for index, (port_name, device) in enumerate(_parse_hardware_ports(hardware)):
        addresses = _ifconfig_addresses(device)
        nics.append(
            Nic(
                name=f"{port_name} ({device})",
                index=index,
                addresses=addresses,
                is_default=default_dev == device,
                metric=None,
                kind=_kind_for_port(port_name),
            )
        )
    return nics


def _list_services() -> list[str]:
    """Return network service names in current order.

    :returns: Service labels, one per line from ``networksetup``.
    """

    text = _run(["networksetup", "-listallnetworkservices"])
    services: list[str] = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("An asterisk") or line.startswith("*"):
            continue
        if line.startswith("*"):
            line = line.lstrip("*").strip()
        services.append(line)
    return services


def apply_plan(plan: RoutePlan) -> None:
    """Put Ethernet services before Wi-Fi when the Tello path stole the default route.

    :param plan: Output of ``plan_routes``.
    :raises RuntimeError: When not root or service names cannot be resolved.
    """

    if not plan.ok or not plan.needs_metric_adjustment:
        return
    if not is_elevated():
        raise RuntimeError("routes apply on macOS requires root (run with sudo).")
    if plan.tello_nic is None or plan.uplink_nic is None:
        raise RuntimeError("Route plan is missing adapters.")

    services = _list_services()
    if not services:
        raise RuntimeError("Could not read network service list.")

    def service_for_nic(nic: Nic) -> str | None:
        port_hint = nic.name.split(" (", 1)[0].strip()
        for svc in services:
            if svc == port_hint or port_hint in svc or svc in port_hint:
                return svc
        return None

    uplink_svc = service_for_nic(plan.uplink_nic)
    tello_svc = service_for_nic(plan.tello_nic)
    if not uplink_svc or not tello_svc:
        raise RuntimeError("Could not map adapters to networksetup service names.")

    # Build a new order: uplink first, tello second, then everything else unchanged.
    reordered: list[str] = []
    for svc in (uplink_svc, tello_svc):
        if svc not in reordered:
            reordered.append(svc)
    for svc in services:
        if svc not in reordered:
            reordered.append(svc)

    _run(["networksetup", "-ordernetworkservices"] + reordered)


def platform_name() -> str:
    """Return a short label for status output.

    :returns: ``macos``.
    """

    return "macos"
