"""
Windows-only: read adapters and adjust interface metrics.

Purpose:
    On Windows, when the Tello Wi-Fi steals the default route, demote it by
    raising its interface metric so USB Ethernet keeps internet and Tailscale.

Key terms:
    - Interface metric: A priority number on each adapter. Lower wins the
      default route when more than one adapter could carry ``0.0.0.0/0``.
    - Elevated: Administrator mode. Changing metrics requires it.

Exports:
    - is_elevated: Whether the process can change routes.
    - collect: Build ``Nic`` list from PowerShell.
    - apply_plan: Apply ``RoutePlan`` metrics when needed.

"""

from __future__ import annotations

import ctypes
import json
import subprocess
import sys
from typing import Any

from .split_route_nic import Nic, RoutePlan

_POWERSHELL_COLLECT = r"""
$ErrorActionPreference = 'Stop'
$ifaces = Get-NetIPInterface -AddressFamily IPv4
$addrs = Get-NetIPAddress -AddressFamily IPv4
$defaults = Get-NetRoute -DestinationPrefix '0.0.0.0/0' -ErrorAction SilentlyContinue
$defaultByIf = @{}
foreach ($r in $defaults) { $defaultByIf[$r.InterfaceIndex] = $true }
$addrByIf = @{}
foreach ($a in $addrs) {
  if (-not $addrByIf.ContainsKey($a.InterfaceIndex)) { $addrByIf[$a.InterfaceIndex] = @() }
  $addrByIf[$a.InterfaceIndex] += $a.IPAddress
}
$out = @()
foreach ($i in $ifaces) {
  $name = (Get-NetAdapter -InterfaceIndex $i.InterfaceIndex -ErrorAction SilentlyContinue).Name
  if (-not $name) { $name = "ifindex-$($i.InterfaceIndex)" }
  $kind = 'other'
  if ($name -match 'Wi-Fi|WLAN|Wireless') { $kind = 'wifi' }
  elseif ($name -match 'Ethernet') { $kind = 'ethernet' }
  $ips = @()
  if ($addrByIf.ContainsKey($i.InterfaceIndex)) { $ips = $addrByIf[$i.InterfaceIndex] }
  $out += [PSCustomObject]@{
    name = $name
    index = [int]$i.InterfaceIndex
    addresses = $ips
    is_default = $defaultByIf.ContainsKey($i.InterfaceIndex)
    metric = [int]$i.InterfaceMetric
    kind = $kind
  }
}
$out | ConvertTo-Json -Compress -Depth 4
"""


def is_elevated() -> bool:
    """Return True when the process has Administrator rights on Windows.

    :returns: True if metrics can be changed.
    """

    try:
        return bool(ctypes.windll.shell32.IsUserAnAdmin())  # type: ignore[attr-defined]
    except (AttributeError, OSError):
        return False


def _run_powershell(script: str) -> str:
    """Run a PowerShell script and return stdout.

    :param script: PowerShell source text.
    :returns: Captured standard output.
    :raises RuntimeError: When PowerShell fails.
    """

    result = subprocess.run(
        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", script],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"PowerShell exited {result.returncode}")
    return result.stdout.strip()


def collect() -> list[Nic]:
    """Snapshot IPv4 adapters the way Windows sees them.

    :returns: List of ``Nic`` records.
    """

    raw = _run_powershell(_POWERSHELL_COLLECT)
    if not raw:
        return []
    payload: Any = json.loads(raw)
    if isinstance(payload, dict):
        payload = [payload]
    nics: list[Nic] = []
    for row in payload:
        addresses = tuple(str(a) for a in (row.get("addresses") or []))
        nics.append(
            Nic(
                name=str(row.get("name") or ""),
                index=int(row.get("index") or -1),
                addresses=addresses,
                is_default=bool(row.get("is_default")),
                metric=int(row["metric"]) if row.get("metric") is not None else None,
                kind=str(row.get("kind") or "other"),
            )
        )
    return nics


def apply_plan(plan: RoutePlan) -> None:
    """Raise the Tello adapter metric and lower the uplink metric when required.

    :param plan: Output of ``plan_routes``.
    :raises RuntimeError: When not elevated or NIC indices are missing.
    """

    if not plan.ok or not plan.needs_metric_adjustment:
        return
    if not is_elevated():
        raise RuntimeError("routes apply on Windows requires Administrator (elevated shell).")
    if plan.tello_nic is None or plan.uplink_nic is None:
        raise RuntimeError("Route plan is missing adapter indices.")
    if plan.tello_metric is None or plan.uplink_metric is None:
        raise RuntimeError("Route plan is missing target metrics.")

    tello_idx = plan.tello_nic.index
    uplink_idx = plan.uplink_nic.index
    tello_metric = plan.tello_metric
    uplink_metric = plan.uplink_metric

    script = (
        f"Set-NetIPInterface -InterfaceIndex {uplink_idx} -InterfaceMetric {uplink_metric}; "
        f"Set-NetIPInterface -InterfaceIndex {tello_idx} -InterfaceMetric {tello_metric}"
    )
    _run_powershell(script)


def platform_name() -> str:
    """Return a short label for status output.

    :returns: ``windows``.
    """

    return "windows"
