"""Pure tests for ``plan_routes`` (no real network adapters)."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tello_xgo_bridge.routes.split_default_route import plan_routes
from tello_xgo_bridge.routes.split_route_nic import Nic


def test_tello_stole_default_route() -> None:
    nics = [
        Nic(
            name="Wi-Fi",
            index=12,
            addresses=("192.168.10.2",),
            is_default=True,
            metric=25,
            kind="wifi",
        ),
        Nic(
            name="Ethernet",
            index=8,
            addresses=("10.0.0.50",),
            is_default=False,
            metric=10,
            kind="ethernet",
        ),
    ]
    plan = plan_routes(nics)
    assert plan.ok
    assert plan.needs_metric_adjustment
    assert plan.tello_nic is not None
    assert plan.uplink_nic is not None
    assert plan.tello_nic.name == "Wi-Fi"
    assert plan.uplink_nic.name == "Ethernet"


def test_uplink_already_preferred() -> None:
    nics = [
        Nic(
            name="Wi-Fi",
            index=12,
            addresses=("192.168.10.2",),
            is_default=False,
            metric=75,
            kind="wifi",
        ),
        Nic(
            name="Ethernet",
            index=8,
            addresses=("10.0.0.50",),
            is_default=True,
            metric=10,
            kind="ethernet",
        ),
    ]
    plan = plan_routes(nics)
    assert plan.ok
    assert not plan.needs_metric_adjustment


def test_only_one_nic() -> None:
    nics = [
        Nic(name="Wi-Fi", index=12, addresses=("192.168.10.2",), is_default=True, kind="wifi"),
    ]
    plan = plan_routes(nics)
    assert not plan.ok
    assert "second" in (plan.error or "").lower()


def test_no_tello_address() -> None:
    nics = [
        Nic(name="Ethernet", index=8, addresses=("10.0.0.50",), is_default=True, kind="ethernet"),
    ]
    plan = plan_routes(nics)
    assert not plan.ok
    assert "tello" in (plan.error or "").lower()
