"""Tests for the laptop-side bridge session (no drone, no Pi)."""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tello_xgo_bridge.robot_bridge import TelloXgoBridge
from tello_xgo_bridge.tello_link import TelloLink
from tello_xgo_bridge.xgo_link import XgoLink


class FakeDrone:
    """Records SDK commands; flags any two that overlap in time."""

    def __init__(self) -> None:
        self.commands: list[str] = []
        self.busy = False
        self.overlaps = 0

    def send_command_with_return(self, command: str, timeout: int = 7) -> str:
        if self.busy:
            self.overlaps += 1
        self.busy = True
        time.sleep(0.02)
        self.commands.append(command)
        self.busy = False
        return "ok"

    def send_read_command(self, command: str) -> str:      # as in djitellopy
        return self.send_command_with_return(command)

    def send_control_command(self, command: str) -> bool:
        return self.send_command_with_return(command) == "ok"

    def query_wifi_signal_noise_ratio(self) -> str:
        return self.send_command_with_return("wifi?")

    def get_battery(self) -> int:
        return 80

    def get_height(self) -> int:
        return 0

    def get_barometer(self) -> float:
        return 12345.0          # djitellopy returns cm

    def get_roll(self) -> int:
        return 0

    def get_pitch(self) -> int:
        return 0

    def get_yaw(self) -> int:
        return 0

    def end(self) -> None:
        pass


def attached() -> tuple[TelloLink, FakeDrone]:
    link, drone = TelloLink(), FakeDrone()
    link._attach(drone)
    link._open = True
    return link, drone


def test_commands_from_two_threads_never_overlap() -> None:
    link, drone = attached()

    def mission() -> None:          # code using .drone directly
        for _ in range(10):
            link.drone.send_control_command("go 20 0 0 30")

    def telemetry() -> None:        # someone polling through the link
        for _ in range(10):
            link.write("battery?")

    threads = [threading.Thread(target=mission), threading.Thread(target=telemetry)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(drone.commands) == 20
    assert drone.overlaps == 0


def test_read_sends_no_command_unless_wifi_asked() -> None:
    link, drone = attached()
    out = link.read()
    assert out["ok"] and drone.commands == []
    assert out["telemetry"]["wifi_snr"] is None
    assert abs(out["telemetry"]["baro_m"] - 123.45) < 1e-9
    link.read(wifi=True)
    assert drone.commands == ["wifi?"]


class FakeXgo(XgoLink):
    def __init__(self) -> None:
        super().__init__("http://unused")
        self.writes: list[tuple[float, float]] = []

    def write(self, walk: float = 0.0, yaw: float = 0.0) -> dict:
        self.writes.append((walk, yaw))
        return {"ok": True}


def test_close_stops_the_dog_and_with_block_closes() -> None:
    bridge = TelloXgoBridge(pi_url="http://unused")
    bridge.xgo = FakeXgo()
    bridge.tello._attach(FakeDrone())
    bridge.tello.open = lambda: setattr(bridge.tello, "_open", True)   # no real drone
    try:
        with bridge:
            raise RuntimeError("boom")
    except RuntimeError:
        pass
    assert bridge.xgo.writes == [(0.0, 0.0)]
    assert not bridge.tello._open


def test_xgo_timeout_is_short() -> None:
    assert XgoLink("http://x")._timeout <= 3.0
