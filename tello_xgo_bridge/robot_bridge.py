"""
One object with ``.tello`` and ``.xgo`` peers for the homework bridge.

Purpose:
    ``TelloXgoBridge`` is the public entry point importers use. It opens both
    links together and closes them together. It does not change OS routes; run
    ``python -m tello_xgo_bridge routes`` for that helper.

Key terms:
    - Peer links: ``tello`` and ``xgo`` expose the same ``read`` / ``write``
      verbs but talk to different robots over different networks.

Exports:
    - TelloXgoBridge: Combined session handle.

"""

from __future__ import annotations

from .tello_link import TelloLink
from .xgo_link import XgoLink


class TelloXgoBridge:
    """Session that can talk to the drone and the dog from one laptop.

    :param pi_url: Tailscale HTTP base for the Pi, e.g. ``http://100.x.x.x:8080``.
    :param tello_host: Documentary drone address (UDP still targets the local Tello).
    """

    def __init__(self, pi_url: str, tello_host: str = "192.168.10.1") -> None:
        self.tello = TelloLink(host=tello_host)
        self.xgo = XgoLink(pi_url=pi_url)
        self._open = False

    def open(self) -> None:
        """Open the Tello SDK session. The XGO uses stateless HTTP and needs no open.

        :returns: None.
        """

        self.tello.open()
        self._open = True

    def close(self) -> None:
        """Stop the dog (best effort), then release the Tello SDK session.

        :returns: None.
        """

        try:
            self.xgo.stop()
        except Exception:
            pass
        self.tello.close()
        self._open = False

    def __enter__(self) -> "TelloXgoBridge":
        """``with TelloXgoBridge(...) as bridge:`` opens, and always closes (stopping the
        dog) when the block ends, even on an error or Ctrl+C. It does not land the drone:
        flight code must still land in its own ``finally``.
        """

        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
