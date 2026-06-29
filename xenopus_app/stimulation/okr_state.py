# -*- coding: utf-8 -*-
"""
Current optokinetic stimulation state.

Each FramePacket can store a snapshot of this state at acquisition time, which
keeps the stimulation parameters synchronized with video frames and tracking
results.

@author: Courtand, Kadri 
"""

import threading
import time


class OKRState(object):
    """Thread-safe container for the current optokinetic stimulation state."""

    def __init__(self):
        """Initialize the shared state with inactive stimulation values."""
        self._lock = threading.Lock()
        self._state = {
            "active": False,
            "speed": 0,
            "direction": 0,
            "frequency": 0,
            "pattern": None,
            "mode": None,
            "timestamp": time.time(),
        }

    def update(self, active=None, speed=None, direction=None,
               frequency=None, pattern=None, mode=None):
        """
        Update one or more stimulation values.

        Parameters left to ``None`` are not modified. The timestamp is refreshed
        at every successful update.
        """
        with self._lock:
            if active is not None:
                self._state["active"] = active
            if speed is not None:
                self._state["speed"] = speed
            if direction is not None:
                self._state["direction"] = direction
            if frequency is not None:
                self._state["frequency"] = frequency
            if pattern is not None:
                self._state["pattern"] = pattern
            if mode is not None:
                self._state["mode"] = mode

            self._state["timestamp"] = time.time()

    def snapshot(self):
        """Return a copy of the current stimulation state."""
        with self._lock:
            return dict(self._state)
