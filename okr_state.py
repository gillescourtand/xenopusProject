# -*- coding: utf-8 -*-
"""
okr_state.py

État courant de la stimulation optocinétique.

Chaque FramePacket récupère un snapshot de cet état au moment de l'acquisition.
"""

import threading
import time


class OKRState(object):
    def __init__(self):
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
        with self._lock:
            return dict(self._state)
