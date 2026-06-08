# -*- coding: utf-8 -*-
"""
performance_monitor.py

Mesure FPS, temps de traitement, frames manquantes et taille des buffers.
"""

import time
import threading
from contextlib import contextmanager
from collections import defaultdict, deque


class PerformanceMonitor(object):
    def __init__(self, history_size=1000):
        self.history_size = history_size
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with getattr(self, "_lock", threading.Lock()):
            self.start_time = time.time()
            self.last_frame_id = None
            self.processed_frames = 0
            self.missing_frames = 0
            self.timings = defaultdict(lambda: deque(maxlen=self.history_size))
            self.queue_sizes = defaultdict(lambda: deque(maxlen=self.history_size))

    @contextmanager
    def timer(self, name):
        t0 = time.perf_counter()
        try:
            yield
        finally:
            elapsed = time.perf_counter() - t0
            with self._lock:
                self.timings[name].append(elapsed)

    def mark_frame_processed(self, frame_id):
        with self._lock:
            if self.last_frame_id is not None:
                gap = frame_id - self.last_frame_id
                if gap > 1:
                    self.missing_frames += gap - 1

            self.last_frame_id = frame_id
            self.processed_frames += 1

    def set_queue_size(self, name, size):
        with self._lock:
            self.queue_sizes[name].append(size)

    def get_summary(self):
        with self._lock:
            elapsed = max(time.time() - self.start_time, 1e-9)
            fps = self.processed_frames / elapsed

            timings_summary = {}
            for name, values in self.timings.items():
                if values:
                    timings_summary[name] = {
                        "last_ms": values[-1] * 1000.0,
                        "avg_ms": (sum(values) / len(values)) * 1000.0,
                        "count": len(values),
                    }

            queue_summary = {}
            for name, values in self.queue_sizes.items():
                if values:
                    queue_summary[name] = {
                        "last": values[-1],
                        "max": max(values),
                    }

            return {
                "elapsed_s": elapsed,
                "processed_frames": self.processed_frames,
                "fps": fps,
                "missing_frames": self.missing_frames,
                "timings": timings_summary,
                "queues": queue_summary,
            }

    def print_summary(self):
        summary = self.get_summary()
        print("------ Performance summary ------")
        print("Processed frames:", summary["processed_frames"])
        print("FPS:", round(summary["fps"], 2))
        print("Missing frames:", summary["missing_frames"])

        for name, data in summary["timings"].items():
            print("{}: last={:.3f} ms avg={:.3f} ms".format(
                name, data["last_ms"], data["avg_ms"]
            ))

        for name, data in summary["queues"].items():
            print("{}: last={} max={}".format(
                name, data["last"], data["max"]
            ))
