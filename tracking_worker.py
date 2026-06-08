# -*- coding: utf-8 -*-
"""
tracking_worker.py

Worker dédié au tracking temps réel.
"""

import threading
import time
import queue


class TrackingWorker(threading.Thread):
    def __init__(self, frame_queue, result_queue, stop_event,
                 tracking_function, monitor=None):
        threading.Thread.__init__(self)
        self.daemon = True

        self.frame_queue = frame_queue
        self.result_queue = result_queue
        self.stop_event = stop_event
        self.tracking_function = tracking_function
        self.monitor = monitor

        self.running = False
        self.processed_frames = 0
        self.errors = 0

    def run(self):
        self.running = True

        while not self.stop_event.is_set():
            try:
                packet = self.frame_queue.get(timeout=0.02)

                if self.monitor is not None:
                    with self.monitor.timer("tracking_total"):
                        result = self.tracking_function(packet)
                    self.monitor.mark_frame_processed(packet.frame_id)
                    self.monitor.set_queue_size("result_queue", self.result_queue.qsize())
                else:
                    result = self.tracking_function(packet)

                self._put_latest(self.result_queue, result)
                self.processed_frames += 1

            except queue.Empty:
                continue

            except Exception as exc:
                self.errors += 1
                print("TrackingWorker error:", exc)
                time.sleep(0.002)

        self.running = False

    def _put_latest(self, q, item):
        try:
            q.put_nowait(item)
        except queue.Full:
            try:
                q.get_nowait()
                q.put_nowait(item)
            except Exception:
                pass

    def stop(self):
        self.stop_event.set()

    def get_stats(self):
        return {
            "running": self.running,
            "processed_frames": self.processed_frames,
            "errors": self.errors,
        }
