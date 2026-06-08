# -*- coding: utf-8 -*-
"""
acquisition_worker.py

Worker d'acquisition.
Il récupère les frames depuis l'acquisition existante si possible.

Priorité :
1. video_capture_widget.acquisition_thread.get_frame_for_analysis()
2. video.grabber / video.device si adaptation future
"""

import time
import threading
import queue
from frame_packet import FramePacket


class AcquisitionWorker(threading.Thread):
    def __init__(self, ui, video, frame_queue, display_queue,
                 stop_event, okr_provider=None, monitor=None,
                 display_every_n_frames=1):
        threading.Thread.__init__(self)
        self.daemon = True

        self.ui = ui
        self.video = video
        self.frame_queue = frame_queue
        self.display_queue = display_queue
        self.stop_event = stop_event
        self.okr_provider = okr_provider
        self.monitor = monitor
        self.display_every_n_frames = max(1, int(display_every_n_frames))

        self.running = False
        self.frame_id = 0
        self.dropped_frames_queue_full = 0

    def run(self):
        self.running = True

        while not self.stop_event.is_set():
            try:
                packet = self._get_next_packet()

                if packet is None:
                    time.sleep(0.001)
                    continue

                self._put_latest(self.frame_queue, packet)

                if packet.frame_id % self.display_every_n_frames == 0:
                    self._put_latest(self.display_queue, packet)

                if self.monitor is not None:
                    try:
                        self.monitor.set_queue_size("frame_queue", self.frame_queue.qsize())
                        self.monitor.set_queue_size("display_queue", self.display_queue.qsize())
                    except Exception:
                        pass

            except Exception as exc:
                print("AcquisitionWorker error:", exc)
                time.sleep(0.005)

        self.running = False

    def _get_next_packet(self):
        acquisition_thread = getattr(self.ui.video_capture_widget, "acquisition_thread", None)

        frame_data = None

        if acquisition_thread is not None and hasattr(acquisition_thread, "get_frame_for_analysis"):
            frame_data = acquisition_thread.get_frame_for_analysis()

        if frame_data is None:
            return None

        image = frame_data.get("image")
        frame_id = frame_data.get("frame_id", self.frame_id)
        timestamp = frame_data.get("timestamp", time.time())
        camera_timestamp = frame_data.get("camera_timestamp", None)

        self.frame_id = max(self.frame_id + 1, frame_id + 1)

        okr_state = None
        if self.okr_provider is not None:
            try:
                okr_state = self.okr_provider.get_state()
            except Exception:
                okr_state = None

        return FramePacket(
            frame_id=frame_id,
            timestamp=timestamp,
            image=image,
            okr_state=okr_state,
            camera_timestamp=camera_timestamp,
            metadata={"source": "video_capture_widget.acquisition_thread"},
        )

    def _put_latest(self, q, item):
        try:
            q.put_nowait(item)
        except queue.Full:
            try:
                q.get_nowait()
                q.put_nowait(item)
                self.dropped_frames_queue_full += 1
            except Exception:
                self.dropped_frames_queue_full += 1

    def stop(self):
        self.stop_event.set()

    def get_stats(self):
        return {
            "running": self.running,
            "frame_id": self.frame_id,
            "dropped_frames_queue_full": self.dropped_frames_queue_full,
        }
