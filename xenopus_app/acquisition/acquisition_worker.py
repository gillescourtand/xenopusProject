# -*- coding: utf-8 -*-
"""
acquisition_worker.py

Background worker that transfers frames from the live camera acquisition layer
into the real-time processing pipeline.

The worker does not talk directly to the Basler camera. It reads frames from
``video_capture_widget.acquisition_thread.get_frame_for_analysis()`` and wraps
them into ``FramePacket`` objects before pushing them to the tracking queue.

@author: Courtand, Kadri 
"""

import queue
import threading
import time

from xenopus_app.core.frame_packet import FramePacket


class AcquisitionWorker(threading.Thread):
    """
    Move acquired camera frames into the real-time pipeline.

    Parameters
    ----------
    ui : object
        Main UI object. It must expose ``video_capture_widget``.
    video : object
        Shared video state object.
    frame_queue : queue.Queue
        Queue consumed by the tracking worker.
    display_queue : queue.Queue
        Queue consumed by the display worker.
    stop_event : threading.Event
        Shared stop flag used by all pipeline workers.
    okr_provider : object, optional
        Object exposing ``get_state()`` for the current optokinetic state.
    monitor : object, optional
        Performance monitor used to store queue sizes.
    display_every_n_frames : int
        Subsampling ratio for the display queue.
    """

    def __init__(
        self,
        ui,
        video,
        frame_queue,
        display_queue,
        stop_event,
        okr_provider=None,
        monitor=None,
        display_every_n_frames=1,
    ):
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
        """
        Read frames from the acquisition buffer until the pipeline is stopped.
        """
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
                    self._update_monitor_queue_sizes()

            except Exception as exc:
                print("AcquisitionWorker error:", exc)
                time.sleep(0.005)

        self.running = False

    def _get_next_packet(self):
        """
        Return the next available frame as a ``FramePacket``.

        Returns
        -------
        FramePacket or None
            ``None`` is returned when the live acquisition buffer is empty.
        """
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

    def _update_monitor_queue_sizes(self):
        """
        Store current pipeline queue sizes in the performance monitor.
        """
        try:
            self.monitor.set_queue_size("frame_queue", self.frame_queue.qsize())
            self.monitor.set_queue_size("display_queue", self.display_queue.qsize())
        except Exception:
            pass

    def _put_latest(self, q, item):
        """
        Push an item to a queue and replace the oldest item if the queue is full.
        """
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
        """
        Request a clean worker stop.
        """
        self.stop_event.set()

    def get_stats(self):
        """
        Return lightweight runtime statistics for the acquisition worker.
        """
        return {
            "running": self.running,
            "frame_id": self.frame_id,
            "dropped_frames_queue_full": self.dropped_frames_queue_full,
        }
