# -*- coding: utf-8 -*-
"""
realtime_pipeline.py

Real-time pipeline orchestration.

This module connects the acquisition, tracking, display, CSV recording, and
performance-monitoring components. It does not implement tracking or camera
acquisition itself; it only wires the workers together and controls their
lifecycle.

@author: Courtand, Kadri 
"""

import threading
import queue

from xenopus_app.acquisition.acquisition_worker import AcquisitionWorker
from xenopus_app.pipeline.tracking_worker import TrackingWorker
from xenopus_app.io.result_recorder import ResultRecorder
from xenopus_app.pipeline.performance_monitor import PerformanceMonitor


class RealtimePipeline(object):
    """Coordinate acquisition, tracking, recording, and monitoring workers."""

    def __init__(
        self,
        ui,
        video,
        tracking_function,
        okr_provider=None,
        frame_queue_size=500,
        result_queue_size=500,
        display_queue_size=5,
        result_file_path=None,
        metadata=None
    ):
        """
        Create a real-time processing pipeline.

        Parameters
        ----------
        ui : object
            Main UI object. It provides ROI, threshold, and widget state.
        video : object
            Shared video/camera state used by the legacy application.
        tracking_function : callable
            Function called by TrackingWorker to analyze one FramePacket.
        okr_provider : object, optional
            Object exposing get_state() to attach OKR state to frames.
        frame_queue_size : int
            Maximum number of frames waiting for tracking.
        result_queue_size : int
            Maximum number of tracking results waiting for CSV writing.
        display_queue_size : int
            Maximum number of packets kept for display updates.
        result_file_path : str, optional
            Destination CSV path. If None, ResultRecorder creates one.
        metadata : dict, optional
            Metadata written at the top of the CSV file.
        """
        self.ui = ui
        self.video = video
        self.tracking_function = tracking_function
        self.okr_provider = okr_provider
        self.result_file_path = result_file_path
        self.metadata = metadata or {}

        self.stop_event = threading.Event()

        self.frame_queue = queue.Queue(maxsize=frame_queue_size)
        self.result_queue = queue.Queue(maxsize=result_queue_size)
        self.display_queue = queue.Queue(maxsize=display_queue_size)

        self.monitor = PerformanceMonitor()

        self.acquisition_worker = AcquisitionWorker(
            ui=ui,
            video=video,
            frame_queue=self.frame_queue,
            display_queue=self.display_queue,
            stop_event=self.stop_event,
            okr_provider=okr_provider,
            monitor=self.monitor,
            display_every_n_frames=1,
        )

        self.tracking_worker = TrackingWorker(
            frame_queue=self.frame_queue,
            result_queue=self.result_queue,
            stop_event=self.stop_event,
            tracking_function=tracking_function,
            monitor=self.monitor,
        )

        self.result_recorder = ResultRecorder(
            result_queue=self.result_queue,
            stop_event=self.stop_event,
            file_path=result_file_path,
            flush_every=50,
            metadata=self.metadata,
        )

        self.latest_result = None
        self._latest_lock = threading.Lock()
        self.running = False

    def start(self):
        """
        Start acquisition, tracking, and CSV recording workers."""
        if self.running:
            return

        self.stop_event.clear()
        self.monitor.reset()

        self.acquisition_worker.start()
        self.tracking_worker.start()
        self.result_recorder.start()

        self.running = True

    def stop(self, timeout=1.0):
        """
        Stop all pipeline workers cleanly.

        Parameters
        ----------
        timeout : float
            Maximum join time, in seconds, for each worker.
        """
        self.stop_event.set()

        for worker in [
            self.acquisition_worker,
            self.tracking_worker,
            self.result_recorder,
        ]:
            try:
                worker.join(timeout)
            except RuntimeError:
                pass

        self.running = False

    def set_latest_result(self, result):
        """
        Store the most recent tracking result for UI overlays.

        The CSV writer consumes result_queue independently. This value gives the
        display layer quick access to the latest result without touching the CSV
        queue.
        """
        with self._latest_lock:
            self.latest_result = result

    def get_latest_result(self):
        """Return the most recent tracking result in a thread-safe way."""
        with self._latest_lock:
            return self.latest_result

    def get_stats(self):
        """Return the current pipeline, worker, queue, and monitor statistics."""
        return {
            "running": self.running,
            "queues": {
                "frame_queue": self.frame_queue.qsize(),
                "result_queue": self.result_queue.qsize(),
                "display_queue": self.display_queue.qsize(),
            },
            "acquisition": self.acquisition_worker.get_stats(),
            "tracking": self.tracking_worker.get_stats(),
            "recorder": self.result_recorder.get_stats(),
            "monitor": self.monitor.get_summary(),
        }
