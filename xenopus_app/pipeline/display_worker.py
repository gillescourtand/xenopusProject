# -*- coding: utf-8 -*-
"""
display_worker.py

Qt-side display updater for the real-time pipeline.

The Qt interface must be updated from the main thread. This worker therefore
uses a QTimer instead of a Python thread. In the current hybrid architecture,
the historical camera display still draws the live image. DisplayWorker only
refreshes tracking overlays and plots, while also draining the display queue so
it cannot grow indefinitely.

@author: Courtand, Kadri 
"""

import queue
from PyQt5.QtCore import QTimer


class DisplayWorker(object):
    """Update visual tracking elements without blocking acquisition or tracking."""

    def __init__(self, pipeline, video_display_widget,
                 update_overlay_callback=None,
                 update_plot_callback=None,
                 display_fps=25,
                 plot_fps=10):
        """
        Create a Qt timer-based display worker.

        Parameters
        ----------
        pipeline : RealtimePipeline
            Pipeline owning the display queue and latest tracking result.
        video_display_widget : object
            Widget used by the historical display path.
        update_overlay_callback : callable, optional
            Function called at display rate to refresh overlays.
        update_plot_callback : callable, optional
            Function called at a lower rate to refresh plots.
        display_fps : int
            Target overlay refresh rate.
        plot_fps : int
            Target plot refresh rate.
        """
        self.pipeline = pipeline
        self.video_display_widget = video_display_widget
        self.update_overlay_callback = update_overlay_callback
        self.update_plot_callback = update_plot_callback

        self.display_interval_ms = int(1000.0 / max(1, display_fps))
        self.plot_every_n_display = max(1, int(display_fps / max(1, plot_fps)))

        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)

        self.running = False
        self.display_count = 0

    def start(self):
        """Start periodic overlay and plot updates."""
        self.running = True
        self.timer.start(self.display_interval_ms)

    def stop(self):
        """Stop periodic updates."""
        self.timer.stop()
        self.running = False

    def _tick(self):
        """
        Run one display update step.

        The live camera frame is already displayed by the legacy camera widget.
        This method only drains the display queue, updates overlays, and updates
        plots at the configured lower frequency.
        """
        self._get_latest_display_packet()

        if self.update_overlay_callback is not None:
            self.update_overlay_callback()

        self.display_count += 1

        if self.display_count % self.plot_every_n_display == 0:
            if self.update_plot_callback is not None:
                self.update_plot_callback()

    def _get_latest_display_packet(self):
        """
        Drain the display queue and return the most recent packet.

        Display only needs the latest state. Older packets can be discarded to
        prevent the UI path from slowing down the 200 fps tracking pipeline.
        """
        latest = None

        while True:
            try:
                latest = self.pipeline.display_queue.get_nowait()
            except queue.Empty:
                break
            except Exception:
                break

        return latest

    def _show_frame(self, image):
        """
        Display one image in the video widget.

        This helper is kept as a fallback for a future fully migrated display
        path. It is not used in the current hybrid display mode.
        """
        if image is None:
            return

        try:
            if hasattr(self.video_display_widget, "show_frame_in_pyqtgraph"):
                self.video_display_widget.show_frame_in_pyqtgraph(image)
            elif hasattr(self.video_display_widget, "img"):
                self.video_display_widget.img.setImage(image, autoLevels=False)
        except Exception as exc:
            print("DisplayWorker show frame error:", exc)

    def get_stats(self):
        """Return lightweight display statistics."""
        return {
            "running": self.running,
            "display_count": self.display_count,
        }
