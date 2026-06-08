# -*- coding: utf-8 -*-
"""
display_worker.py

Affichage non bloquant côté Qt.

Utilise un QTimer car l'interface Qt doit rester dans le thread principal.
"""

import queue
from PyQt5.QtCore import QTimer


class DisplayWorker(object):
    def __init__(self, pipeline, video_display_widget,
                 update_overlay_callback=None,
                 update_plot_callback=None,
                 display_fps=25,
                 plot_fps=10):
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
        self.running = True
        self.timer.start(self.display_interval_ms)

    def stop(self):
        self.timer.stop()
        self.running = False

    def _tick(self):
        """
        Dans cette version hybride, la vidéo live est déjà affichée
        par video_capture_5.Display.

        Le DisplayWorker du nouveau pipeline sert uniquement à mettre à jour :
        - les overlays ;
        - les graphes ;
        - les résultats visuels liés au tracking.

        On évite donc de réafficher la frame ici, sinon on entre en conflit
        avec l'ancien affichage.
        """

        # On vide quand même la display_queue pour éviter qu'elle se remplisse
        self._get_latest_display_packet()

        if self.update_overlay_callback is not None:
            self.update_overlay_callback()

        self.display_count += 1

        if self.display_count % self.plot_every_n_display == 0:
            if self.update_plot_callback is not None:
                self.update_plot_callback()

    def _get_latest_display_packet(self):
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
        return {
            "running": self.running,
            "display_count": self.display_count,
        }
