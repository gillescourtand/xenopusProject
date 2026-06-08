# -*- coding: utf-8 -*-
"""
result_recorder.py

Écriture des résultats de tracking dans un fichier CSV.
Une ligne CSV correspond à un TrackingResult.
"""

import csv
import os
import threading
import time
import queue


class ResultRecorder(threading.Thread):
    def __init__(self, result_queue, stop_event, file_path=None, flush_every=50):
        threading.Thread.__init__(self)

        self.daemon = True

        # File contenant les résultats produits par le TrackingWorker.
        self.result_queue = result_queue

        # Événement partagé permettant d'arrêter proprement le thread.
        self.stop_event = stop_event

        # Chemin du fichier CSV de sortie.
        self.file_path = file_path

        # Nombre de lignes écrites avant de forcer un flush disque.
        self.flush_every = flush_every

        # Nombre total de résultats écrits.
        self.rows_written = 0

        # Dernier timestamp écrit.
        # Sert à calculer l'écart de temps entre deux frames successives.
        self.last_timestamp = None

        # Indique si le fichier est actuellement ouvert.
        self.running = False

    def run(self):
        self.running = True

        if self.file_path is None:
            self.file_path = self._default_file_path()

        with open(self.file_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file, delimiter=";")

            writer.writerow([
                "frame_id",
                "timestamp",
                "eye1_angle",
                "eye2_angle",
                "eye1_y",
                "eye2_y",
                "tail_angle",
                "tail_x",
                "tail_y",
                "okr_active",
                "okr_speed",
                "okr_direction",
                "okr_frequency",
                "okr_pattern",
                "valid",
                "error",
            ])

            while not self.stop_event.is_set() or not self.result_queue.empty():
                try:
                    result = self.result_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                okr = result.okr_state or {}

                writer.writerow([
                    self._clean_value(result.frame_id),
                    self._clean_value(result.timestamp),
                    self._clean_value(result.eye1_angle),
                    self._clean_value(result.eye2_angle),
                    self._clean_value(result.eye1_y),
                    self._clean_value(result.eye2_y),
                    self._clean_value(result.tail_angle),
                    self._clean_value(result.tail_x),
                    self._clean_value(result.tail_y),
                    self._clean_value(okr.get("active", False)),
                    self._clean_value(okr.get("speed", 0)),
                    self._clean_value(okr.get("direction", 0)),
                    self._clean_value(okr.get("frequency", 0)),
                    self._clean_value(okr.get("pattern", "")),
                    self._clean_value(result.valid),
                    self._clean_value(result.error),
                ])

                self.rows_written += 1

                if self.rows_written % self.flush_every == 0:
                    csv_file.flush()

        self.running = False

    def _clean_value(self, value):
        """
        Nettoie les valeurs avant écriture CSV.
        Les valeurs None sont écrites en NaN pour éviter les cellules vides.
        """
        if value is None:
            return "NaN"

        return value

    def _default_file_path(self):
        filename = "xenopus_tracking_{}.csv".format(
            time.strftime("%Y%m%d_%H%M%S")
        )
        return os.path.abspath(filename)

    def get_stats(self):
        return {
            "running": self.running,
            "rows_written": self.rows_written,
            "file_path": self.file_path,
        }