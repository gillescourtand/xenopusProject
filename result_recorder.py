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
    def __init__(self, result_queue, stop_event, file_path=None, flush_every=50, metadata=None):
        threading.Thread.__init__(self)

        self.daemon = True

        # File contenant les résultats produits par le TrackingWorker.
        self.result_queue = result_queue

        # Événement partagé permettant d'arrêter proprement le thread.
        self.stop_event = stop_event

        # Chemin du fichier CSV de sortie.
        self.file_path = file_path
        self.metadata = metadata or {}

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

            self._write_metadata(writer)

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

                "tail_R_angle",
                "tail_R_x",
                "tail_R_y",
                "tail_M_angle",
                "tail_M_x",
                "tail_M_y",
                "tail_C_angle",
                "tail_C_x",
                "tail_C_y",

                "okr_active",
                "okr_pause",
                "stim_width",
                "stim_spacing",
                "stim_speed",
                "stim_switch_frequency",
                "stim_duration_cycle",
                "stim_duration_enabled",
                "stim_pattern",
                "stim_mode",
                "stim_direction",
                "stim_direction_text",

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

                    self._clean_value(getattr(result, "tail_R_angle", None)),
                    self._clean_value(getattr(result, "tail_R_x", None)),
                    self._clean_value(getattr(result, "tail_R_y", None)),
                    self._clean_value(getattr(result, "tail_M_angle", None)),
                    self._clean_value(getattr(result, "tail_M_x", None)),
                    self._clean_value(getattr(result, "tail_M_y", None)),
                    self._clean_value(getattr(result, "tail_C_angle", None)),
                    self._clean_value(getattr(result, "tail_C_x", None)),
                    self._clean_value(getattr(result, "tail_C_y", None)),

                    self._clean_value(okr.get("active", False)),
                    self._clean_value(okr.get("paused", False)),
                    self._clean_value(okr.get("width", 0)),
                    self._clean_value(okr.get("spacing", 0)),
                    self._clean_value(okr.get("speed", 0)),
                    self._clean_value(okr.get("frequency", 0)),
                    self._clean_value(okr.get("duration_cycle", 0)),
                    self._clean_value(okr.get("duration_enabled", False)),
                    self._clean_value(okr.get("pattern", "")),
                    self._clean_value(okr.get("mode", "")),
                    self._clean_value(okr.get("direction", 0)),
                    self._clean_value(okr.get("direction_text", "")),

                    self._clean_value(result.valid),
                    self._clean_value(result.error),
                ])

                self.rows_written += 1

                if self.rows_written % self.flush_every == 0:
                    csv_file.flush()

        self.running = False
        
    def _write_metadata(self, writer):
        framerate = self.metadata.get("framerate", "")
        width = self.metadata.get("width", "")
        height = self.metadata.get("height", "")
        stage = self.metadata.get("stage", "")
        created_at = self.metadata.get("created_at", "")

        if created_at != "":
            writer.writerow(["created at : " + str(created_at)])

        if stage != "":
            writer.writerow(["stage : " + str(stage)])

        if framerate != "":
            writer.writerow(["framerate : " + str(framerate) + " fps"])

        if width != "" or height != "":
            writer.writerow(["video size : " + str(width), str(height)])

        writer.writerow([])

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