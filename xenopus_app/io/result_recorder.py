# -*- coding: utf-8 -*-
"""CSV writer for real-time and imported-video tracking results.

Each row represents one TrackingResult and uses the shared CSV schema from
``xenopus_app.io.csv_schema``.

@author: Courtand, Kadri 
"""

import csv
import os
import queue
import threading
import time

from xenopus_app.io.csv_schema import CSV_COLUMNS, CSV_DELIMITER


class ResultRecorder(threading.Thread):
    """Background CSV writer fed by the tracking result queue.

    The recorder runs in its own thread so disk writes do not block frame
    acquisition or tracking. It drains the queue until the stop event is set and
    all pending results have been written.
    """

    def __init__(self, result_queue, stop_event, file_path=None, flush_every=50, metadata=None):
        """Create a recorder for a result queue.

        Args:
            result_queue: Queue containing TrackingResult instances.
            stop_event: Shared event used to request a clean shutdown.
            file_path: Optional CSV output path. A timestamped file is created
                in the current working directory when omitted.
            flush_every: Number of rows written between explicit disk flushes.
            metadata: Optional session metadata written at the top of the CSV.
        """
        threading.Thread.__init__(self)

        self.daemon = True
        self.result_queue = result_queue
        self.stop_event = stop_event
        self.file_path = file_path
        self.metadata = metadata or {}
        self.flush_every = flush_every
        self.rows_written = 0
        self.last_timestamp = None
        self.running = False

    def run(self):
        """Write queued tracking results to the CSV file."""
        self.running = True

        if self.file_path is None:
            self.file_path = self._default_file_path()

        with open(self.file_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file, delimiter=CSV_DELIMITER)

            self._write_metadata(writer)
            writer.writerow(CSV_COLUMNS)

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
        """Write optional session metadata before the CSV header."""
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
        """Normalize values before writing them to CSV."""
        if value is None:
            return "NaN"

        return value

    def _default_file_path(self):
        """Build a timestamped CSV path in the current working directory."""
        filename = "xenopus_tracking_{}.csv".format(
            time.strftime("%Y%m%d_%H%M%S")
        )
        return os.path.abspath(filename)

    def get_stats(self):
        """Return lightweight recorder status information."""
        return {
            "running": self.running,
            "rows_written": self.rows_written,
            "file_path": self.file_path,
        }
