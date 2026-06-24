# -*- coding: utf-8 -*-
"""
video_file_analysis.py

Analyse d'une vidéo importée, sans utiliser la capture caméra ni le video player.

Le point important pour la maintenabilité :
- ce module ne refait pas le tracking ;
- il lit simplement une vidéo frame par frame ;
- il crée des FramePacket ;
- il appelle AppController.tracking_adapter(packet), donc le même tracking que le live.
"""

import os
import time
import threading
import queue

import cv2

from frame_packet import FramePacket
from result_recorder import ResultRecorder


class VideoFileAnalysisWorker(threading.Thread):
    def __init__(self, controller, video_path, result_file_path, metadata=None, video_crop=None):
        threading.Thread.__init__(self)
        self.daemon = True

        self.controller = controller
        self.video_path = video_path
        self.result_file_path = result_file_path
        self.metadata = metadata or {}
        self.video_crop = video_crop

        self.stop_event = threading.Event()
        self.recorder_stop_event = threading.Event()
        self.result_queue = queue.Queue()

        self._lock = threading.Lock()
        self._status = {
            "running": False,
            "done": False,
            "stopped": False,
            "error": "",
            "video_path": video_path,
            "result_file_path": result_file_path,
            "frame_id": 0,
            "total_frames": 0,
            "progress": 0.0,
            "fps": 0.0,
            "width": 0,
            "height": 0,
            "rows_written": 0,
            "preview_frame_id": -1,
        }

        # Résultats gardés en mémoire pour navigation/review après analyse.
        # Clé = frame_id, valeur = TrackingResult.
        self.results_by_frame = {}

    def run(self):
        cap = None
        recorder = None

        try:
            self._update_status(running=True, done=False, stopped=False, error="")

            cap = cv2.VideoCapture(self.video_path)

            if not cap.isOpened():
                raise RuntimeError("Impossible d'ouvrir la vidéo : {}".format(self.video_path))

            fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

            self.metadata.update({
                "source": "video_file",
                "analysis_source": "video_file",
                "video_path": self.video_path,
                "video_name": os.path.basename(self.video_path),
                "framerate": round(fps, 3) if fps > 0 else "",
                "width": width,
                "height": height,
                "total_frames": total_frames,
            })

            self._update_status(
                total_frames=total_frames,
                fps=fps,
                width=width,
                height=height,
            )

            recorder = ResultRecorder(
                result_queue=self.result_queue,
                stop_event=self.recorder_stop_event,
                file_path=self.result_file_path,
                metadata=self.metadata,
            )
            recorder.start()

            frame_id = 0
            t0 = time.perf_counter()

            while not self.stop_event.is_set():
                ok, frame = cap.read()

                if not ok or frame is None:
                    break

                if len(frame.shape) > 2:
                    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    image = frame

                image = self._apply_video_crop(image)

                if fps > 0:
                    timestamp = frame_id / fps
                else:
                    timestamp = time.perf_counter() - t0

                try:
                    okr_state = self.controller.get_state()
                except Exception:
                    okr_state = None

                packet = FramePacket(
                    frame_id=frame_id,
                    timestamp=timestamp,
                    image=image,
                    okr_state=okr_state,
                    camera_timestamp=None,
                    metadata={
                        "source": "video_file",
                        "video_path": self.video_path,
                    }
                )

                # Tracking unique et partagé avec le live.
                result = self.controller.tracking_adapter(packet)

                with self._lock:
                    self.results_by_frame[frame_id] = result

                self.result_queue.put(result)

                frame_id += 1

                if frame_id == 1 or frame_id % 10 == 0:
                    progress = 0.0
                    if total_frames > 0:
                        progress = min(100.0, (frame_id / float(total_frames)) * 100.0)

                    self._update_status(
                        frame_id=frame_id,
                        progress=progress,
                        rows_written=getattr(recorder, "rows_written", 0),
                        preview_frame_id=max(0, frame_id - 1),
                    )

            self.recorder_stop_event.set()

            if recorder is not None:
                recorder.join(timeout=30.0)

            rows_written = getattr(recorder, "rows_written", 0) if recorder is not None else 0

            if self.stop_event.is_set():
                self._update_status(
                    running=False,
                    done=False,
                    stopped=True,
                    frame_id=frame_id,
                    progress=(frame_id / float(total_frames) * 100.0) if total_frames else 0.0,
                    rows_written=rows_written,
                )
            else:
                self._update_status(
                    running=False,
                    done=True,
                    stopped=False,
                    frame_id=frame_id,
                    progress=100.0,
                    rows_written=rows_written,
                )

        except Exception as exc:
            self.recorder_stop_event.set()

            try:
                if recorder is not None:
                    recorder.join(timeout=5.0)
            except Exception:
                pass

            self._update_status(
                running=False,
                done=False,
                stopped=False,
                error=str(exc),
            )

        finally:
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass

    def _apply_video_crop(self, image):
        """
        Applique le crop choisi dans l'interface.

        Les ROIs/arcs sont placés sur l'image croppée affichée.
        Il faut donc analyser la même zone, avec les mêmes coordonnées.
        """
        if image is None:
            return image

        crop = self.video_crop

        if not crop:
            return image

        try:
            img_h, img_w = image.shape[:2]

            x = int(crop.get("x", 0))
            y = int(crop.get("y", 0))
            w = int(crop.get("width", img_w))
            h = int(crop.get("height", img_h))

            x = max(0, min(x, img_w - 1))
            y = max(0, min(y, img_h - 1))
            w = max(1, min(w, img_w - x))
            h = max(1, min(h, img_h - y))

            return image[y:y + h, x:x + w]

        except Exception:
            return image

    def stop(self):
        self.stop_event.set()
        self.recorder_stop_event.set()

    def _update_status(self, **kwargs):
        with self._lock:
            self._status.update(kwargs)

    def get_status(self):
        with self._lock:
            return dict(self._status)

    def get_result(self, frame_id):
        """
        Retourne le résultat de tracking pour une frame donnée.
        Utilisé par l'interface pour naviguer après l'analyse.
        """
        try:
            frame_id = int(frame_id)
        except Exception:
            return None

        with self._lock:
            return self.results_by_frame.get(frame_id)
