# -*- coding: utf-8 -*-
"""
app_controller.py

Pont entre MotionAnalysis_Xenopus_v17d.py et la nouvelle architecture temps réel.
"""

import os
import time

from realtime_pipeline import RealtimePipeline
from display_worker import DisplayWorker
from okr_state import OKRState
from tracking_opencv import track_frame_opencv
import numpy as np


class AppController(object):
    def __init__(self, ui, video, varM):
        self.ui = ui
        self.video = video
        self.varM = varM

        self.okr_state = OKRState()
        self.pipeline = None
        self.display_worker = None
        self.last_overlay_frame_id = -1
        
    def _get_spinbox_value(self, widget, attribute_name, default_value):
        """
        Récupère la valeur d'un QSpinBox depuis l'interface.
        Si le widget n'existe pas, on garde une valeur par défaut.
        """
        try:
            spinbox = getattr(widget, attribute_name, None)

            if spinbox is None:
                return default_value

            return int(spinbox.value())

        except Exception:
            return default_value
        
    def _get_spinbox_value(self, widget, attribute_name, default_value):
        """
        Récupère proprement la valeur d'un QSpinBox depuis l'interface.
        Si le widget n'existe pas, on garde une valeur par défaut.
        """
        try:
            spinbox = getattr(widget, attribute_name, None)

            if spinbox is None:
                return default_value

            return int(spinbox.value())

        except Exception:
            return default_value
        
    def _set_buffer_controls_enabled(self, enabled):
        """
        Active ou désactive les réglages de buffers pendant le tracking.
        Les tailles de buffers doivent être choisies avant le démarrage.
        """
        buffer_widget = getattr(self.ui, "video_capture_widget", None)

        if buffer_widget is None:
            return

        names = [
            "bufferSizeFrames_spinbox",
            "trackingBufferFrames_spinbox",
            "resultBufferFrames_spinbox",
            "displayBufferFrames_spinbox",
        ]

        for name in names:
            spinbox = getattr(buffer_widget, name, None)

            if spinbox is not None:
                spinbox.setEnabled(enabled)

    def start_tracking(self, result_file_path=None):
        if self.pipeline is not None and self.pipeline.running:
            return

        result_file_path = result_file_path or self._default_result_file()

        buffer_widget = getattr(self.ui, "video_capture_widget", None)

        tracking_buffer_size = self._get_spinbox_value(
            buffer_widget,
            "trackingBufferFrames_spinbox",
            500
        )

        result_buffer_size = self._get_spinbox_value(
            buffer_widget,
            "resultBufferFrames_spinbox",
            500
        )

        display_buffer_size = self._get_spinbox_value(
            buffer_widget,
            "displayBufferFrames_spinbox",
            5
        )
        
        buffer_widget = getattr(self.ui, "video_capture_widget", None)

        tracking_buffer_size = self._get_spinbox_value(
            buffer_widget,
            "trackingBufferFrames_spinbox",
            500
        )

        result_buffer_size = self._get_spinbox_value(
            buffer_widget,
            "resultBufferFrames_spinbox",
            500
        )

        display_buffer_size = self._get_spinbox_value(
            buffer_widget,
            "displayBufferFrames_spinbox",
            5
        )

        self.pipeline = RealtimePipeline(
            ui=self.ui,
            video=self.video,
            tracking_function=self.tracking_adapter,
            okr_provider=self,
            result_file_path=result_file_path,
            frame_queue_size=tracking_buffer_size,
            result_queue_size=result_buffer_size,
            display_queue_size=display_buffer_size,
        )

        self.display_worker = DisplayWorker(
            pipeline=self.pipeline,
            video_display_widget=self.ui.videoDisplay_Widget,
            update_overlay_callback=self.safe_update_overlay,
            update_plot_callback=self.safe_update_plot,
            display_fps=25,
            plot_fps=10,
        )

        self.pipeline.start()
        self._set_buffer_controls_enabled(False)
        self.display_worker.start()

        print("Realtime pipeline started")
        print("Result file:", result_file_path)

    def stop_tracking(self):
        if self.display_worker is not None:
            self.display_worker.stop()
            self.display_worker = None

        if self.pipeline is not None:
            self.pipeline.stop(timeout=1.0)

            try:
                self.pipeline.monitor.print_summary()
            except Exception:
                pass

        self._set_buffer_controls_enabled(True)
        print("Realtime pipeline stopped")

    def tracking_adapter(self, packet):
        result = track_frame_opencv(
            packet=packet,
            rois_eye=self.ui.roisEye,
            eye_thresholds=[
                self.ui.threshEye1_slider.value(),
                self.ui.threshEye2_slider.value()
            ],
            tail_threshold=self.ui.threshTail_slider.value(),
            tail_region=self.ui.regionlr.getRegion(),
            root_position=self._get_tail_root_position(),
            body_axis_y=self.varM.bodyAxis_Y,
            body_angle=self.varM.bodyAngle,
            kernel_size=self.ui.openKernel_spinbox.value()
        )

        if self.pipeline is not None:
            self.pipeline.set_latest_result(result)

        self.store_result_for_realtime(result)

        return result
    
    def store_result_for_realtime(self, result):
        """
        Stocke les données à la fréquence réelle du tracking.
        C'est cette méthode qui alimente les graphes et les listes de résultats.
        """

        try:
            if result.eye1_angle is not None and len(self.ui.roisEllipseEye) > 0:
                self.ui.roisEllipseEye[0].angleList.append([result.frame_id, result.eye1_angle])

            if result.eye2_angle is not None and len(self.ui.roisEllipseEye) > 1:
                self.ui.roisEllipseEye[1].angleList.append([result.frame_id, result.eye2_angle])

            if result.eye1_y is not None and len(self.ui.roisEllipseEye) > 0:
                self.ui.roisEllipseEye[0].yList.append([result.frame_id, result.eye1_y])

            if result.eye2_y is not None and len(self.ui.roisEllipseEye) > 1:
                self.ui.roisEllipseEye[1].yList.append([result.frame_id, result.eye2_y])

            if result.tail_angle is not None:
                self.ui.tailAngleList.append([result.frame_id, result.tail_angle])

            if result.tail_x is not None and result.tail_y is not None:
                self.ui.tailPosList.append([result.frame_id, [result.tail_x, result.tail_y]])

        except Exception as exc:
            print("store_result_for_realtime error:", exc)
    
    def _apply_eye_descriptor(self, eye_index, descriptor):
        """
        Applique visuellement l'ellipse et l'axe de l'œil.
        Reprend la logique de l'ancien update_overlay().
        """

        try:
            if descriptor is None:
                return

            if eye_index >= len(self.ui.roisEllipseEye):
                return

            if eye_index >= len(self.ui.eyeAxeLines):
                return

            roiEllipseEye = self.ui.roisEllipseEye[eye_index]

            x, y, MA, ma, angle, vx, vy, xrot, yrot = descriptor

            roiEllipseEye.descriptor = descriptor

            roiEllipseEye.setPos((x, y), update=False)
            roiEllipseEye.setSize((MA, ma), update=False)
            roiEllipseEye.setAngle(angle, update=False)
            roiEllipseEye.translate(-vx, -vy, update=False)
            roiEllipseEye.stateChanged()

            centerx = x - vx + xrot
            centery = y - vy + yrot

            self.ui.eyeAxeLines[eye_index].setPos((centerx, centery))
            self.ui.eyeAxeLines[eye_index].setAngle(angle + 90)

            try:
                self.ui.videoDisplay_Widget.plotView.addItem(
                    self.ui.eyeAxeLines[eye_index],
                    ignoreBounds=True
                )
            except Exception:
                pass

            # Recentrage léger de la ROI autour de l'œil détecté
            # Important pour que la zone suive si l'animal bouge un peu.
            if eye_index < len(self.ui.roisEye):
                roiEye = self.ui.roisEye[eye_index]
                xroi, yroi = roiEye.pos()
                wroi, hroi = roiEye.size()

                target_x = centerx - wroi / 2
                target_y = centery - hroi / 2

                max_step = 8

                dx = target_x - xroi
                dy = target_y - yroi

                dx = max(-max_step, min(max_step, dx))
                dy = max(-max_step, min(max_step, dy))

                roiEye.setPos([xroi + dx, yroi + dy], update=False)
                roiEye.stateChanged()

        except Exception as exc:
            print("_apply_eye_descriptor error:", exc)

    def safe_update_overlay(self):
        """
        Met à jour les overlays visuels à partir du dernier résultat.
        Important : les données sont stockées à 200 fps ailleurs.
        Ici, on ne fait que l'affichage à fréquence plus basse.
        """
        try:
            if self.pipeline is None:
                return

            result = self.pipeline.get_latest_result()

            if result is None:
                return

            # Mise à jour du marqueur de queue
            if result.tail_x is not None and result.tail_y is not None:
                try:
                    self.ui.mark.data['pos'][2] = [result.tail_x, result.tail_y]
                    self.ui.mark.updateGraph()
                except Exception as exc:
                    print("Erreur update tail marker:", exc)

            # Eye 1
            if len(self.ui.roisEllipseEye) > 0 and "eye1_descriptor" in result.metadata:
                self._apply_eye_descriptor(
                    eye_index=0,
                    descriptor=result.metadata["eye1_descriptor"]
                )

            # Eye 2
            if len(self.ui.roisEllipseEye) > 1 and "eye2_descriptor" in result.metadata:
                self._apply_eye_descriptor(
                    eye_index=1,
                    descriptor=result.metadata["eye2_descriptor"]
                )

        except Exception as exc:
            print("safe_update_overlay error:", exc)
            
    def _update_eye_roi_from_descriptor(self, eye_index, descriptor):
        """
        Recentre la ROI de l'œil autour du centre détecté.
        À appeler côté affichage/Qt, pas directement dans le thread tracking.
        """
        try:
            if descriptor is None:
                return

            if eye_index >= len(self.ui.roisEye):
                return

            roi = self.ui.roisEye[eye_index]
            wroi, hroi = roi.size()

            center_x = descriptor.get("center_x")
            center_y = descriptor.get("center_y")

            if center_x is None or center_y is None:
                return

            roi.setPos(
                [center_x - wroi / 2, center_y - hroi / 2],
                update=False
            )
            roi.stateChanged()

        except Exception as exc:
            print("update_eye_roi_from_descriptor error:", exc)

    def safe_update_plot(self):
        """
        Ne pas appeler ui.update_plot(), car l'ancienne méthode dépend de :
            self.analysis_thread.last_processed_id

        Or avec le nouveau pipeline, ProcessingThread n'est plus utilisé.
        On met donc les graphes à jour directement à partir des listes.
        """
        try:
            # Eye 1 angle
            if len(self.ui.roisEllipseEye) > 0:
                data_eye1 = np.asarray(self.ui.roisEllipseEye[0].angleList[-200:])
                if data_eye1.size > 0:
                    self.ui.w3.plot(data_eye1, pen=self.ui.penCyan, clear=True)

            # Eye 2 angle
            if len(self.ui.roisEllipseEye) > 1:
                data_eye2 = np.asarray(self.ui.roisEllipseEye[1].angleList[-200:])
                if data_eye2.size > 0:
                    self.ui.w4.plot(data_eye2, pen=self.ui.penOrange, clear=True)

            # Tail angle
            if hasattr(self.ui, "tailAngleList"):
                data_tail = np.asarray(self.ui.tailAngleList[-200:])
                if data_tail.size > 0:
                    self.ui.w5.plot(data_tail, pen=self.ui.penGreen, clear=True)

            # Eye 1 Y
            if len(self.ui.roisEllipseEye) > 0:
                data_eye1_y = np.asarray(self.ui.roisEllipseEye[0].yList[-200:])
                if data_eye1_y.size > 0:
                    self.ui.w6.plot(data_eye1_y, pen=self.ui.penCyan, clear=True)

            # Eye 2 Y
            if len(self.ui.roisEllipseEye) > 1:
                data_eye2_y = np.asarray(self.ui.roisEllipseEye[1].yList[-200:])
                if data_eye2_y.size > 0:
                    self.ui.w7.plot(data_eye2_y, pen=self.ui.penOrange, clear=True)

        except Exception as exc:
            print("safe_update_plot error:", exc)

    def get_state(self):
        self.update_okr_from_ui()
        return self.okr_state.snapshot()

    def update_okr_from_ui(self):
        try:
            optok = self.ui.optokinetic_Widget

            speed = getattr(getattr(optok, "stim_speed", None), "value", 0)
            direction = getattr(getattr(optok, "stim_direction", None), "value", 0)
            frequency = getattr(getattr(optok, "stim_switch_frequency", None), "value", 0)
            pattern = getattr(getattr(optok, "stim_pattern", None), "value", None)
            mode = getattr(getattr(optok, "stim_mode", None), "value", None)

            self.okr_state.update(
                active=(speed != 0),
                speed=speed,
                direction=direction,
                frequency=frequency,
                pattern=pattern,
                mode=mode,
            )
        except Exception:
            pass

    def _get_tail_root_position(self):
        try:
            return self.ui.mark.data['pos'][0]
        except Exception:
            return [0, 0]

    def _default_result_file(self):
        filename = "xenopus_tracking_{}.csv".format(
            time.strftime("%Y%m%d_%H%M%S")
        )
        return os.path.join(os.getcwd(), filename)

    def get_stats(self):
        if self.pipeline is None:
            return {}
        return self.pipeline.get_stats()
