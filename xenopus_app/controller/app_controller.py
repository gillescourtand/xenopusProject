# -*- coding: utf-8 -*-
"""
app_controller.py

Application controller for the Xenopus tracking GUI.

This module bridges the Qt user interface with the real-time pipeline, the
imported-video analysis worker, the OpenCV tracker, CSV output management and
OKR stimulation state capture.

@author: Courtand, Kadri 
"""

import os
import time
import re

from xenopus_app.pipeline.realtime_pipeline import RealtimePipeline
from xenopus_app.video.imported_video_worker import VideoFileAnalysisWorker
from xenopus_app.pipeline.display_worker import DisplayWorker
from xenopus_app.stimulation.okr_state import OKRState
from xenopus_app.tracking.opencv_tracker import track_frame_opencv
import numpy as np


class AppController(object):
    """Coordinate UI actions, tracking pipelines, overlays and result files.

    The controller keeps the main window lightweight by centralizing actions that
    involve several subsystems: live acquisition, frame tracking, display updates,
    imported video analysis, OKR state snapshots and output filename generation.
    """

    def __init__(self, ui, video, varM):
        """
        Initialize the controller and keep references to shared application state.
        """
        self.ui = ui
        self.video = video
        self.varM = varM

        self.okr_state = OKRState()
        self.pipeline = None
        self.display_worker = None
        self.video_file_worker = None
        self.last_overlay_frame_id = -1
        
    def _get_spinbox_value(self, widget, attribute_name, default_value):
        """
        Return an integer value from a QSpinBox-like attribute.

        Parameters
        ----------
        widget : object
            Object that may contain the requested attribute.
        attribute_name : str
            Name of the spin box attribute to read.
        default_value : int
            Value returned if the widget or attribute is unavailable.
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
        Enable or disable buffer-size controls while tracking is running.

        Buffer sizes are read when the pipeline starts, so changing them during a
        live acquisition would not be applied safely.
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
        """
        Start the real-time tracking pipeline.

        The method reads current buffer sizes from the camera panel, creates the
        pipeline, starts the display worker and locks buffer-size controls until
        tracking stops.
        """
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

        metadata = self._get_result_metadata()
        self.pipeline = RealtimePipeline(
            ui=self.ui,
            video=self.video,
            tracking_function=self.tracking_adapter,
            okr_provider=self,
            result_file_path=result_file_path,
            frame_queue_size=tracking_buffer_size,
            result_queue_size=result_buffer_size,
            display_queue_size=display_buffer_size,
            metadata=metadata,
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
        """
        Stop live tracking and restore UI controls.
        """
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
        """
        Convert a FramePacket into a TrackingResult using the OpenCV tracker.

        This adapter gathers the current UI parameters, calls the tracker and
        stores the result for CSV writing, overlays and plots.
        """
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
            kernel_size=self.ui.openKernel_spinbox.value(),
            tail_arc_roi=self.ui.get_tail_arc_roi_params()
            if hasattr(self.ui, "get_tail_arc_roi_params") else None,
            tail_arc_rois=self.ui.get_tail_arc_roi_params_all()
            if hasattr(self.ui, "get_tail_arc_roi_params_all") else None,
            tail_thresholds=self.ui.get_tail_arc_thresholds()
            if hasattr(self.ui, "get_tail_arc_thresholds") else None
        )

        if self.pipeline is not None and getattr(self.pipeline, "running", False):
            self.pipeline.set_latest_result(result)

        self.store_result_for_realtime(result)

        return result
    
    def store_result_for_realtime(self, result):
        """
        Store tracking results at the real processing rate.

        These lists feed the real-time plots and keep one data point per tracked
        frame, independently from the lower-rate display refresh.
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

            for label in ["R", "M", "C"]:
                angle = getattr(result, "tail_{}_angle".format(label), None)
                x_pos = getattr(result, "tail_{}_x".format(label), None)
                y_pos = getattr(result, "tail_{}_y".format(label), None)

                angle_list = getattr(self.ui, "tailAngleList{}".format(label), None)
                pos_list = getattr(self.ui, "tailPosList{}".format(label), None)

                if angle is not None and angle_list is not None:
                    angle_list.append([result.frame_id, angle])

                if x_pos is not None and y_pos is not None and pos_list is not None:
                    pos_list.append([result.frame_id, [x_pos, y_pos]])

        except Exception as exc:
            print("store_result_for_realtime error:", exc)
    
    def _apply_eye_descriptor(self, eye_index, descriptor, move_roi=True):
        """
        Apply one eye ellipse and its visual axis to the image overlay.

        Parameters
        ----------
        eye_index : int
            Eye index in the UI lists.
        descriptor : sequence
            Ellipse descriptor produced by the tracker.
        move_roi : bool
            If True, gently recenter the ROI around the detected eye. This is
            used in live mode. In imported-video review mode, overlays are shown
            without moving the saved ROIs.
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

            # Gently recenter the ROI around the detected eye so the search
            # area can follow small animal movements during live tracking.
            if move_roi and eye_index < len(self.ui.roisEye):
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

    def apply_result_overlay(self, result, move_eye_roi=True):
        """
        Apply a TrackingResult to the current image overlay.

        This method is used both during live tracking and during imported-video
        review when the user navigates frame by frame.
        """
        try:
            if result is None:
                return

            # Update the three tail markers.
            tail_marker = None

            for label in ["R", "M", "C"]:
                x_pos = getattr(result, "tail_{}_x".format(label), None)
                y_pos = getattr(result, "tail_{}_y".format(label), None)

                if x_pos is not None and y_pos is not None:
                    try:
                        if hasattr(self.ui, "set_tail_arc_tracking_marker"):
                            self.ui.set_tail_arc_tracking_marker(label, [x_pos, y_pos])
                    except Exception as exc:
                        print("Erreur update tail arc marker {}:".format(label), exc)

                    # Legacy tail marker: keep the last valid detected point.
                    tail_marker = [x_pos, y_pos]
                else:
                    try:
                        if hasattr(self.ui, "set_tail_arc_tracking_marker"):
                            self.ui.set_tail_arc_tracking_marker(label, None)
                    except Exception:
                        pass

            if tail_marker is None and result.tail_x is not None and result.tail_y is not None:
                tail_marker = [result.tail_x, result.tail_y]

            if tail_marker is not None:
                try:
                    self.ui.mark.data['pos'][2] = tail_marker
                    self.ui.mark.updateGraph()
                except Exception as exc:
                    print("Erreur update tail marker:", exc)

            # Eye 1
            if len(self.ui.roisEllipseEye) > 0 and "eye1_descriptor" in result.metadata:
                self._apply_eye_descriptor(
                    eye_index=0,
                    descriptor=result.metadata["eye1_descriptor"],
                    move_roi=move_eye_roi
                )

            # Eye 2
            if len(self.ui.roisEllipseEye) > 1 and "eye2_descriptor" in result.metadata:
                self._apply_eye_descriptor(
                    eye_index=1,
                    descriptor=result.metadata["eye2_descriptor"],
                    move_roi=move_eye_roi
                )

        except Exception as exc:
            print("apply_result_overlay error:", exc)

    def safe_update_overlay(self):
        """
        Refresh visual overlays from the latest available tracking result.

        Data are stored at the real tracking rate elsewhere. This method only
        updates the visual layer at the lower display rate.
        """
        try:
            if self.pipeline is None:
                return

            result = self.pipeline.get_latest_result()

            if result is None:
                return

            self.apply_result_overlay(result, move_eye_roi=True)

        except Exception as exc:
            print("safe_update_overlay error:", exc)

    def _update_eye_roi_from_descriptor(self, eye_index, descriptor):
        """
        Recenter an eye ROI around a detected eye center.

        This helper must run on the UI/display side, not directly from the
        tracking worker thread.
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
        Update plots from the new pipeline result lists.

        The legacy ui.update_plot() method depends on
        ``self.analysis_thread.last_processed_id``. The real-time pipeline no
        longer uses ProcessingThread, so plots are refreshed directly from the
        stored result lists.
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

            # Tail angles R / M / C
            tail_series = [
                ("R", getattr(self.ui, "tailAngleListR", []), getattr(self.ui, "penTailR", self.ui.penGreen)),
                ("M", getattr(self.ui, "tailAngleListM", []), getattr(self.ui, "penTailM", self.ui.penGreen)),
                ("C", getattr(self.ui, "tailAngleListC", []), getattr(self.ui, "penTailC", self.ui.penGreen)),
            ]

            first_tail_plot = True
            for label, values, pen in tail_series:
                data_tail = np.asarray(values[-200:])
                if data_tail.size > 0:
                    self.ui.w5.plot(data_tail, pen=pen, clear=first_tail_plot, name=label)
                    first_tail_plot = False

            if first_tail_plot and hasattr(self.ui, "tailAngleList"):
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
        """
        Return a snapshot of the current optokinetic stimulation state.

        The returned dictionary is attached to each frame/result so the CSV can
        be synchronized with the stimulation parameters.
        """
        try:
            opto = getattr(self.ui, "optokinetic_Widget", None)

            if opto is None:
                return {
                    "active": False,
                    "paused": False,
                    "timestamp": time.perf_counter(),
                }

            speed = self._read_value(getattr(opto, "stim_speed", None), 0)
            window_active = False

            try:
                if hasattr(opto, "display_button"):
                    window_active = bool(opto.display_button.isChecked())
            except Exception:
                window_active = False

            try:
                thread_active = len(getattr(opto, "thread_list", [])) > 0
            except Exception:
                thread_active = False

            active = window_active or thread_active

            try:
                pause_text = opto.pause_button.text().strip().lower()
            except Exception:
                pause_text = ""

            try:
                speed_value = int(speed)
            except Exception:
                speed_value = 0

            paused = active and (speed_value == 0 or pause_text == "start")

            return {
                "active": active,
                "paused": paused,
                "timestamp": time.perf_counter(),

                "width": self._read_value(getattr(opto, "stim_width", None), 0),
                "spacing": self._read_value(getattr(opto, "stim_spacing", None), 0),
                "speed": speed,
                "frequency": self._read_value(getattr(opto, "stim_switch_frequency", None), 0),
                "duration_cycle": self._read_value(getattr(opto, "stim_duration_cycle", None), 0),
                "duration_enabled": bool(opto.stim_duration_ckb.isChecked())
                if hasattr(opto, "stim_duration_ckb") else False,

                "pattern": self._read_value(getattr(opto, "stim_pattern", None), ""),
                "mode": self._read_value(getattr(opto, "stim_mode", None), ""),

                "direction": self._read_value(getattr(opto, "stim_direction", None), 0),
                "direction_text": opto.stim_direction_input.currentText()
                if hasattr(opto, "stim_direction_input") else "",
            }

        except Exception as exc:
            print("get_state OKR error:", exc)
            return None

    def _read_value(self, value_object, default_value):
        """
        Read a raw value or an object exposing a ``value`` attribute.
        """
        try:
            if value_object is None:
                return default_value

            if hasattr(value_object, "value"):
                return value_object.value

            return value_object

        except Exception:
            return default_value

    def update_okr_from_ui(self):
        """
        Push the current optokinetic UI values into the thread-safe OKR state.
        """
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
        """
        Return the current tail-root marker position.
        """
        try:
            return self.ui.mark.data['pos'][0]
        except Exception:
            return [0, 0]

    def _default_result_file(self):
        """
        Return the default CSV path for a new acquisition.
        """
        return self.preview_next_result_file_path()
    
    def _get_output_dir(self):
        """
        Return an existing output directory, creating it when required.
        """
        output_dir = getattr(self.ui, "result_save_dir", None)

        if output_dir is None or output_dir == "":
            output_dir = os.getcwd()

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        return output_dir


    def _get_stage(self):
        """
        Return a sanitized stage label used in output file names.
        """
        stage = ""

        stage_input = getattr(self.ui, "stageLineEdit", None)

        if stage_input is not None:
            stage = stage_input.text().strip()

        if stage.lower().startswith("st"):
            stage = stage[2:]

        stage = re.sub(r"[^0-9A-Za-z]+", "", stage)

        if stage == "":
            stage = "NA"

        return stage

    def _get_requested_track_number(self):
        """
        Return the user-requested file number or None for automatic numbering.
        """
        number_input = getattr(self.ui, "fileNumber_spinBox", None)

        if number_input is None:
            return None

        number = int(number_input.value())

        # -1 = automatic numbering.
        #  0 = manual file number ..._000.csv.
        #  1 = manual file number ..._001.csv.
        if number < 0:
            return None

        return number

    def _get_next_track_number(self, output_dir, file_prefix):
        """
        Compute the next available CSV number for the selected stage and date.
        """
        max_number = -1

        try:
            print("CSV output dir:", output_dir)
            print("CSV expected prefix:", file_prefix)

            for filename in os.listdir(output_dir):
                filename_clean = filename.strip()

                if not filename_clean.lower().endswith(".csv"):
                    continue

                if not filename_clean.startswith(file_prefix):
                    continue

                number_part = filename_clean[len(file_prefix):-4]

                if not number_part.isdigit():
                    continue

                number = int(number_part)
                max_number = max(max_number, number)

                print("Existing CSV detected:", filename_clean)

        except Exception as exc:
            print("CSV numbering error:", exc)

        return max_number + 1


    def preview_next_result_filename(self):
        """
        Return only the preview CSV filename.
        """
        return os.path.basename(self.preview_next_result_file_path())

    def preview_next_result_file_path(self):
        """
        Build the full preview path for the next result CSV file.
        """
        output_dir = self._get_output_dir()
        stage = self._get_stage()

        date_prefix = time.strftime("%y%m%d")
        file_prefix = "{}-St{}_".format(date_prefix, stage)

        requested_number = self._get_requested_track_number()

        if requested_number is None:
            track_number = self._get_next_track_number(output_dir, file_prefix)
        else:
            track_number = requested_number

        filename = "{}{:03d}.csv".format(file_prefix, track_number)

        return os.path.join(output_dir, filename)

    def start_video_file_analysis(self, video_path, result_file_path=None, video_crop=None):
        """
        Start analysis of an imported video file.

        The tracking logic is not duplicated here: imported-video analysis reuses
        ``tracking_adapter()`` so live and offline processing share the same
        OpenCV tracker.
        """
        if self.pipeline is not None and self.pipeline.running:
            raise RuntimeError("Live tracking is already running.")

        if self.video_file_worker is not None:
            status = self.video_file_worker.get_status()

            if status.get("running", False):
                raise RuntimeError("A video-file analysis is already running.")

        result_file_path = result_file_path or self.preview_next_result_file_path()

        metadata = self._get_result_metadata()
        metadata["analysis_source"] = "video_file"
        metadata["video_path"] = video_path

        if video_crop:
            metadata["video_crop_x"] = video_crop.get("x", "")
            metadata["video_crop_y"] = video_crop.get("y", "")
            metadata["video_crop_width"] = video_crop.get("width", "")
            metadata["video_crop_height"] = video_crop.get("height", "")

        self.video_file_worker = VideoFileAnalysisWorker(
            controller=self,
            video_path=video_path,
            result_file_path=result_file_path,
            metadata=metadata,
            video_crop=video_crop,
        )
        self.video_file_worker.start()

        print("Video-file analysis started")
        print("Video:", video_path)
        print("Result file:", result_file_path)

    def stop_video_file_analysis(self):
        """
        Request the imported-video worker to stop.
        """
        if self.video_file_worker is not None:
            try:
                self.video_file_worker.stop()
            except Exception:
                pass

    def get_video_file_analysis_status(self):
        """
        Return the current imported-video analysis status.
        """
        if self.video_file_worker is None:
            return {
                "running": False,
                "done": False,
                "stopped": False,
                "error": "",
                "progress": 0.0,
                "frame_id": 0,
                "total_frames": 0,
                "rows_written": 0,
            }

        try:
            return self.video_file_worker.get_status()
        except Exception:
            return {
                "running": False,
                "done": False,
                "stopped": False,
                "error": "status unavailable",
                "progress": 0.0,
                "frame_id": 0,
                "total_frames": 0,
                "rows_written": 0,
            }

    def get_video_file_result(self, frame_id):
        """
        Return the stored tracking result for one imported-video frame.
        """
        try:
            if self.video_file_worker is None:
                return None

            return self.video_file_worker.get_result(frame_id)

        except Exception:
            return None

    def get_stats(self):
        """
        Return pipeline statistics when live tracking is available.
        """
        if self.pipeline is None:
            return {}
        return self.pipeline.get_stats()
    
    def _get_result_metadata(self):
        """
        Build metadata written at the beginning of the result CSV file.
        """
        try:
            framerate = ""

            try:
                framerate = self.video.device.ResultingFrameRate.GetValue()
            except Exception:
                framerate = getattr(self.video, "measuredLivefps", "")

            return {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "stage": self._get_stage(),
                "framerate": round(float(framerate), 3) if framerate != "" else "",
                "width": getattr(self.video, "width", ""),
                "height": getattr(self.video, "height", ""),
            }

        except Exception:
            return {}
