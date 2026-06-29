# -*- coding: utf-8 -*-
"""Settings serialization helpers for the Xenopus application.

This module contains the mixin responsible for saving and loading analysis
settings from JSON files. It preserves the legacy UI state while keeping the
logic outside of the main window module.

@author: Courtand, Kadri 
"""

import json
import math
import os
import time

import numpy as np

from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer, QRectF

from xenopus_app.core.app_state import Roi
from xenopus_app.rois import define_rois

ui = None
app = None
analysisSet = None
tracking = None
video = None
image = None
target = None
varM = None
timestampList = None
roisLimb = None
stimList = None
framesBuffer = None

def set_context(**ctx):
    """Inject shared legacy objects used by the settings mixin."""
    globals().update(ctx)


class SettingsManagerMixin(object):
    """Mixin that saves and restores Xenopus analysis settings."""

    def _json_safe_number(self, value):
        """Convert Qt and NumPy numeric values to JSON-safe Python types."""
        try:
            if isinstance(value, (np.integer,)):
                return int(value)
            if isinstance(value, (np.floating,)):
                return float(value)
        except Exception:
            pass

        try:
            if isinstance(value, bool):
                return bool(value)
            if isinstance(value, int):
                return int(value)
            if isinstance(value, float):
                return float(value)
        except Exception:
            pass

        return value

    def _read_named_widget_values(self, owner):
        """Read Qt widget values exposed as attributes on an object.

        This keeps camera and UI settings without having to manually list every
        field.
        """
        values = {}

        if owner is None:
            return values

        try:
            items = vars(owner).items()
        except Exception:
            return values

        for name, widget in items:
            if name.startswith("_"):
                continue

            try:
                if isinstance(widget, QtWidgets.QSpinBox):
                    values[name] = {"type": "QSpinBox", "value": int(widget.value())}
                elif isinstance(widget, QtWidgets.QDoubleSpinBox):
                    values[name] = {"type": "QDoubleSpinBox", "value": float(widget.value())}
                elif isinstance(widget, QtWidgets.QSlider):
                    values[name] = {"type": "QSlider", "value": int(widget.value())}
                elif isinstance(widget, QtWidgets.QCheckBox):
                    values[name] = {"type": "QCheckBox", "checked": bool(widget.isChecked())}
                elif isinstance(widget, QtWidgets.QRadioButton):
                    values[name] = {"type": "QRadioButton", "checked": bool(widget.isChecked())}
                elif isinstance(widget, QtWidgets.QComboBox):
                    values[name] = {
                        "type": "QComboBox",
                        "index": int(widget.currentIndex()),
                        "text": str(widget.currentText()),
                    }
                elif isinstance(widget, QtWidgets.QLineEdit):
                    values[name] = {"type": "QLineEdit", "text": str(widget.text())}
            except Exception:
                pass

        return values

    def _apply_named_widget_values(self, owner, values):
        """Apply values previously collected by ``_read_named_widget_values``."""
        if owner is None or not isinstance(values, dict):
            return

        for name, info in values.items():
            widget = getattr(owner, name, None)
            if widget is None or not isinstance(info, dict):
                continue

            try:
                old_block = widget.blockSignals(True)
            except Exception:
                old_block = None

            try:
                if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox, QtWidgets.QSlider)):
                    if "value" in info:
                        widget.setValue(info["value"])
                elif isinstance(widget, (QtWidgets.QCheckBox, QtWidgets.QRadioButton)):
                    if "checked" in info:
                        widget.setChecked(bool(info["checked"]))
                elif isinstance(widget, QtWidgets.QComboBox):
                    text = info.get("text", None)
                    index = int(info.get("index", -1))
                    if text is not None and widget.findText(str(text)) >= 0:
                        widget.setCurrentText(str(text))
                    elif 0 <= index < widget.count():
                        widget.setCurrentIndex(index)
                elif isinstance(widget, QtWidgets.QLineEdit):
                    widget.setText(str(info.get("text", "")))
            except Exception:
                pass

            try:
                if old_block is not None:
                    widget.blockSignals(old_block)
            except Exception:
                pass

    def _tail_reference_points_to_list(self):
        """Return the root, nose, and tail reference points as JSON-safe lists."""
        try:
            data = getattr(self.mark, "data", {})
            pos = data.get("pos", None) if isinstance(data, dict) else None

            if pos is None or len(pos) < 3:
                return None

            return [[float(p[0]), float(p[1])] for p in pos[:3]]
        except Exception:
            return None

    def _set_tail_reference_points_from_list(self, points):
        """Restore root, nose, and tail reference points from a settings list."""
        try:
            if points is None or len(points) < 3:
                return False

            pos = np.array([
                [float(points[0][0]), float(points[0][1])],
                [float(points[1][0]), float(points[1][1])],
                [float(points[2][0]), float(points[2][1])],
            ], dtype=float)

            adj = np.array([[0, 1], [0, 2]])
            symbols = ['o', 'o', 'o']
            symbolBrushes = [(255, 0, 255), (255, 0, 255), (0, 255, 0)]
            lines = np.array([
                (255, 0, 255, 255, 2),
                (0, 255, 0, 255, 2),
            ], dtype=[('red', np.ubyte), ('green', np.ubyte), ('blue', np.ubyte), ('alpha', np.ubyte), ('width', float)])
            texts = ["root", "nose", "tail"]

            self.mark.size = 8
            self.mark.setData(
                pos=pos,
                adj=adj,
                pen=lines,
                size=self.mark.size,
                symbolBrush=symbolBrushes,
                symbolPen='w',
                symbol=symbols,
                pxMode=False,
                text=texts,
            )

            try:
                nose = pos[1]
                root = pos[0]
                varM.bodyAxis_Y = float(nose[1])
                varM.bodyAngle = math.atan2(float(nose[1] - root[1]), float(nose[0] - root[0])) * 180.0 / math.pi
            except Exception:
                pass

            return True
        except Exception as exc:
            print("_set_tail_reference_points_from_list error:", exc)
            return False

    def _eye_rois_to_settings(self):
        """Serialize eye ROI positions and sizes."""
        rois = []

        try:
            for roi in self.roisEye:
                x, y = roi.pos()
                w, h = roi.size()
                rois.append({
                    "pos": [float(x), float(y)],
                    "size": [float(w), float(h)],
                })
        except Exception:
            pass

        return rois

    def _clear_eye_rois(self):
        """Remove existing eye ROIs, labels, ellipses, and axis lines from the plot."""
        try:
            for label in list(getattr(self, "roisEyeLabels", [])):
                try:
                    self.videoDisplay_Widget.plotView.scene().removeItem(label)
                except Exception:
                    pass

            for roi in list(getattr(self, "roisEye", [])):
                try:
                    self.videoDisplay_Widget.plotView.removeItem(roi)
                except Exception:
                    try:
                        self.videoDisplay_Widget.plotView.scene().removeItem(roi)
                    except Exception:
                        pass

            for ellipse in list(getattr(self, "roisEllipseEye", [])):
                try:
                    self.videoDisplay_Widget.plotView.removeItem(ellipse)
                except Exception:
                    try:
                        self.videoDisplay_Widget.plotView.scene().removeItem(ellipse)
                    except Exception:
                        pass

            # Old eye axis lines must not remain after loading settings.
            # They are rebuilt from the current frame and restored thresholds.
            for line in list(getattr(self, "eyeAxeLines", [])):
                try:
                    self.videoDisplay_Widget.plotView.removeItem(line)
                except Exception:
                    try:
                        self.videoDisplay_Widget.plotView.scene().removeItem(line)
                    except Exception:
                        pass

            self.roisEye = []
            self.roisEllipseEye = []
            self.roisEyeLabels = []
        except Exception as exc:
            print("_clear_eye_rois error:", exc)

    def _restore_eye_rois_from_settings(self, rois):
        """Restore eye ROIs and their matching ellipse overlays from saved settings."""
        if not isinstance(rois, list):
            return

        try:
            self._clear_eye_rois()

            max_w = max(1, int(getattr(self.video, "width", 0) or getattr(self, "imported_video_raw_width", 1) or 1))
            max_h = max(1, int(getattr(self.video, "height", 0) or getattr(self, "imported_video_raw_height", 1) or 1))
            roi_limits = QRectF(0, 0, max_w, max_h)

            for item in rois[:2]:
                pos = item.get("pos", [0, 0]) if isinstance(item, dict) else [0, 0]
                size = item.get("size", [100, 80]) if isinstance(item, dict) else [100, 80]

                roi = Roi(
                    [float(pos[0]), float(pos[1])],
                    [float(size[0]), float(size[1])],
                    maxBounds=roi_limits,
                    centered=True,
                    pen=("b"),
                    removable=True,
                )
                roi.sigRemoveRequested.connect(self.remove_ROI)
                self.roisEye.append(roi)
                self.videoDisplay_Widget.plotView.addItem(roi)
                self.add_eye_roi_label(roi)

                ellipse = define_rois.EllipseROI_Centered_NoHandle(
                    pos=roi.pos(),
                    size=[1, 1],
                    pen=(3, 5),
                )
                self.roisEllipseEye.append(ellipse)
                self.videoDisplay_Widget.plotView.addItem(ellipse)

            self.refresh_eye_roi_labels()
        except Exception as exc:
            print("_restore_eye_rois_from_settings error:", exc)

    def _tail_arc_settings_to_dict(self):
        """Serialize R, M, and C tail arc settings."""
        settings = {}

        for label in ["R", "M", "C"]:
            roi = self.get_tail_arc_rois().get(label)
            slider = self.get_tail_arc_curve_slider(label)

            threshold_slider = {
                "R": self.threshTail_slider,
                "M": getattr(self, "threshTailM_slider", self.threshTail_slider),
                "C": getattr(self, "threshTailC_slider", self.threshTail_slider),
            }.get(label, self.threshTail_slider)

            settings[label] = {
                "enabled": bool(self.is_tail_arc_enabled(label)),
                "threshold": int(threshold_slider.value()),
                "curve": int(slider.value()) if slider is not None else 40,
                "initialized": bool(roi.initialized) if roi is not None else False,
                "geometry": roi.get_parameters() if roi is not None and roi.initialized else None,
            }

        return settings

    def _apply_tail_arc_settings(self, settings):
        """Apply saved settings to the R, M, and C tail arc controls and overlays."""
        if not isinstance(settings, dict):
            return

        threshold_sliders = {
            "R": self.threshTail_slider,
            "M": getattr(self, "threshTailM_slider", self.threshTail_slider),
            "C": getattr(self, "threshTailC_slider", self.threshTail_slider),
        }

        for label in ["R", "M", "C"]:
            item = settings.get(label, {})
            if not isinstance(item, dict):
                item = {}

            checkbox = getattr(self, "tailArc{}_checkBox".format(label), None)
            if checkbox is not None and "enabled" in item:
                checkbox.blockSignals(True)
                checkbox.setChecked(bool(item.get("enabled")))
                checkbox.blockSignals(False)

            slider = threshold_sliders.get(label)
            if slider is not None and "threshold" in item:
                slider.blockSignals(True)
                slider.setValue(int(item.get("threshold", slider.value())))
                slider.blockSignals(False)

            curve_slider = self.get_tail_arc_curve_slider(label)
            if curve_slider is not None and "curve" in item:
                curve_slider.blockSignals(True)
                curve_slider.setValue(int(item.get("curve", curve_slider.value())))
                curve_slider.blockSignals(False)

            roi = self.get_tail_arc_rois().get(label)
            geometry = item.get("geometry")

            if roi is not None and isinstance(geometry, dict):
                try:
                    roi.center = np.array(geometry.get("center", [50.0, 150.0]), dtype=float)
                    roi.inner_radius = float(geometry.get("inner_radius", 35.0))
                    roi.outer_radius = float(geometry.get("outer_radius", 95.0))
                    roi.start_angle = float(geometry.get("start_angle", -0.75))
                    roi.end_angle = float(geometry.get("end_angle", 0.75))
                    roi.initialized = bool(item.get("initialized", True))
                    roi._dirty_version = int(geometry.get("version", getattr(roi, "_dirty_version", 0)))
                    roi._last_curve_value = float(item.get("curve", 50.0))
                    roi._save_curve_reference()
                    roi.update_graph()
                except Exception as exc:
                    print("restore arc {} error:".format(label), exc)

        self.update_tail_arc_enabled_states(update_preview=False)

        try:
            self.threshTailValue_label.setNum(int(self.threshTail_slider.value()))
            self.threshTailMValue_label.setNum(int(self.threshTailM_slider.value()))
            self.threshTailCValue_label.setNum(int(self.threshTailC_slider.value()))
            self.tailArcCurveRValue_label.setNum(int(self.tailArcCurveR_slider.value()))
            self.tailArcCurveMValue_label.setNum(int(self.tailArcCurveM_slider.value()))
            self.tailArcCurveCValue_label.setNum(int(self.tailArcCurveC_slider.value()))
        except Exception:
            pass

    def _collect_analysis_settings(self):
        """Collect settings for both Real-time camera and Imported video modes.

        The same JSON file can restore either a live session or an imported-video
        session.
        """
        crop_config = self.get_imported_video_crop_config()

        return {
            "format": "xenopus_analysis_settings",
            "version": 2,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "analysis_mode": str(getattr(self, "analysis_mode", "live")),
            "video_state": {
                "path": getattr(self.video, "path", None),
                "width": int(getattr(self.video, "width", 0) or 0),
                "height": int(getattr(self.video, "height", 0) or 0),
                "fps": float(getattr(self.video, "fps", 0) or 0),
                "nb_frames": int(getattr(self.video, "nbFrames", 0) or 0),
                "features_file": getattr(self.video, "featuresFile", None),
                "pixel_format": getattr(self.video, "pixFormat", None),
                "grab_frame_rate": self._json_safe_number(getattr(self.video, "grabFrameRate", None)),
                "offset_x": self._json_safe_number(getattr(self.video, "offsetX", None)),
                "offset_y": self._json_safe_number(getattr(self.video, "offsetY", None)),
            },
            "imported_video": {
                "path": self.imported_video_path,
                "raw_width": int(getattr(self, "imported_video_raw_width", 0) or 0),
                "raw_height": int(getattr(self, "imported_video_raw_height", 0) or 0),
                "total_frames": int(getattr(self, "imported_video_total_frames", 0) or 0),
            },
            "imported_crop": {
                "enabled": bool(self.videoCropEnable_ckb.isChecked()),
                "config": crop_config,
                "slider_values": {
                    "frame_width": int(self.videoCropFrameWidth_slider.value()),
                    "frame_height": int(self.videoCropFrameHeight_slider.value()),
                    "offset_x": int(self.videoCropOffsetX_slider.value()),
                    "offset_y": int(self.videoCropOffsetY_slider.value()),
                },
            },
            "tail_reference_points": self._tail_reference_points_to_list(),
            "tail_arcs": self._tail_arc_settings_to_dict(),
            "eye_rois": self._eye_rois_to_settings(),
            "thresholds": {
                "eye1": int(self.threshEye1_slider.value()),
                "eye2": int(self.threshEye2_slider.value()),
                "tail_R": int(self.threshTail_slider.value()),
                "tail_M": int(getattr(self, "threshTailM_slider", self.threshTail_slider).value()),
                "tail_C": int(getattr(self, "threshTailC_slider", self.threshTail_slider).value()),
            },
            "analysis_controls": {
                "mode_index": int(self.manipType_comboBox.currentIndex()),
                "mode_text": str(self.manipType_comboBox.currentText()),
                "selection_eye": bool(self.selectEyes_radioButton.isChecked()),
                "selection_tail_root": bool(self.selectTailRoot_radioButton.isChecked()),
                "white_background": bool(self.whiteBgd_radioButton.isChecked()),
                "black_background": bool(self.blackBgd_radioButton.isChecked()),
                "preview_during_analysis": bool(self.previewDuringAnalysis_ckb.isChecked()),
            },
            "output": {
                "folder": self.result_save_dir,
                "stage": str(self.stageLineEdit.text()),
                "file_number": int(self.fileNumber_spinBox.value()),
            },
            "camera_ui": self._read_named_widget_values(getattr(self, "video_capture_widget", None)),
            "main_ui": self._read_named_widget_values(self),
        }

    def save_analysis_settings(self):
        """Save all settings for the current analysis mode.

        This works for both Real-time camera and Imported video.
        """
        try:
            base_dir = self.result_save_dir or os.path.dirname(self.imported_video_path or "") or os.path.dirname(getattr(self.video, "path", "") or "") or os.path.expanduser("~")
            mode = str(getattr(self, "analysis_mode", "live"))
            default_name = "xenopus_{}_settings.json".format("live" if mode == "live" else "imported")
            default_path = os.path.join(base_dir, default_name)

            file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                "Save Xenopus settings",
                default_path,
                "Xenopus settings (*.json);;JSON file (*.json)"
            )

            if not file_path:
                return

            if not file_path.lower().endswith(".json"):
                file_path += ".json"

            settings = self._collect_analysis_settings()

            with open(file_path, "w", encoding="utf-8") as settings_file:
                json.dump(settings, settings_file, indent=2, ensure_ascii=False)

            self._show_settings_status("Settings saved: {}".format(file_path))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Save settings",
                "Impossible d'enregistrer les settings : {}".format(exc)
            )

    def _show_settings_status(self, message):
        """Display a settings status message in the active UI context."""
        try:
            if getattr(self, "analysis_mode", "live") == "video":
                self.importedVideoStatus_label.setText(message)
            else:
                print(message)
        except Exception:
            print(message)

    def refresh_eye_axes_from_current_frame(self):
        """Rebuild eye ellipses and axis lines after loading settings.

        Settings store ROIs and thresholds. Eye axis lines are computed from the
        current frame, so they must be rebuilt after ROIs are restored.
        """
        try:
            if self.track_checkBox.isChecked():
                return

            frame = self.get_current_analysis_frame()

            if frame is None:
                return

            if len(getattr(self, "roisEye", [])) == 0:
                return

            # update_eyes_overlay uses the body axis for the corrected angle.
            # Refresh it from root/nose points when they are available.
            try:
                if len(self.mark.data) > 0:
                    root = self.mark.data['pos'][0]
                    nose = self.mark.data['pos'][1]
                    varM.bodyAxis_Y = float(nose[1])
                    varM.bodyAngle = math.atan2(
                        float(nose[1] - root[1]),
                        float(nose[0] - root[0])
                    ) * 180.0 / math.pi
            except Exception:
                pass

            threshold_values = [
                int(self.threshEye1_slider.value()),
                int(self.threshEye2_slider.value()),
            ]

            max_count = min(
                len(getattr(self, "roisEye", [])),
                len(getattr(self, "eyeAxeLines", [])),
                2
            )

            for roi_index in range(max_count):
                try:
                    self.update_eyes_overlay(threshold_values[roi_index], roi_index)
                except Exception as exc:
                    print("refresh eye axis {} error:".format(roi_index + 1), exc)

        except Exception as exc:
            print("refresh_eye_axes_from_current_frame error:", exc)

    def _apply_imported_video_crop_settings(self, crop_settings):
        """Restore imported-video crop controls from saved settings."""
        if not isinstance(crop_settings, dict):
            return

        try:
            self.videoCropEnable_ckb.setChecked(bool(crop_settings.get("enabled", True)))

            slider_values = crop_settings.get("slider_values", {})
            config = crop_settings.get("config") or {}

            frame_width = slider_values.get("frame_width", config.get("width", self.videoCropFrameWidth_slider.value()))
            frame_height = slider_values.get("frame_height", config.get("height", self.videoCropFrameHeight_slider.value()))
            offset_x = slider_values.get("offset_x", config.get("x", self.videoCropOffsetX_slider.value()))
            offset_y = slider_values.get("offset_y", config.get("y", self.videoCropOffsetY_slider.value()))

            self.videoCropOffsetX_slider.setValue(int(offset_x))
            self.videoCropOffsetY_slider.setValue(int(offset_y))
            self.videoCropFrameWidth_slider.setValue(int(frame_width))
            self.videoCropFrameHeight_slider.setValue(int(frame_height))
            self.update_imported_video_crop_labels()
        except Exception as exc:
            print("_apply_imported_video_crop_settings error:", exc)

    def _apply_analysis_settings(self, settings):
        """Apply a parsed Xenopus settings dictionary to the UI."""
        if not isinstance(settings, dict):
            raise ValueError("Invalid settings file.")

        fmt = settings.get("format")
        if fmt == "xenopus_imported_video_settings":
            # Backward compatibility with JSON files created before live-mode support.
            settings = dict(settings)
            settings["format"] = "xenopus_analysis_settings"
            settings["analysis_mode"] = "video"
            settings["imported_video"] = settings.get("video", {})
        elif fmt != "xenopus_analysis_settings":
            raise ValueError("This JSON file is not a Xenopus settings file.")

        requested_mode = settings.get("analysis_mode", getattr(self, "analysis_mode", "live"))
        if requested_mode not in ("live", "video"):
            requested_mode = getattr(self, "analysis_mode", "live")

        # Restore known dimensions first, so ROIs can be restored even when no camera/video is open.
        video_state = settings.get("video_state", {})
        try:
            self.video.width = int(video_state.get("width", getattr(self.video, "width", 0) or 0) or 0)
            self.video.height = int(video_state.get("height", getattr(self.video, "height", 0) or 0) or 0)
            self.video.fps = float(video_state.get("fps", getattr(self.video, "fps", 0) or 0) or 0)
            self.video.nbFrames = int(video_state.get("nb_frames", getattr(self.video, "nbFrames", 0) or 0) or 0)
            if video_state.get("path"):
                self.video.path = video_state.get("path")
            if video_state.get("features_file"):
                self.video.featuresFile = video_state.get("features_file")
        except Exception:
            pass

        # Do not call set_analysis_mode() when the requested mode is already active.
        # Otherwise DockArea recomputes splitters and can recreate an unwanted
        # separator in the video Capture panel after loading settings.
        current_mode = getattr(self, "analysis_mode", "live")
        if requested_mode != current_mode:
            try:
                self.set_analysis_mode(requested_mode, update_combo=True)
            except Exception:
                pass
        else:
            try:
                self._sync_analysis_mode_combo(requested_mode)
            except Exception:
                pass

        analysis_controls = settings.get("analysis_controls", {})
        output_settings = settings.get("output", {})

        try:
            if "mode_index" in analysis_controls:
                index = int(analysis_controls.get("mode_index", 0))
                if 0 <= index < self.manipType_comboBox.count():
                    self.manipType_comboBox.setCurrentIndex(index)
            self.selectEyes_radioButton.setChecked(bool(analysis_controls.get("selection_eye", self.selectEyes_radioButton.isChecked())))
            self.selectTailRoot_radioButton.setChecked(bool(analysis_controls.get("selection_tail_root", self.selectTailRoot_radioButton.isChecked())))
            self.whiteBgd_radioButton.setChecked(bool(analysis_controls.get("white_background", self.whiteBgd_radioButton.isChecked())))
            self.blackBgd_radioButton.setChecked(bool(analysis_controls.get("black_background", self.blackBgd_radioButton.isChecked())))
            self.previewDuringAnalysis_ckb.setChecked(bool(analysis_controls.get("preview_during_analysis", self.previewDuringAnalysis_ckb.isChecked())))
        except Exception:
            pass

        try:
            self.result_save_dir = output_settings.get("folder", self.result_save_dir)
            self.outputFolder_label.setText(self.result_save_dir if self.result_save_dir else "No folder selected")
            self.stageLineEdit.setText(str(output_settings.get("stage", self.stageLineEdit.text())))
            self.fileNumber_spinBox.setValue(int(output_settings.get("file_number", self.fileNumber_spinBox.value())))
            self.update_next_csv_preview()
        except Exception:
            pass

        # Camera/live configuration is stored as generic UI widget values.
        self._apply_named_widget_values(getattr(self, "video_capture_widget", None), settings.get("camera_ui", {}))

        # Imported video is restored only when the JSON comes from that mode or contains a video path.
        imported_settings = settings.get("imported_video", {}) or settings.get("video", {})
        video_path = imported_settings.get("path")
        loaded_video = False

        if requested_mode == "video" and video_path:
            self.imported_video_path = video_path
            self.importedVideoPath_label.setText(video_path)

            if os.path.exists(video_path):
                loaded_video = bool(self.load_imported_video_first_frame(video_path, keep_crop_values=False))
            else:
                self.importedVideoStatus_label.setText("Settings loaded, but video file not found: {}".format(video_path))

        if requested_mode == "video" and not loaded_video:
            try:
                raw_w = int(imported_settings.get("raw_width", 0) or 0)
                raw_h = int(imported_settings.get("raw_height", 0) or 0)
                if raw_w > 0 and raw_h > 0:
                    self.configure_imported_video_crop_controls(raw_w, raw_h)
            except Exception:
                pass

        self._apply_imported_video_crop_settings(settings.get("imported_crop", {}))

        if requested_mode == "video" and loaded_video:
            self.load_imported_video_first_frame(video_path, keep_crop_values=True)

        thresholds = settings.get("thresholds", {})
        try:
            self.threshEye1_slider.setValue(int(thresholds.get("eye1", self.threshEye1_slider.value())))
            self.threshEye2_slider.setValue(int(thresholds.get("eye2", self.threshEye2_slider.value())))
        except Exception:
            pass

        self._restore_eye_rois_from_settings(settings.get("eye_rois", []))
        self._set_tail_reference_points_from_list(settings.get("tail_reference_points"))
        self._apply_tail_arc_settings(settings.get("tail_arcs", {}))

        try:
            if self.selectTailRoot_radioButton.isChecked():
                self.update_tail_segment_thresh(self.threshTail_slider.value())
        except Exception:
            pass

        try:
            self.update_tail_segment_overlay()
        except Exception:
            pass

        # After loading settings, eye ROIs are restored but lines/ellipses must
        # be recomputed from the current frame.
        self.refresh_eye_axes_from_current_frame()
        try:
            QTimer.singleShot(0, self.refresh_eye_axes_from_current_frame)
            QTimer.singleShot(120, self.refresh_eye_axes_from_current_frame)
        except Exception:
            pass

        # In Real-time mode, loading settings must not change dock geometry.
        # Only keep the Imported video dock hidden, without calling
        # _restore_live_layout() or resizeDocks().
        if requested_mode == "live":
            try:
                self._set_dock_clean_visible(self.d15, False)
                self._set_dock_clean_visible(self.d11, True)
            except Exception:
                pass

    def load_analysis_settings(self):
        """Load a JSON settings file for live or imported-video analysis."""
        try:
            default_dir = self.result_save_dir or os.path.dirname(self.imported_video_path or "") or os.path.dirname(getattr(self.video, "path", "") or "") or os.path.expanduser("~")

            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Load Xenopus settings",
                default_dir,
                "Xenopus settings (*.json);;JSON file (*.json)"
            )

            if not file_path:
                return

            with open(file_path, "r", encoding="utf-8") as settings_file:
                settings = json.load(settings_file)

            self._apply_analysis_settings(settings)
            self._show_settings_status("Settings loaded: {}".format(file_path))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Load settings",
                "Impossible de charger les settings : {}".format(exc)
            )

    def save_imported_video_settings(self):
        """Compatibility wrapper for the former imported-video settings save action."""
        self.save_analysis_settings()

    def load_imported_video_settings(self):
        """Compatibility wrapper for the former imported-video settings load action."""
        self.load_analysis_settings()

