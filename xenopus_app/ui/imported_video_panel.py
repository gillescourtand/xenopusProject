# -*- coding: utf-8 -*-
"""
Imported-video loading, cropping, review, and analysis controls.

@author: Courtand, Kadri 
"""

import os
import cv2
from PyQt5 import QtWidgets

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
    """Inject shared legacy context values into this module."""
    globals().update(ctx)


class ImportedVideoPanelMixin(object):
    """Mixin implementing imported-video loading, crop, analysis, and review controls."""
    def choose_imported_video_file(self):
        """Open a file dialog and load the first frame of the selected video."""
        video_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open video file",
            os.path.expanduser("~"),
            "Video files (*.avi *.mp4 *.mov *.mkv *.mpg *.mpeg);;All files (*.*)"
        )

        if not video_path:
            return

        self.imported_video_path = video_path
        self.importedVideoPath_label.setText(video_path)
        self.importedVideoStatus_label.setText("Video selected.")
        self.show_video_file_interface()

        self.load_imported_video_first_frame(video_path)

    def load_imported_video_first_frame(self, video_path, keep_crop_values=False):
        """Load the first video frame, apply crop settings, and update UI/video state."""
        try:
            cap = cv2.VideoCapture(video_path)

            if not cap.isOpened():
                QtWidgets.QMessageBox.warning(
                    self,
                    "Video file",
                    "Impossible d'ouvrir la vidéo."
                )
                return False

            ok, frame = cap.read()

            fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

            cap.release()

            if not ok or frame is None:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Video file",
                    "Impossible de lire la première frame."
                )
                return False

            if len(frame.shape) > 2:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            if not keep_crop_values:
                self.configure_imported_video_crop_controls(width, height)

            cropped_gray = self.crop_imported_video_frame(gray)
            self.imported_video_frame = cropped_gray.copy()

            self.video.path = video_path
            self.video.width = cropped_gray.shape[1]
            self.video.height = cropped_gray.shape[0]
            self.video.fps = fps
            self.video.nbFrames = total_frames

            self.show_gray_frame_in_image_dock(cropped_gray)

            try:
                self._apply_imported_video_layout()
            except Exception:
                pass

            try:
                updater = getattr(self.video_capture_widget, "videoDisplayer_updater", None)

                if updater is not None:
                    updater.current_frame_to_display = gray.copy()
            except Exception:
                pass

            self.importedVideoStatus_label.setText(
                "Loaded: {} frames | {:.2f} fps | full {} x {} | shown {} x {}".format(
                    total_frames,
                    fps,
                    width,
                    height,
                    self.video.width,
                    self.video.height
                )
            )

            self.imported_video_total_frames = total_frames
            self.set_imported_review_limits(total_frames)

            return True

        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Video file",
                "Erreur chargement vidéo : {}".format(exc)
            )
            return False

    def get_imported_video_crop_config(self):
        """
        Return the imported-video crop in OpenCV coordinates.

        The returned dictionary contains x, y, width, and height. If cropping
        is disabled, None is returned.
        """
        try:
            if not self.videoCropEnable_ckb.isChecked():
                return None

            raw_w = int(getattr(self, "imported_video_raw_width", 0) or 0)
            raw_h = int(getattr(self, "imported_video_raw_height", 0) or 0)

            if raw_w <= 0 or raw_h <= 0:
                return None

            x = int(self.videoCropOffsetX_slider.value())
            y = int(self.videoCropOffsetY_slider.value())
            w = int(self.videoCropFrameWidth_slider.value())
            h = int(self.videoCropFrameHeight_slider.value())

            x = max(0, min(x, raw_w - 1))
            y = max(0, min(y, raw_h - 1))
            w = max(1, min(w, raw_w - x))
            h = max(1, min(h, raw_h - y))

            return {
                "x": x,
                "y": y,
                "width": w,
                "height": h,
            }

        except Exception:
            return None

    def crop_imported_video_frame(self, gray_frame):
        """
        Apply the current crop settings to a grayscale frame.
        """
        if gray_frame is None:
            return gray_frame

        crop = self.get_imported_video_crop_config()

        if not crop:
            return gray_frame

        try:
            x = int(crop["x"])
            y = int(crop["y"])
            w = int(crop["width"])
            h = int(crop["height"])
            return gray_frame[y:y + h, x:x + w]
        except Exception:
            return gray_frame

    def update_imported_video_crop_labels(self):
        """
        Update the visible labels attached to the crop sliders.

        These labels replace the older x/y/width/height input fields.
        """
        try:
            raw_w = int(getattr(self, "imported_video_raw_width", 0) or 0)
            raw_h = int(getattr(self, "imported_video_raw_height", 0) or 0)

            x = int(self.videoCropOffsetX_slider.value())
            y = int(self.videoCropOffsetY_slider.value())

            if raw_w > 0:
                self.videoCropFrameWidth_slider.setMaximum(max(1, raw_w - x))
            if raw_h > 0:
                self.videoCropFrameHeight_slider.setMaximum(max(1, raw_h - y))

            self.videoCropFrameWidth_value.setText(str(int(self.videoCropFrameWidth_slider.value())))
            self.videoCropFrameHeight_value.setText(str(int(self.videoCropFrameHeight_slider.value())))
            self.videoCropOffsetX_value.setText(str(x))
            self.videoCropOffsetY_value.setText(str(y))

        except Exception:
            pass

    def configure_imported_video_crop_controls(self, width, height):
        """
        Initialize crop controls after a video has been opened.
        """
        try:
            width = int(width or 0)
            height = int(height or 0)
        except Exception:
            width = 0
            height = 0

        self.imported_video_raw_width = width
        self.imported_video_raw_height = height

        max_x = max(0, width - 1)
        max_y = max(0, height - 1)

        self.videoCropOffsetX_slider.setMaximum(max_x)
        self.videoCropOffsetY_slider.setMaximum(max_y)
        self.videoCropFrameWidth_slider.setMaximum(max(1, width))
        self.videoCropFrameHeight_slider.setMaximum(max(1, height))

        self.videoCropOffsetX_slider.setValue(0)
        self.videoCropOffsetY_slider.setValue(0)
        self.videoCropFrameWidth_slider.setValue(max(1, width))
        self.videoCropFrameHeight_slider.setValue(max(1, height))
        self.update_imported_video_crop_labels()

    def apply_imported_video_crop(self):
        """
        Apply the crop to the current displayed frame.

        Eye ROIs and tail arcs must be positioned on the cropped image.
        """
        if self.imported_video_path is None:
            return

        try:
            self.load_imported_video_first_frame(self.imported_video_path, keep_crop_values=True)
            self.importedVideoStatus_label.setText(
                "Crop applied: offset x={} offset y={} frame width={} frame height={}".format(
                    self.videoCropOffsetX_slider.value(),
                    self.videoCropOffsetY_slider.value(),
                    self.videoCropFrameWidth_slider.value(),
                    self.videoCropFrameHeight_slider.value()
                )
            )
        except Exception as exc:
            print("apply_imported_video_crop error:", exc)

    def reset_imported_video_crop(self):
        """
        Reset the crop to the full video frame.
        """
        if self.imported_video_raw_width > 0 and self.imported_video_raw_height > 0:
            self.videoCropOffsetX_slider.setValue(0)
            self.videoCropOffsetY_slider.setValue(0)
            self.videoCropFrameWidth_slider.setValue(self.imported_video_raw_width)
            self.videoCropFrameHeight_slider.setValue(self.imported_video_raw_height)
            self.update_imported_video_crop_labels()

        if self.imported_video_path is not None:
            self.load_imported_video_first_frame(self.imported_video_path, keep_crop_values=True)

    def set_imported_review_limits(self, total_frames):
        """
        Configure the imported-video review slider and spinbox.
        """
        try:
            total_frames = int(total_frames or 0)
        except Exception:
            total_frames = 0

        self.imported_video_total_frames = total_frames
        max_frame = max(0, total_frames - 1)

        self.imported_review_ignore_signals = True
        try:
            self.reviewFrameSlider.setRange(0, max_frame)
            self.reviewFrameSpin.setRange(0, max_frame)
            self.reviewFrameSlider.setValue(0)
            self.reviewFrameSpin.setValue(0)

            enabled = total_frames > 0
            self.reviewFrameSlider.setEnabled(enabled)
            self.reviewFrameSpin.setEnabled(enabled)
            self.reviewFramePrev_btn.setEnabled(enabled)
            self.reviewFrameNext_btn.setEnabled(enabled)
        finally:
            self.imported_review_ignore_signals = False

    def on_review_frame_slider_changed(self, value):
        """Handle imported-video review slider changes."""
        if getattr(self, "imported_review_ignore_signals", False):
            return

        self.show_imported_video_review_frame(int(value), update_controls=False)

    def on_review_frame_spin_changed(self, value):
        """Handle imported-video review spinbox changes."""
        if getattr(self, "imported_review_ignore_signals", False):
            return

        self.show_imported_video_review_frame(int(value), update_controls=False)

    def review_imported_video_previous_frame(self):
        """Move the imported-video review view to the previous frame."""
        try:
            frame_id = max(0, int(self.reviewFrameSpin.value()) - 1)
            self.show_imported_video_review_frame(frame_id)
        except Exception:
            pass

    def review_imported_video_next_frame(self):
        """Move the imported-video review view to the next frame."""
        try:
            max_frame = max(0, int(self.reviewFrameSlider.maximum()))
            frame_id = min(max_frame, int(self.reviewFrameSpin.value()) + 1)
            self.show_imported_video_review_frame(frame_id)
        except Exception:
            pass

    def show_imported_video_review_frame(self, frame_id, update_controls=True):
        """
        Display an imported-video frame and restore its tracking overlays when
        the frame has already been analyzed.

        This enables frame-by-frame review after analysis.
        """
        if self.imported_video_path is None:
            return

        try:
            frame_id = int(frame_id)
        except Exception:
            frame_id = 0

        max_frame = max(0, int(getattr(self, "imported_video_total_frames", 0) or 0) - 1)
        frame_id = max(0, min(max_frame, frame_id))

        try:
            cap = cv2.VideoCapture(self.imported_video_path)

            if not cap.isOpened():
                return

            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
            ok, frame = cap.read()
            cap.release()

            if not ok or frame is None:
                return

            if len(frame.shape) > 2:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            cropped_gray = self.crop_imported_video_frame(gray)

            self.imported_video_frame = cropped_gray.copy()
            self.show_gray_frame_in_image_dock(cropped_gray)

            if update_controls:
                self.imported_review_ignore_signals = True
                try:
                    self.reviewFrameSlider.setValue(frame_id)
                    self.reviewFrameSpin.setValue(frame_id)
                finally:
                    self.imported_review_ignore_signals = False
            else:
                self.imported_review_ignore_signals = True
                try:
                    self.reviewFrameSlider.setValue(frame_id)
                    self.reviewFrameSpin.setValue(frame_id)
                finally:
                    self.imported_review_ignore_signals = False

            result = None

            try:
                if self.controller is not None:
                    result = self.controller.get_video_file_result(frame_id)
            except Exception:
                result = None

            if result is not None:
                try:
                    self.controller.apply_result_overlay(result, move_eye_roi=False)
                    self.importedVideoStatus_label.setText(
                        "Review frame {} / {} — tracking overlay displayed".format(
                            frame_id,
                            max_frame
                        )
                    )
                except Exception as exc:
                    print("review overlay error:", exc)
            else:
                self.importedVideoStatus_label.setText(
                    "Review frame {} / {} — no tracking result yet".format(
                        frame_id,
                        max_frame
                    )
                )

        except Exception as exc:
            print("show_imported_video_review_frame error:", exc)

    def maybe_auto_preview_imported_video_frame(self, frame_id):
        """
        Periodically preview the current imported-video frame during analysis.

        The preview is intentionally throttled to avoid slowing the analysis.
        """
        if not self.previewDuringAnalysis_ckb.isChecked():
            return

        try:
            frame_id = int(frame_id)
        except Exception:
            return

        if frame_id < 0:
            return

        if self.imported_review_last_auto_frame >= 0:
            if frame_id - self.imported_review_last_auto_frame < 25:
                return

        self.imported_review_last_auto_frame = frame_id
        self.show_imported_video_review_frame(frame_id)

    def start_imported_video_analysis(self):
        """Validate settings, then start analysis of the imported video."""
        if getattr(self, "analysis_mode", "live") != "video":
            self.set_analysis_mode("video", update_combo=True)

            if getattr(self, "analysis_mode", "live") != "video":
                return

        if self.controller is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Controller",
                "AppController is not initialized."
            )
            return

        if self.imported_video_path is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Video file",
                "Choose a video file before starting analysis."
            )
            return

        ok = self.check_analysis_parameters()

        if not ok:
            return

        if not self.validate_result_settings():
            return

        self.reset_Plot()
        self.reset_Buffer()
        self.clear_tail_arc_tracking_markers()

        try:
            self.controller.start_video_file_analysis(
                self.imported_video_path,
                video_crop=self.get_imported_video_crop_config()
            )
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Video analysis",
                str(exc)
            )
            return

        self.importedVideoProgress.setValue(0)
        self.imported_review_last_auto_frame = -1
        self.importedVideoStatus_label.setText("Imported video analysis running...")
        self.startImportedVideo_btn.setEnabled(False)
        self.stopImportedVideo_btn.setEnabled(True)
        self.track_checkBox.setEnabled(False)
        self.video_file_status_timer.start()
        self.show_video_file_interface()

    def stop_imported_video_analysis(self):
        """Request the imported-video analysis worker to stop."""
        try:
            if self.controller is not None:
                self.controller.stop_video_file_analysis()
        except Exception:
            pass

        self.importedVideoStatus_label.setText("Stopping imported video analysis...")

    def update_imported_video_status(self):
        """Poll imported-video analysis status and update progress, controls, and preview."""
        if self.controller is None:
            return

        status = self.controller.get_video_file_analysis_status()

        progress = int(round(float(status.get("progress", 0.0))))
        progress = max(0, min(100, progress))
        self.importedVideoProgress.setValue(progress)

        frame_id = int(status.get("frame_id", 0) or 0)
        total_frames = int(status.get("total_frames", 0) or 0)
        rows_written = int(status.get("rows_written", 0) or 0)

        if status.get("running", False):
            self.importedVideoStatus_label.setText(
                "Running: frame {} / {} | CSV rows {}".format(
                    frame_id,
                    total_frames,
                    rows_written
                )
            )

            preview_frame_id = int(status.get("preview_frame_id", frame_id - 1) or 0)
            self.maybe_auto_preview_imported_video_frame(preview_frame_id)
            return

        if status.get("error", ""):
            self.video_file_status_timer.stop()
            self.startImportedVideo_btn.setEnabled(True)
            self.stopImportedVideo_btn.setEnabled(False)
            self.track_checkBox.setEnabled(True)
            self.importedVideoStatus_label.setText(
                "Error: {}".format(status.get("error", ""))
            )
            return

        if status.get("done", False):
            self.video_file_status_timer.stop()
            self.importedVideoProgress.setValue(100)
            self.startImportedVideo_btn.setEnabled(True)
            self.stopImportedVideo_btn.setEnabled(False)
            self.track_checkBox.setEnabled(True)
            self.importedVideoStatus_label.setText(
                "Done: {} frames analyzed | CSV rows {}".format(
                    frame_id,
                    rows_written
                )
            )
            self.updatePlot_Full()
            self.update_next_csv_preview()

            # Automatically display the last analyzed frame with its markers.
            if frame_id > 0:
                self.show_imported_video_review_frame(frame_id - 1)

            return

        if status.get("stopped", False):
            self.video_file_status_timer.stop()
            self.startImportedVideo_btn.setEnabled(True)
            self.stopImportedVideo_btn.setEnabled(False)
            self.track_checkBox.setEnabled(True)
            self.importedVideoStatus_label.setText(
                "Stopped: {} frames analyzed | CSV rows {}".format(
                    frame_id,
                    rows_written
                )
            )
            self.updatePlot_Full()
            self.update_next_csv_preview()

            if frame_id > 0:
                self.show_imported_video_review_frame(frame_id - 1)

            return

