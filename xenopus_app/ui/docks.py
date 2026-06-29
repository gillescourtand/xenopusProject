# -*- coding: utf-8 -*-
"""
Dock and analysis-mode helpers for the Xenopus main window.

@author: Courtand, Kadri 
"""

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


class DockModeMixin(object):
    """Mixin that manages dock layout and active analysis mode."""
    def get_current_analysis_frame(self):
        """
        Return the frame used by previews.

        Imported-video mode uses the loaded video frame. Live mode uses the
        latest frame displayed by the camera widget.
        """
        try:
            if getattr(self, "imported_video_frame", None) is not None:
                if getattr(self, "analysis_mode", "live") == "video":
                    return self.imported_video_frame
        except Exception:
            pass

        try:
            updater = getattr(self.video_capture_widget, "videoDisplayer_updater", None)

            if updater is not None:
                frame = getattr(updater, "current_frame_to_display", None)

                if frame is not None:
                    return frame
        except Exception:
            pass

        return getattr(self, "imported_video_frame", None)

    def show_gray_frame_in_image_dock(self, gray_frame):
        """
        Display a grayscale frame directly in the existing image plot.

        This bypasses show_frame_in_pyqtgraph() so imported-video display does
        not depend on the live-camera LUT path.
        """
        try:
            self.video.currentGrayFrame = gray_frame.copy()
            self.video.currentFrame = gray_frame.copy()
            self.video.currentTFrame = cv2.transpose(gray_frame[::-1, :])
            self.videoDisplay_Widget.img.setImage(
                self.video.currentTFrame,
                autoLevels=False
            )
        except Exception as exc:
            print("show_gray_frame_in_image_dock error:", exc)

    def _remember_live_dock_heights(self):
        """
        Cache the current live dock heights so the layout can be restored cleanly.
        """
        if getattr(self, "_live_dock_height_cache", None):
            return

        for name, dock in [("d1", self.d1), ("d11", self.d11)]:
            try:
                self._live_dock_height_cache[name] = {
                    "min": dock.minimumHeight(),
                    "max": dock.maximumHeight(),
                    "height": dock.height(),
                }
            except Exception:
                pass

    def _force_dock_height(self, dock, height):
        """
        Temporarily force a dock height.
        """
        try:
            dock.setMinimumHeight(int(height))
            dock.setMaximumHeight(int(height))
        except Exception:
            pass

    def _release_dock_height(self, dock):
        """
        Release a previously forced dock height.
        """
        try:
            dock.setMinimumHeight(0)
            dock.setMaximumHeight(16777215)
        except Exception:
            pass

    def _apply_imported_video_layout(self):
        """
        In Imported-video mode, fill the unused space by enlarging the image dock.
        """
        self._remember_live_dock_heights()

        # In Imported-video mode, the camera dock does not need to keep vertical space.
        try:
            self._force_dock_height(self.d11, 0)
            self._force_dock_height(self.d15, 232)
        except Exception:
            pass

        # Enlarge the image dock while keeping a small band for progress controls.
        try:
            self._force_dock_height(self.d1, self._video_mode_image_height)
        except Exception:
            pass

        try:
            self.area.resizeDocks(
                [self.d1, self.d15, self.d11],
                [self._video_mode_image_height, 205, 1],
                "vertical"
            )
        except Exception:
            pass

        try:
            self.videoDisplay_Widget.plotView.setAspectLocked(True)
            self.videoDisplay_Widget.plotView.autoRange()
        except Exception:
            pass

    def _restore_live_layout(self):
        """
        Restore the standard Real-time camera layout.
        """
        try:
            self._release_dock_height(self.d1)
            self._release_dock_height(self.d11)
            self._release_dock_height(self.d15)
        except Exception:
            pass

        try:
            self.area.resizeDocks(
                [self.d1, self.d11, self.d15],
                [360, 210, 1],
                "vertical"
            )
        except Exception:
            pass

    def on_analysis_mode_changed(self, index):
        """
        Handle changes from the top-level analysis-mode tabs.

        0 selects real-time camera mode. 1 selects imported-video mode.
        """
        if index == 1:
            self.set_analysis_mode("video", update_combo=False)
        else:
            self.set_analysis_mode("live", update_combo=False)

    def _is_live_tracking_running(self):
        """Return True when live tracking or its pipeline is currently active."""
        try:
            if self.track_checkBox.isChecked():
                return True
        except Exception:
            pass

        try:
            if self.controller is not None:
                if self.controller.pipeline is not None and self.controller.pipeline.running:
                    return True
        except Exception:
            pass

        return False

    def _is_video_file_analysis_running(self):
        """Return True when imported-video analysis is currently active."""
        try:
            if self.controller is None:
                return False

            status = self.controller.get_video_file_analysis_status()
            return bool(status.get("running", False))

        except Exception:
            return False

    def _set_dock_clean_visible(self, dock, visible):
        """
        Show or hide a pyqtgraph Dock while minimizing empty space.

        hide() alone can leave a large gap or title bar depending on the dock
        layout, so visibility is combined with height limits.
        """
        if dock is None:
            return

        try:
            if visible:
                dock.setMaximumHeight(16777215)
                dock.setMinimumHeight(0)
                dock.show()
            else:
                dock.hide()
                dock.setMinimumHeight(0)
                dock.setMaximumHeight(0)
        except Exception:
            pass

        try:
            if hasattr(dock, "label"):
                dock.label.setVisible(visible)
        except Exception:
            pass

    def set_analysis_mode(self, mode, update_combo=True):
        """
        Enable exactly one analysis mode at a time.

        live selects the camera workflow. video selects the imported-video
        workflow without camera capture. Controls from the inactive mode are
        hidden to avoid empty UI space and state conflicts.
        """
        if mode not in ("live", "video"):
            mode = "live"

        # Prevent mode switching while an analysis is running.
        if mode == "video" and self._is_live_tracking_running():
            QtWidgets.QMessageBox.warning(
                self,
                "Analysis mode",
                "Stop real-time tracking before switching to imported video mode."
            )
            self._sync_analysis_mode_combo("live")
            return

        if mode == "live" and self._is_video_file_analysis_running():
            QtWidgets.QMessageBox.warning(
                self,
                "Analysis mode",
                "Stop imported video analysis before switching to real-time mode."
            )
            self._sync_analysis_mode_combo("video")
            return

        self.analysis_mode = mode
        self._sync_analysis_mode_combo(mode)

        if mode == "video":
            # Imported-video mode:
            # - show imported-video controls;
            # - hide and collapse the camera dock;
            # - keep Image, ROI, segmentation, optokinetic, and plot docks available.
            try:
                self.importedVideo_toolbar.hide()
            except Exception:
                pass

            self._set_dock_clean_visible(self.d11, False)
            self._set_dock_clean_visible(self.d15, True)

            try:
                self.d1.raiseDock()
            except Exception:
                pass

            try:
                self.d8.raiseDock()
            except Exception:
                pass

            try:
                self.track_checkBox.setEnabled(False)
            except Exception:
                pass

            # Fill the space left by the hidden camera dock.
            self._apply_imported_video_layout()

            try:
                self.startImportedVideo_btn.setEnabled(True)
            except Exception:
                pass

            try:
                if self.imported_video_path is None:
                    self.importedVideoStatus_label.setText(
                        "Open a video, place ROIs/arcs, then analyze."
                    )
            except Exception:
                pass

        else:
            # Real-time mode:
            # - hide imported-video controls;
            # - show the camera dock;
            # - keep imported-video controls inaccessible.
            try:
                self.importedVideo_toolbar.hide()
            except Exception:
                pass

            self._restore_live_layout()

            self._set_dock_clean_visible(self.d15, False)
            self._set_dock_clean_visible(self.d11, True)

            try:
                self.d11.raiseDock()
            except Exception:
                pass

            try:
                self.d8.raiseDock()
            except Exception:
                pass

            try:
                self.track_checkBox.setEnabled(True)
            except Exception:
                pass

    def _sync_analysis_mode_combo(self, mode):
        """Synchronize the analysis-mode tab widget without emitting extra callbacks."""
        try:
            self.analysisMode_tabs.blockSignals(True)
            self.analysisMode_tabs.setCurrentIndex(1 if mode == "video" else 0)
            self.analysisMode_tabs.blockSignals(False)
        except Exception:
            pass

    def show_video_file_interface(self):
        """
        Backward-compatible entry point for switching to Imported-video mode.
        """
        self.set_analysis_mode("video", update_combo=True)

    def show_direct_interface(self):
        """
        Backward-compatible entry point for switching to Real-time camera mode.
        """
        self.set_analysis_mode("live", update_combo=True)

