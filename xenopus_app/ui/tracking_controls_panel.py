# -*- coding: utf-8 -*-
"""
Tracking controls, thresholds, preview overlays, and result setup.

@author: Courtand, Kadri 
"""

import math
import os
import time

import cv2
import pyqtgraph as pg
from PyQt5 import QtWidgets

from xenopus_app.rois import define_rois
from xenopus_app.rois.tail_arc_roi import tail_Track_arc_fast
from xenopus_app.tracking.legacy_tracking import eye_segmentation, eye_Rotation
from xenopus_app.stimulation.ftdi_trigger import trigger

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


class TrackingControlsMixin(object):
    """Mixin that manages tracking controls, thresholds, buffers, and previews."""
    def choose_result_folder(self):
        """Open a folder dialog and store the CSV output directory."""
        folder = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Choose output folder",
            os.path.expanduser("~")
        )

        if folder:
            self.result_save_dir = folder
            self.outputFolder_label.setText(folder)
            self.update_next_csv_preview()

    def get_result_stage(self):
        """Return the normalized Xenopus stage string used in CSV filenames."""
        stage = self.stageLineEdit.text().strip()

        if stage.lower().startswith("st"):
            stage = stage[2:]

        return stage

    def get_result_file_number(self):
        """Return the requested CSV file number, or None for automatic numbering."""
        number = self.fileNumber_spinBox.value()

        if number == 0:
            return None

        return number

    def update_next_csv_preview(self):
        """Update the label showing the next CSV filename."""
        try:
            if self.result_save_dir is None:
                self.nextCsv_label.setText("Next CSV: -")
                return

            stage = self.get_result_stage()

            if stage == "":
                self.nextCsv_label.setText("Next CSV: missing stage")
                return

            if self.controller is None:
                self.nextCsv_label.setText("Next CSV: -")
                return

            filename = self.controller.preview_next_result_filename()
            self.nextCsv_label.setText("Next CSV: " + filename)

        except Exception:
            self.nextCsv_label.setText("Next CSV: -")

    def validate_result_settings(self):
        """Validate output folder, stage, and overwrite state before starting tracking."""
        if self.result_save_dir is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Output folder",
                "Choose an output folder before starting tracking."
            )
            return False

        if self.get_result_stage() == "":
            QtWidgets.QMessageBox.warning(
                self,
                "Stage",
                "Define the Xenopus stage before starting tracking."
            )
            return False

        try:
            if self.controller is not None:
                result_path = self.controller.preview_next_result_file_path()

                if os.path.exists(result_path):
                    reply = QtWidgets.QMessageBox.warning(
                        self,
                        "Existing CSV file",
                        "This file already exists and will be overwritten:\n\n"
                        + result_path
                        + "\n\nContinue ?",
                        QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                        QtWidgets.QMessageBox.No
                    )

                    if reply != QtWidgets.QMessageBox.Yes:
                        return False

        except Exception as exc:
            print("CSV overwrite check error:", exc)

        return True

    def clear_old_acquisition_buffers(self):
        """
        Clear frames left in the previous acquisition thread.

        This prevents the new pipeline from consuming stale frames before the
        fresh acquisition run starts.
        """
        try:
            acquisition_thread = getattr(self.video_capture_widget, "acquisition_thread", None)

            if acquisition_thread is None:
                return

            frame_buffer = getattr(acquisition_thread, "frame_buffer", None)

            if frame_buffer is not None:
                analysis_lock = getattr(frame_buffer, "analysis_lock", None)
                analysis_frames = getattr(frame_buffer, "analysis_frames", None)

                if analysis_lock is not None and analysis_frames is not None:
                    with analysis_lock:
                        analysis_frames.clear()

                display_lock = getattr(frame_buffer, "display_lock", None)
                display_frames = getattr(frame_buffer, "display_frames", None)

                if display_lock is not None and display_frames is not None:
                    with display_lock:
                        display_frames.clear()

            display_queue = getattr(acquisition_thread, "display_queue", None)

            if display_queue is not None:
                while True:
                    try:
                        display_queue.get_nowait()
                    except Exception:
                        break

            # The AcquisitionWorker must not keep pointing to the previous thread.
            self.video_capture_widget.acquisition_thread = None

            print("Old acquisition buffers cleared")

        except Exception as exc:
            print("Erreur clear_old_acquisition_buffers:", exc)

    def toggle_tracking(self):
        """Start or stop real-time tracking from the Track checkbox."""
        if self.controller is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Controller",
                "AppController is not initialized in the main."
            )
            self.track_checkBox.setChecked(False)
            return

        if self.track_checkBox.isChecked():
            if getattr(self, "analysis_mode", "live") != "live":
                QtWidgets.QMessageBox.warning(
                    self,
                    "Analysis mode",
                    "Switch to Real-time camera mode before starting live tracking."
                )
                self.track_checkBox.setChecked(False)
                return

            ok = self.check_analysis_parameters()

            if not ok:
                self.track_checkBox.setChecked(False)
                return

            if not self.validate_result_settings():
                self.track_checkBox.setChecked(False)
                return

            self.reset_Plot()
            self.reset_Buffer()

            try:
                if hasattr(self.video_capture_widget, "stop_acquisition"):
                    self.video_capture_widget.stop_acquisition()
                    # Short safety delay to let the previous thread close.
                    # Avoid the older 0.1 s delay to reduce first-frame loss.
                    time.sleep(0.02)

                # Drop frames left by the previous acquisition.
                # Otherwise, the CSV can start with frame IDs from the previous run
                # before restarting at 0 when the new acquisition begins.
                self.clear_old_acquisition_buffers()

            except Exception as exc:
                print("Erreur stop acquisition:", exc)
                self.track_checkBox.setChecked(False)
                return

            try:
                # Start tracking and CSV writing before acquisition.
                # This lets the pipeline consume the first arriving frame immediately.
                self.controller.start_tracking()
            except Exception as exc:
                print("Erreur start pipeline:", exc)
                self.track_checkBox.setChecked(False)
                return

            try:
                if hasattr(self.video_capture_widget, "start_acquisition"):
                    self.video_capture_widget.start_acquisition()
            except Exception as exc:
                print("Erreur start acquisition:", exc)
                try:
                    self.controller.stop_tracking()
                except Exception:
                    pass
                self.track_checkBox.setChecked(False)
                return

            try:
                if self.video_capture_widget.trigger_checkBox.isChecked():
                    trigger()
            except Exception:
                pass

            self.track_checkBox.setText("Stop")
            print("Tracking started with realtime pipeline")

        else:
            try:
                self.controller.stop_tracking()
            except Exception as exc:
                print("Erreur stop pipeline:", exc)

            try:
                if hasattr(self.video_capture_widget, "stop_acquisition"):
                    self.video_capture_widget.stop_acquisition()
                self.clear_old_acquisition_buffers()
            except Exception as exc:
                print("Erreur stop acquisition:", exc)

            self.track_checkBox.setText("Track")
            self.updatePlot_Full()
            self.update_next_csv_preview()
            print("Tracking stopped")

    def check_analysis_parameters(self):
        """Validate analysis-mode selection before a run starts."""
        ok=True

        if self.manipType_comboBox.currentIndex()==0:
            message="You have to select an analysis mode"
            QtWidgets.QMessageBox.warning(ui,"analysis mode",str(message),QtWidgets.QMessageBox.Ok)
            self.track_checkBox.setChecked(False)
            ok=False

        return ok

    def threshEyeValue_change(self,value,idx):
        """Preview eye detection when an eye-threshold slider changes."""
        # A slider can change before its matching ROI exists.
        # In that case, keep the previous UI behavior and do nothing.
        if idx < len(self.roisEye):
            if self.track_checkBox.isChecked()==False :
                print(idx)
                self.update_eyes_overlay(value,idx)

    def _ensure_eye_ellipse_roi(self, roiIndex):
        """Guarantee that roisEllipseEye follows roisEye after module splitting."""
        try:
            from xenopus_app.rois import define_rois
            while len(self.roisEllipseEye) <= roiIndex and len(self.roisEllipseEye) < len(self.roisEye):
                eye_roi = self.roisEye[len(self.roisEllipseEye)]
                ellipse = define_rois.EllipseROI_Centered_NoHandle(
                    pos=eye_roi.pos(),
                    size=[1, 1],
                    pen=(3, 5),
                )
                self.roisEllipseEye.append(ellipse)
                self.videoDisplay_Widget.plotView.addItem(ellipse)
        except Exception as exc:
            print("_ensure_eye_ellipse_roi error:", exc)

    def update_eyes_overlay(self,value,roiIndex):
        """Update eye ellipse and axis overlays for a selected eye ROI."""

        frame=self.get_current_analysis_frame()

        if not self.track_checkBox.isChecked() and len(self.mark.data)>0:
            if roiIndex < len(self.roisEye):

                self._ensure_eye_ellipse_roi(roiIndex)
                if roiIndex >= len(self.roisEllipseEye):
                    print("eye ellipse ROI missing for eye", roiIndex)
                    return

                roiEye=self.roisEye[roiIndex]

                xroi,yroi=roiEye.pos()
                wroi,hroi=roiEye.size()

                threshEye=[self.threshEye1_slider.value(),self.threshEye2_slider.value()]

                cnt=eye_segmentation(roiIndex,roiEye,threshEye,frame)

                if cnt :
                    ellipse_pos,anglep=eye_Rotation(roiIndex,roiEye,cnt)

                    if roiIndex==0 :
                        linePen=pg.mkPen(color='c', width=2)
                    elif roiIndex==1 :
                        linePen=pg.mkPen(color=(255,128,0), width=2)

                    self.eyeAxeLines[roiIndex].setPos((ellipse_pos[0],ellipse_pos[1]))
                    self.eyeAxeLines[roiIndex].setAngle(-anglep+90)
                    self.eyeAxeLines[roiIndex].setPen(linePen)
                    self.videoDisplay_Widget.plotView.addItem(self.eyeAxeLines[roiIndex], ignoreBounds=True)

                    x,y,MA,ma,angle,vx,vy,xrot,yrot=self.roisEllipseEye[roiIndex].descriptor
                    self.roisEllipseEye[roiIndex].setPos((x,y),update=False)
                    self.roisEllipseEye[roiIndex].setSize((MA, ma),update=False)
                    self.roisEllipseEye[roiIndex].setAngle(angle,update=False)
                    self.roisEllipseEye[roiIndex].translate(-vx,-vy,update=False)
                    self.roisEllipseEye[roiIndex].stateChanged()

                    self.eyeAxeLines[roiIndex].setPos((x-vx+xrot,y-vy+yrot))
                    self.eyeAxeLines[roiIndex].setAngle(angle+90)

                    if yroi>varM.bodyAxis_Y:
                        angleCorr=-anglep+90-varM.bodyAngle
                    else:
                        angleCorr=anglep+90+varM.bodyAngle

                    print("init angle eye",roiIndex," : ",angleCorr)

                else :
                    print("eye detection failed, change threshold value")

        else :
            msg="no reference for body axe. You can add one with 'tail-root'"
            QtWidgets.QMessageBox.warning(ui,"warning",str(msg),QtWidgets.QMessageBox.Ok)

    def update_tail_arc_curve(self, value):
        """Backward-compatible wrapper that updates the R tail-arc curve."""
        # Backward compatibility: the old method name acts on R.
        self.update_tail_arc_curve_R(value)

    def _update_tail_arc_curve_for_label(self, label, value):
        """Update the curvature of one enabled tail arc and refresh preview if needed."""
        if not self.is_tail_arc_enabled(label):
            return

        roi = self.get_tail_arc_rois().get(label)

        if roi is not None and roi.initialized:
            roi.set_curve_value(value)

        if not self.track_checkBox.isChecked() and self.selectTailRoot_radioButton.isChecked()==True:
            self.update_tail_segment_thresh(self.threshTail_slider.value())

    def update_tail_arc_curve_R(self, value):
        """Update the R tail-arc curve."""
        self._update_tail_arc_curve_for_label("R", value)

    def update_tail_arc_curve_M(self, value):
        """Update the M tail-arc curve."""
        self._update_tail_arc_curve_for_label("M", value)

    def update_tail_arc_curve_C(self, value):
        """Update the C tail-arc curve."""
        self._update_tail_arc_curve_for_label("C", value)

    def update_tail_segment_thresh(self,thresh_value):
        """Preview tail detection for all enabled tail arcs at the current threshold."""
        if not self.track_checkBox.isChecked() and self.selectTailRoot_radioButton.isChecked()==True:

            if len(self.mark.data) == 0:
                return

            frame = self.get_current_analysis_frame()

            if frame is None:
                return

            if not self._initialize_tail_arcs_if_needed():
                return

            thresholds = self.get_tail_arc_thresholds()
            last_valid_tail = None

            # Preview all three arcs: each arc displays its own point
            # and its own root-to-point line.
            for label in ["R", "M", "C"]:
                roi = self.get_tail_arc_rois().get(label)

                if not self.is_tail_arc_enabled(label):
                    self.set_tail_arc_tracking_marker(label, None)
                    continue

                if roi is None or not roi.initialized:
                    self.set_tail_arc_tracking_marker(label, None)
                    continue

                iframe, tail_pos, tail_angle = tail_Track_arc_fast(
                    0,
                    frame,
                    thresholds.get(label, self.threshTail_slider.value()),
                    roi.get_parameters(),
                    append_to_lists=False
                )

                if tail_pos[0] != 0 or tail_pos[1] != 0:
                    self.set_tail_arc_tracking_marker(label, tail_pos)
                    last_valid_tail = tail_pos
                else:
                    self.set_tail_arc_tracking_marker(label, None)

            # Keep the historical tail point for compatibility, but
            # display the three R/M/C points separately.
            if last_valid_tail is not None:
                self.mark.data['pos'][2] = [last_valid_tail[0], last_valid_tail[1]]
                self.mark.updateGraph()

    def update_tail_segment_overlay(self):
        """Update body-axis state and tail-arc overlays from reference markers."""
        if self.track_checkBox.isChecked()==False :

            if len(self.mark.data)>0:
                tailRoot=self.mark.data['pos'][0]
                nose=self.mark.data['pos'][1]
                tail=self.mark.data['pos'][2]

                noseX=nose[0]
                noseY=nose[1]
                rootX=tailRoot[0]
                rootY=tailRoot[1]

                varM.bodyAxis_Y=noseY

                xv= noseX-rootX
                yv= noseY-rootY

                varM.bodyAngle = math.atan2(yv, xv)* 180 / math.pi
                print("angle de l'axe du corps : ",varM.bodyAngle)

                self._initialize_tail_arcs_if_needed()

                for roi in self.get_tail_arc_rois().values():
                    if roi.initialized:
                        roi.update_graph()

                self.update_tail_arc_enabled_states(update_preview=False)

                frame = self.get_current_analysis_frame()

                if frame is not None:
                    self.update_tail_segment_thresh(self.threshTail_slider.value())

            else :
                msg="no reference for body axe. You can add one with 'tail-root'"
                QtWidgets.QMessageBox.warning(ui,"warning",str(msg),QtWidgets.QMessageBox.Ok)

    def init_track(self):
        """Initialize the legacy OpenCV multi-tracker for limb ROIs."""

        bboxes=[]
        if self.video.nbFrames!=None:
            currentFrame=self.video.capture.get(1)-1
            retVal=self.video.capture.set(1,currentFrame)

            (ret, frame) = self.video.capture.read()

            target.tracker =cv2.MultiTracker("MIL")
            if len(roisLimb)>0:

                for roi in roisLimb :
                    print(roi.size())
                    print(roi.pos())
                    x,y=roi.pos()
                    w,h=roi.size()

                    ycv2=self.video.height-y-h
                    bbox=(x, ycv2, w, h)
                    bboxes.append(bbox)

            ok = target.tracker.add(frame,(bboxes))

            print("tracker : ",ok)

    def activate_interface(self,module):
        """Enable controls associated with a loaded video or live camera."""

        if module=="load video":
            # The old video player was removed from the visible UI.
            # Keep this block only to avoid errors from old signal connections.
            if self.videoPlayer_Widget is not None:
                self.videoPlayer_Widget.playVideo_btn.setEnabled(True)
                self.videoPlayer_Widget.playVideo_btn.setChecked(False)
                self.videoPlayer_Widget.timeLine_slider.setEnabled(True)
                self.videoPlayer_Widget.stepFwdVideo_btn.setEnabled(True)
                self.videoPlayer_Widget.stepBwdVideo_btn.setEnabled(True)
            return

        elif module=="live video":
            self.video_capture_widget.liveVideo_btn.setEnabled(True)

    def reset_Buffer(self):
        """Clear the legacy frame buffer and update its label."""
        framesBuffer[:]=[]
        self.video_capture_widget.lenBuffer_label.setText(str(len(framesBuffer)))

