# -*- coding: utf-8 -*-
"""
Plot update and application-close helpers for the main UI.

@author: Courtand, Kadri 
"""

import numpy as np
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


class PlotPanelMixin(object):
    """Mixin that updates plots and handles safe application shutdown."""
    def reset_Plot(self):
        """Clear all plot data and reset tracking result lists."""

        for eyeEllipse in self.roisEllipseEye :
            eyeEllipse.angleList[:]=[]

            eyeEllipse.yList[:]=[]

        self.tailAngleList[:]=[]
        self.tailPosList[:]=[]

        for list_name in [
            "tailAngleListR", "tailAngleListM", "tailAngleListC",
            "tailPosListR", "tailPosListM", "tailPosListC",
        ]:
            if hasattr(self, list_name):
                getattr(self, list_name)[:]=[]

        stimList[:]=[]

        try:

            dataArray = np.asarray([])
            self.w3.plot(dataArray,clear=True)
            self.w4.plot(dataArray,clear=True)
            self.w5.plot(dataArray,clear=True)
            self.w6.plot(dataArray,clear=True)
            self.w7.plot(dataArray,clear=True)
            print("plot reset ok ")
        except ValueError as e :
                QtWidgets.QMessageBox.warning(ui,"Error",str(ValueError),QtWidgets.QMessageBox.Ok)

    def update_plot(self) :
        """Update rolling live plots from the latest tracking lists."""
        currentIdx=self.analysis_thread.last_processed_id
        if self.track_checkBox.isChecked()==True:

            dataArray1 = np.asarray(self.roisEllipseEye[0].angleList[currentIdx-200: currentIdx])

            self.w3.plot(dataArray1,pen=self.penCyan,clear=True)

            dataArray2 = np.asarray(self.roisEllipseEye[1].angleList[currentIdx-200: currentIdx])

            self.w4.plot(dataArray2,pen=self.penOrange,clear=True)

            first_tail_plot = True
            for values, pen in [
                (getattr(self, "tailAngleListR", []), self.penTailR),
                (getattr(self, "tailAngleListM", []), self.penTailM),
                (getattr(self, "tailAngleListC", []), self.penTailC),
            ]:
                dataArray3 = np.asarray(values[currentIdx-200: currentIdx])

                if dataArray3.size > 0:
                    self.w5.plot(dataArray3,pen=pen,clear=first_tail_plot)
                    first_tail_plot = False

            if first_tail_plot and len(self.tailAngleList)!=0:
                dataArray3 = np.asarray(self.tailAngleList[currentIdx-200: currentIdx])

                self.w5.plot(dataArray3,pen=self.penGreen,clear=True)

    def updatePlot_Full(self) :
        """Render full plots after tracking or imported-video analysis stops."""

        if self.track_checkBox.isChecked()==False:
            if len(self.roisEllipseEye)>0 :

                dataArray1 = np.asarray(self.roisEllipseEye[0].angleList)

                self.w3.plot(dataArray1,pen=self.penCyan,clear=True)

                dataArray2 = np.asarray(self.roisEllipseEye[1].angleList)

                self.w4.plot(dataArray2,pen=self.penOrange,clear=True)

                dataArray4 = np.asarray(self.roisEllipseEye[0].yList)

                self.w6.plot(dataArray4,pen=self.penCyan,clear=True)

                dataArray5 = np.asarray(self.roisEllipseEye[1].yList)

                self.w7.plot(dataArray5,pen=self.penOrange,clear=True)

                first_tail_plot = True
                for values, pen in [
                    (getattr(self, "tailAngleListR", []), self.penTailR),
                    (getattr(self, "tailAngleListM", []), self.penTailM),
                    (getattr(self, "tailAngleListC", []), self.penTailC),
                ]:
                    dataArray3 = np.asarray(values)

                    if dataArray3.size > 0:
                        self.w5.plot(dataArray3,pen=pen,clear=first_tail_plot)
                        first_tail_plot = False

                if first_tail_plot and len(self.tailAngleList)!=0:
                    dataArray3 = np.asarray(self.tailAngleList)

                    self.w5.plot(dataArray3,pen=self.penGreen,clear=True)

    def close_camera(self):
        """Stop acquisition and close the camera safely if it is open."""
        try:
            if getattr(self, "controller", None) is not None:
                try:
                    self.controller.stop_tracking()
                except Exception:
                    pass
                try:
                    self.controller.stop_video_file_analysis()
                except Exception:
                    pass
        except Exception:
            pass

        try:
            capture_widget = getattr(self, "video_capture_widget", None)
            acquisition_thread = getattr(capture_widget, "acquisition_thread", None)

            if acquisition_thread is not None:
                try:
                    acquisition_thread.stop()
                except Exception:
                    pass

                try:
                    if acquisition_thread.is_alive():
                        acquisition_thread.join(timeout=2.0)
                except Exception:
                    pass

                try:
                    capture_widget.acquisition_thread = None
                except Exception:
                    pass
        except Exception as exc:
            print("Camera acquisition cleanup error:", exc)

        try:
            device = getattr(getattr(self, "video", None), "device", None)

            if device is not None:
                try:
                    if device.IsGrabbing():
                        device.StopGrabbing()
                except Exception:
                    pass

                try:
                    if device.IsOpen():
                        device.Close()
                except Exception:
                    try:
                        device.Close()
                    except Exception:
                        pass
        except Exception as exc:
            print("Camera close error:", exc)

    def closeEvent(self, event):
        """Ask for confirmation, then stop workers and close the camera safely."""
        reply = QtWidgets.QMessageBox.question(
            self,
            'User confirm',
            'Have you saved track and optokinetic results ?',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )

        if reply != QtWidgets.QMessageBox.Yes:
            event.ignore()
            return

        self.close_camera()
        event.accept()
