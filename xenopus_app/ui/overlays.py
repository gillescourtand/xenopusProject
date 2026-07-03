# -*- coding: utf-8 -*-
"""
ROI, body-axis, eye-overlay, and tail-arc overlay helpers.

@author: Courtand, Kadri 
"""

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QRectF

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
    """Inject shared legacy context values into this module."""
    globals().update(ctx)


class RoiOverlayMixin(object):
    """Mixin that manages ROI creation, labels, body reference markers, and overlays."""
    def update_eye_roi_label_positions(self):
        """Keep eye ROI labels anchored inside their matching ROI."""
        for i, roi in enumerate(self.roisEye):
            if i >= len(self.roisEyeLabels):
                continue

            try:
                label = self.roisEyeLabels[i]
                label.setText("Eye{}".format(i + 1))

                w, h = roi.size()
                label.setAnchor((1, 0))
                label.setPos(float(w) - 4.0, 4.0)

            except Exception:
                pass

    def refresh_eye_roi_labels(self):
        """Refresh eye ROI label names and positions after ROI changes."""
        for i, label in enumerate(self.roisEyeLabels):
            try:
                label.setText("Eye{}".format(i + 1))
            except Exception:
                pass

        self.update_eye_roi_label_positions()

    def add_eye_roi_label(self, roi):
        """Attach a numbered label to an eye ROI."""
        idx = len(self.roisEyeLabels) + 1

        label = pg.TextItem(
            "Eye{}".format(idx),
            color=(120, 170, 255, 120),
            anchor=(1, 0)
        )
        label.setZValue(1200)
        label.setParentItem(roi)

        self.roisEyeLabels.append(label)

        try:
            roi.sigRegionChanged.connect(self.update_eye_roi_label_positions)
        except Exception:
            pass

        self.update_eye_roi_label_positions()
        return label

    def mouse_clicked(self,evt):
        """Create or update ROIs and reference markers from image-view mouse clicks."""
        pos = evt[0].pos()

        if self.videoDisplay_Widget.plotView.sceneBoundingRect().contains(pos):

            mousePointFromScene=self.videoDisplay_Widget.gviewBox.mapToView(pos)

            x0,y0=mousePointFromScene.x(),mousePointFromScene.y()

            if evt[0].button() == 1 and self.selectTailRoot_radioButton.isChecked():

                self.init_markNoseRootTail(x0,y0)
                self.update_tail_segment_overlay()

            if evt[0].button() == 1 and self.selectEyes_radioButton.isChecked():

                if len(self.roisEye) >= 2:
                    print("Maximum 2 eye ROIs.")
                    return

                w=100
                h=80

                roi_limits=QRectF(0,0,self.video.width,self.video.height)

                self.roisEye.append(Roi([x0-w/2, y0-h/2], [w, h],maxBounds=roi_limits,centered=True, pen=("b"),removable=True))
                self.roisEye[-1].sigRemoveRequested.connect(self.remove_ROI)
                self.videoDisplay_Widget.plotView.addItem(self.roisEye[-1])
                self.add_eye_roi_label(self.roisEye[-1])

                self.roisEllipseEye.append(define_rois.EllipseROI_Centered_NoHandle(pos=self.roisEye[-1].pos(),size=[1,1],pen=(3,5)))
                self.videoDisplay_Widget.plotView.addItem(self.roisEllipseEye[-1])

            if evt[0].button() == 1 and self.selectLimbs_radioButton.isChecked():

                w=100
                h=80

                roisLimb.append(Roi([x0-w/2, y0-h/2], [w, h],centered=True, pen=("m"),removable=True))
                roisLimb[-1].sigRemoveRequested.connect(self.remove_ROI)
                self.videoDisplay_Widget.plotView.addItem(roisLimb[-1])

        else:

            return

    def remove_ROI(self,evt):
        """Remove an eye ROI and its associated label and ellipse overlay."""

        print("remove:",evt)

        index=self.roisEye.index(evt)

        if index < len(self.roisEyeLabels):
            try:
                self.videoDisplay_Widget.plotView.scene().removeItem(self.roisEyeLabels[index])
            except Exception:
                pass
            del self.roisEyeLabels[index]

        self.videoDisplay_Widget.plotView.scene().removeItem(evt)

        self.roisEye.remove(evt)

        self.videoDisplay_Widget.plotView.scene().removeItem(self.roisEllipseEye[index])
        del self.roisEllipseEye[index]

        self.refresh_eye_roi_labels()

    def init_markNoseRootTail(self,x0,y0):
        """Initialize or move the root, nose, and tail reference markers."""

        if len(self.mark.data) != 0 :

            tailRoot=self.mark.data['pos'][0]
            nose=self.mark.data['pos'][1]
            tailpos=self.mark.data['pos'][2]

            relativePos_nose=tailRoot-nose
            relativePos_tail=tailRoot-tailpos
            x1=x0-relativePos_nose[0]
            y1=y0-relativePos_nose[1]
            x2=x0-relativePos_tail[0]
            y2=y0-relativePos_tail[1]

            pos = np.array([
                [x0,y0],
                [x1,y1],
                [x2,y2]
                ], dtype=float)
        else :
            pos = np.array([
                [x0,y0],
                [x0+80,y0],
                [x0-80,y0]
                ], dtype=float)

        adj = np.array([
            [0,1],
            [0,2]
            ])

        symbols = ['o','o','o']

        symbolBrushes=[(255,0,255),
                       (255,0,255),
                       (0,255,0)
                       ]

        lines = np.array([
            (255,0,255,255,2),
            (0,255,0,255,2)
            ], dtype=[('red',np.ubyte),('green',np.ubyte),('blue',np.ubyte),('alpha',np.ubyte),('width',float)])

        texts=["root","nose","tail"]

        self.mark.size=8
        self.mark.setData(pos=pos, adj=adj, pen=lines, size=self.mark.size, symbolBrush=symbolBrushes,symbolPen='w',symbol=symbols, pxMode=False, text=texts)

    def is_tail_arc_enabled(self, label):
        """
        Indique si l'arc R/M/C est actif.
        Return whether a tail arc is enabled.

        A disabled arc is hidden and excluded from tracking.
        """
        label = str(label).upper()
        checkbox = getattr(self, "tailArc{}_checkBox".format(label), None)

        if checkbox is None:
            return True

        return bool(checkbox.isChecked())

    def update_tail_arc_enabled_states(self, *args, update_preview=True):
        """
        Synchronize the R/M/C checkboxes with slider state, arc visibility, and
        tracking-marker visibility.
        """
        try:
            for label in ["R", "M", "C"]:
                enabled = self.is_tail_arc_enabled(label)

                for widget in getattr(self, "tailArcControlWidgets", {}).get(label, []):
                    try:
                        widget.setEnabled(enabled)
                    except Exception:
                        pass

                roi = self.get_tail_arc_rois().get(label)
                if roi is not None:
                    try:
                        roi.set_visible(enabled and roi.initialized)
                    except Exception:
                        pass

                if not enabled:
                    self.set_tail_arc_tracking_marker(label, None)

            if update_preview and not self.track_checkBox.isChecked():
                if self.selectTailRoot_radioButton.isChecked() and len(self.mark.data) > 0:
                    self.update_tail_segment_thresh(self.threshTail_slider.value())

        except Exception as exc:
            print("update_tail_arc_enabled_states error:", exc)

    def get_tail_arc_roi_params(self):
        """Return the legacy single-arc tracking parameters for the R arc."""
        # Backward compatibility: return only R when the R arc is enabled.
        if self.is_tail_arc_enabled("R") and hasattr(self, "tailArcROI_R") and self.tailArcROI_R.initialized:
            return self.tailArcROI_R.get_parameters()
        return None

    def get_tail_arc_roi_params_all(self):
        """Return tracking parameters for all initialized and enabled tail arcs."""
        params = {}

        # Return a dictionary even when it is empty.
        # If no arc is enabled, tracking must not fall back to
        # the old rectangular tail-tracking mode.
        for label, roi in self.get_tail_arc_rois().items():
            if not self.is_tail_arc_enabled(label):
                continue

            if roi.initialized:
                params[label] = roi.get_parameters()

        return params

    def get_tail_arc_rois(self):
        """Return the available tail-arc ROI objects indexed by label."""
        if hasattr(self, "tailArcROIs"):
            return self.tailArcROIs

        if hasattr(self, "tailArcROI"):
            return {"R": self.tailArcROI}

        return {}

    def get_tail_arc_thresholds(self):
        """Return the current tail thresholds for R, M, and C arcs."""
        return {
            "R": int(self.threshTail_slider.value()),
            "M": int(self.threshTailM_slider.value()) if hasattr(self, "threshTailM_slider") else int(self.threshTail_slider.value()),
            "C": int(self.threshTailC_slider.value()) if hasattr(self, "threshTailC_slider") else int(self.threshTail_slider.value()),
        }

    def get_tail_arc_curve_slider(self, label):
        """Return the curve slider associated with one tail-arc label."""
        label = str(label).upper()

        if label == "M" and hasattr(self, "tailArcCurveM_slider"):
            return self.tailArcCurveM_slider

        if label == "C" and hasattr(self, "tailArcCurveC_slider"):
            return self.tailArcCurveC_slider

        if hasattr(self, "tailArcCurveR_slider"):
            return self.tailArcCurveR_slider

        return self.tailArcCurve_slider

    def set_tail_arc_tracking_marker(self, label, tail_pos):
        """
        Display the tracked tail point and root-to-point line for one arc.

        Each enabled arc can display its own R, M, or C line.
        """
        try:
            label = str(label).upper()
            line_item = self.tailArcTrackingLines.get(label)
            point_item = self.tailArcTrackingPoints.get(label)

            if line_item is None or point_item is None:
                return

            if tail_pos is not None and not self.is_tail_arc_enabled(label):
                tail_pos = None

            if tail_pos is None or len(tail_pos) < 2:
                line_item.setData([], [])
                point_item.setData([], [])
                return

            x_pos = float(tail_pos[0])
            y_pos = float(tail_pos[1])

            if x_pos == 0 and y_pos == 0:
                line_item.setData([], [])
                point_item.setData([], [])
                return

            root = self.mark.data['pos'][0]
            root_x = float(root[0])
            root_y = float(root[1])

            line_item.setData([root_x, x_pos], [root_y, y_pos])
            point_item.setData([x_pos], [y_pos])

        except Exception as exc:
            print("set_tail_arc_tracking_marker error:", exc)

    def clear_tail_arc_tracking_markers(self):
        """Clear all visible tail-arc tracking markers."""
        for label in ["R", "M", "C"]:
            self.set_tail_arc_tracking_marker(label, None)

    def _selected_tail_arc_labels(self):
        """Return enabled tail-arc labels."""
        return [label for label in ["R", "M", "C"] if self.is_tail_arc_enabled(label)]

    def _initialize_tail_arcs_if_needed(self):
        """Initialize tail arcs from root, nose, and tail markers when needed."""
        if len(self.mark.data) == 0:
            return False

        tailRoot = self.mark.data['pos'][0]
        nose = self.mark.data['pos'][1]
        tail = self.mark.data['pos'][2]

        for label, roi in self.get_tail_arc_rois().items():
            if not roi.initialized:
                roi.initialize_from_points(tailRoot, nose, tail)

        self.update_tail_arc_enabled_states(update_preview=False)
        return True

