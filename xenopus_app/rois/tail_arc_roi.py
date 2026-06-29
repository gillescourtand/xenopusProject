# -*- coding: utf-8 -*-
"""
Tail arc ROI widgets and fast arc-based tail tracking helpers.

@author: Courtand, Kadri 
"""

import math
import cv2
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5 import QtWidgets
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui as QtgGui

ui = None
varM = None

def set_context(**ctx):
    """Inject shared runtime objects used by this legacy-compatible module."""
    globals().update(ctx)

def _angle_delta_signed(start_angle, end_angle):
    """Return the signed shortest angular difference between two radians values."""
    return (end_angle - start_angle + math.pi) % (2.0 * math.pi) - math.pi


def _tail_arc_crop_mask(image_shape, arc_roi):
    """Build a small binary mask covering only the requested tail arc crop."""
    img_height, img_width = image_shape[:2]

    cx, cy = arc_roi["center"]
    cx = float(cx)
    cy = float(cy)

    inner_radius = max(1.0, float(arc_roi["inner_radius"]))
    outer_radius = max(inner_radius + 1.0, float(arc_roi["outer_radius"]))

    start_angle = float(arc_roi["start_angle"])
    end_angle = float(arc_roi["end_angle"])

    margin = 3

    x0 = max(0, int(cx - outer_radius - margin))
    x1 = min(img_width, int(cx + outer_radius + margin + 1))


    y_plot_min = cy - outer_radius - margin
    y_plot_max = cy + outer_radius + margin

    y0 = max(0, int(img_height - y_plot_max))
    y1 = min(img_height, int(img_height - y_plot_min + 1))

    if x1 <= x0 or y1 <= y0:
        return None, None

    yy, xx = np.indices((y1 - y0, x1 - x0), dtype=np.float32)
    xx += x0


    y_plot = img_height - (yy + y0)

    dx = xx - cx
    dy = y_plot - cy

    radius = np.sqrt(dx * dx + dy * dy)
    angle = np.arctan2(dy, dx)

    delta_total = _angle_delta_signed(start_angle, end_angle)
    delta_pixel = (angle - start_angle + math.pi) % (2.0 * math.pi) - math.pi

    if delta_total >= 0:
        angle_ok = (delta_pixel >= 0) & (delta_pixel <= delta_total)
    else:
        angle_ok = (delta_pixel <= 0) & (delta_pixel >= delta_total)

    radius_ok = (radius >= inner_radius) & (radius <= outer_radius)

    mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    mask[radius_ok & angle_ok] = 255

    return (x0, y0, x1, y1), mask


def tail_Track_arc_fast(iframe, analysimg, thresh_tail, arc_roi, append_to_lists=True):
    """Track the tail inside one arc ROI using thresholding and contour moments."""
    if analysimg is None or arc_roi is None:
        return iframe, [0, 0], 180

    if len(analysimg.shape) > 2:
        gray = cv2.cvtColor(analysimg, cv2.COLOR_BGR2GRAY)
    else:
        gray = analysimg

    im_height = gray.shape[0]

    crop_box, arc_mask = _tail_arc_crop_mask(gray.shape, arc_roi)

    if crop_box is None:
        return iframe, [0, 0], 180

    x0, y0, x1, y1 = crop_box
    crop = gray[y0:y1, x0:x1]

    threshTailZone = cv2.threshold(crop, thresh_tail, 255, cv2.THRESH_BINARY_INV)[1]
    threshTailZone = cv2.bitwise_and(threshTailZone, threshTailZone, mask=arc_mask)
    threshTailZone = cv2.dilate(threshTailZone, None, iterations=5)

    tailcnts = cv2.findContours(
        threshTailZone.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )[-2]

    if tailcnts:
        ctail = sorted(tailcnts, key=cv2.contourArea, reverse=True)[:1]
        M = cv2.moments(ctail[0])

        if M["m00"] == 0:
            return iframe, [0, 0], 180

        tailX_local = int(M["m10"] / M["m00"])
        tailY_local = int(M["m01"] / M["m00"])

        ctailX = tailX_local + x0
        ctailY = int(im_height) - (tailY_local + y0)

        rootx, rooty = ui.mark.data['pos'][0]

        tailAngle = ((math.atan2((rooty - ctailY), (rootx - ctailX))) * 180 / math.pi) * (-1)
        tailAngleCorr = tailAngle + varM.bodyAngle

    else:
        print("tail detection failed")
        tailAngleCorr = 180
        ctailX = 0
        ctailY = 0

    tail_pos = [ctailX, ctailY]

    if append_to_lists:
        ui.tailAngleList.append([iframe, tailAngleCorr])
        ui.tailPosList.append([iframe, tail_pos])

    return iframe, tail_pos, tailAngleCorr


class TailArcHandleGraph(pg.GraphItem):
    """Graph item holding draggable handles for a tail arc ROI."""
    def __init__(self, on_change=None):
        """Initialize the object state and graphical items."""
        pg.GraphItem.__init__(self)
        self.on_change = on_change
        self.dragPoint = None
        self.dragOffset = None
        self.textItems = []
        self.data = {
            "pos": np.empty((0, 2), dtype=float),
            "data": np.empty(0, dtype=[("index", int)])
        }

    def set_positions(self, positions, labels=None):
        """Set the draggable handle positions for the tail arc ROI."""

        for item in self.textItems:
            try:
                item.scene().removeItem(item)
            except Exception:
                pass

        self.textItems = []

        pos = np.asarray(positions, dtype=float)
        data = np.empty(len(pos), dtype=[("index", int)])
        data["index"] = np.arange(len(pos))
        self.data = {"pos": pos.copy(), "data": data}

        self.updateGraph()

    def updateGraph(self):
        """Refresh the graph item and reposition its text labels."""
        pg.GraphItem.setData(
            self,
            pos=self.data["pos"],
            data=self.data["data"],
            size=8,
            symbol="o",
            symbolBrush=(40, 140, 255, 230),
            symbolPen=pg.mkPen("w", width=1),
            pxMode=True
        )

        for i, item in enumerate(self.textItems):
            if i < len(self.data["pos"]):
                item.setPos(self.data["pos"][i][0], self.data["pos"][i][1])

    def mouseDragEvent(self, ev):
        """Handle mouse dragging for ROI creation, view panning, or marker movement."""
        if ev.button() != Qt.LeftButton:
            ev.ignore()
            return

        if ev.isStart():
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)

            if len(pts) == 0:
                ev.ignore()
                return

            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.dragOffset = self.data["pos"][ind] - pos

        elif ev.isFinish():
            self.dragPoint = None
            return

        else:
            if self.dragPoint is None:
                ev.ignore()
                return

        ind = self.dragPoint.data()[0]
        new_pos = ev.pos() + self.dragOffset
        new_xy = np.array([new_pos.x(), new_pos.y()], dtype=float)

        self.data["pos"][ind] = new_xy
        self.updateGraph()

        if self.on_change is not None:
            self.on_change(ind, new_xy)

        ev.accept()


class TailArcFillItem(QtWidgets.QGraphicsPathItem):
    """Clickable fill item used to move the whole tail arc ROI."""
    def __init__(self, roi):
        """Initialize the object state and graphical items."""
        QtWidgets.QGraphicsPathItem.__init__(self)
        self.roi = roi
        self._last_pos = None
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setAcceptHoverEvents(True)


    def mousePressEvent(self, ev):
        """Start moving the arc fill item unless a handle is under the cursor."""
        if ev.button() == Qt.LeftButton:


            try:
                p = ev.pos()
                handle_positions = getattr(self.roi.handles, "data", {}).get("pos", [])
                for hp in handle_positions:
                    dx = float(hp[0]) - float(p.x())
                    dy = float(hp[1]) - float(p.y())
                    if (dx * dx + dy * dy) <= (14.0 * 14.0):
                        ev.ignore()
                        return
            except Exception:
                pass

            self._last_pos = ev.pos()

            ev.accept()
        else:
            QtWidgets.QGraphicsPathItem.mousePressEvent(self, ev)

    def mouseMoveEvent(self, ev):
        """Move the whole arc ROI while the fill item is being dragged."""
        if self._last_pos is None:
            QtWidgets.QGraphicsPathItem.mouseMoveEvent(self, ev)
            return

        pos = ev.pos()
        delta = pos - self._last_pos
        self._last_pos = pos

        self.roi.move_by(delta.x(), delta.y())
        ev.accept()

    def mouseReleaseEvent(self, ev):
        """Stop the current fill-item drag operation."""
        self._last_pos = None

        ev.accept()


class TailArcROI(object):
    """Interactive annular-sector ROI used to constrain tail tracking to one arc band."""
    def __init__(self, plot_view, label="R", color=(255, 0, 0), band_index=0):
        """Initialize the object state and graphical items."""
        self.plot_view = plot_view
        self.label = str(label)
        self.color = tuple(color)
        self.band_index = int(band_index)

        self.center = np.array([50.0, 150.0], dtype=float)
        self.inner_radius = 35.0
        self.outer_radius = 95.0
        self.start_angle = -0.75
        self.end_angle = 0.75
        self.initialized = False

        self._dirty_version = 0
        self._curve_reference = None
        self._last_curve_value = 50.0

        self.fill_item = TailArcFillItem(self)
        self.fill_item.setPen(pg.mkPen(color=self.color, width=1))
        self.fill_item.setBrush(QtgGui.QBrush(QtgGui.QColor(
            int(self.color[0]), int(self.color[1]), int(self.color[2]), 45
        )))

        pen = pg.mkPen(color=self.color, width=2)
        side_pen = pg.mkPen(color=self.color, width=1)

        self.inner_curve = pg.PlotDataItem(x=[], y=[], pen=pen)
        self.outer_curve = pg.PlotDataItem(x=[], y=[], pen=pen)
        self.side_curve_1 = pg.PlotDataItem(x=[], y=[], pen=side_pen)
        self.side_curve_2 = pg.PlotDataItem(x=[], y=[], pen=side_pen)
        self.handles = TailArcHandleGraph(on_change=self._handle_moved)

        self.label_item = pg.TextItem(self.label, color=self.color, anchor=(0.0, 1.0))

        self.fill_item.setZValue(800)
        self.plot_view.addItem(self.fill_item)

        for item in [
            self.inner_curve,
            self.outer_curve,
            self.side_curve_1,
            self.side_curve_2,
        ]:
            item.setZValue(900)
            self.plot_view.addItem(item)

        self.handles.setZValue(2000)
        self.plot_view.addItem(self.handles)

        self.label_item.setZValue(2100)
        self.plot_view.addItem(self.label_item)

        self.set_visible(False)

    def set_visible(self, visible):
        """Show or hide all graphics items that make up the tail arc ROI."""
        for item in [
            self.fill_item,
            self.inner_curve,
            self.outer_curve,
            self.side_curve_1,
            self.side_curve_2,
            self.handles,
            self.label_item,
        ]:
            item.setVisible(visible)

    def move_by(self, dx, dy):
        """Translate the entire tail arc ROI by a display-space offset."""
        self.center = self.center + np.array([float(dx), float(dy)], dtype=float)

        if self._curve_reference is not None:
            self._curve_reference["mid_point"] = self._curve_reference["mid_point"] + np.array([float(dx), float(dy)], dtype=float)

        self._dirty_version += 1
        self.update_graph()

    def initialize_from_points(self, root, nose, tail):
        """Initialize the arc geometry from root, nose, and tail reference points."""
        root = np.asarray(root, dtype=float)
        nose = np.asarray(nose, dtype=float)
        tail = np.asarray(tail, dtype=float)

        direction = tail - root
        length = np.linalg.norm(direction)

        if length < 1:
            direction = root - nose
            length = np.linalg.norm(direction)

        if length < 1:
            return

        angle_mid = math.atan2(direction[1], direction[0])












        try:
            img_h = float(ui.video.height)
        except Exception:
            img_h = max(300.0, length * 4.0)

        self.center = root.copy()



        base_length = max(float(length), img_h * 0.45)
        fractions = [0.28, 0.52, 0.76]
        index = max(0, min(2, int(self.band_index)))

        root_gap = max(25.0, base_length * fractions[index])
        arc_width = max(20.0, base_length * 0.11)

        self.inner_radius = root_gap
        self.outer_radius = root_gap + arc_width


        self.start_angle = angle_mid - math.radians(60.0)
        self.end_angle = angle_mid + math.radians(60.0)

        self.initialized = True
        self._dirty_version += 1

        self._save_curve_reference()
        self._last_curve_value = 50.0
        self.set_visible(True)
        self.update_graph()

        try:
            slider = ui.get_tail_arc_curve_slider(self.label)
            slider.blockSignals(True)
            slider.setValue(40)
            slider.blockSignals(False)
            self.set_curve_value(40)
        except Exception:
            pass

    def _save_curve_reference(self):
        """Store the current arc geometry used by the curve slider."""
        mid_angle = self._angle_mid()
        mid_radius = (self.inner_radius + self.outer_radius) / 2.0
        width = max(2.0, self.outer_radius - self.inner_radius)
        delta = abs(_angle_delta_signed(self.start_angle, self.end_angle))

        if delta < math.radians(3.0):
            delta = math.radians(3.0)

        direction = np.array([math.cos(mid_angle), math.sin(mid_angle)], dtype=float)
        mid_point = self.center + mid_radius * direction

        self._curve_reference = {
            "mid_point": mid_point.copy(),
            "mid_angle": float(mid_angle),
            "mid_radius": float(mid_radius),
            "width": float(width),
            "arc_length": float(mid_radius * delta),
            "sign": 1.0 if _angle_delta_signed(self.start_angle, self.end_angle) >= 0 else -1.0,
        }

    def _update_curve_reference_width_only(self):
        """Update only the stored band width after a radius handle is moved."""
        if self._curve_reference is None:
            self._save_curve_reference()
            return

        width = max(2.0, self.outer_radius - self.inner_radius)
        self._curve_reference["width"] = float(width)

    def _curve_scale_from_value(self, value):
        """Convert the curve slider value to a geometric scale factor."""
        value = max(0.0, min(100.0, float(value)))

        if value <= 50.0:

            return 1.0 + ((50.0 - value) / 50.0) * 3.0


        return 1.0 - ((value - 50.0) / 50.0) * 0.5

    def set_curve_value(self, value):
        """Adjust the arc curvature while keeping the current arc position stable."""
        if not self.initialized:
            return

        value = max(0.0, min(100.0, float(value)))

        previous_value = getattr(self, "_last_curve_value", 50.0)

        previous_scale = self._curve_scale_from_value(previous_value)
        new_scale = self._curve_scale_from_value(value)

        if previous_scale <= 0:
            previous_scale = 1.0

        ratio = new_scale / previous_scale

        mid_angle = self._angle_mid()
        mid_radius = max(1.0, (self.inner_radius + self.outer_radius) / 2.0)
        width = max(2.0, self.outer_radius - self.inner_radius)

        delta = _angle_delta_signed(self.start_angle, self.end_angle)
        sign = 1.0 if delta >= 0 else -1.0
        delta_abs = max(math.radians(3.0), abs(delta))

        direction = np.array([math.cos(mid_angle), math.sin(mid_angle)], dtype=float)
        mid_point = self.center + mid_radius * direction



        arc_length = mid_radius * delta_abs

        new_mid_radius = max(width / 2.0 + 2.0, mid_radius * ratio)

        self.center = mid_point - new_mid_radius * direction

        self.inner_radius = max(2.0, new_mid_radius - width / 2.0)
        self.outer_radius = self.inner_radius + width

        new_delta = arc_length / max(new_mid_radius, 1.0)
        new_delta = max(math.radians(3.0), min(math.radians(170.0), new_delta))
        new_delta *= sign

        self.start_angle = mid_angle - new_delta / 2.0
        self.end_angle = mid_angle + new_delta / 2.0

        self._last_curve_value = value
        self._dirty_version += 1
        self.update_graph()


    def get_parameters(self):
        """Return the current arc geometry as JSON-serializable values."""
        return {
            "center": [float(self.center[0]), float(self.center[1])],
            "inner_radius": float(self.inner_radius),
            "outer_radius": float(self.outer_radius),
            "start_angle": float(self.start_angle),
            "end_angle": float(self.end_angle),
            "version": int(self._dirty_version),
        }

    def _angle_mid(self):
        """Return the middle angle of the arc."""
        delta = _angle_delta_signed(self.start_angle, self.end_angle)
        return self.start_angle + delta / 2.0

    def _arc_points(self, radius, n=50):
        """Sample points along one arc radius."""
        delta = _angle_delta_signed(self.start_angle, self.end_angle)
        angles = self.start_angle + np.linspace(0.0, delta, n)

        x = self.center[0] + radius * np.cos(angles)
        y = self.center[1] + radius * np.sin(angles)

        return x, y, angles

    def update_graph(self):
        """Recompute and redraw all graphical items of the tail arc ROI."""
        self.inner_radius = max(2.0, float(self.inner_radius))
        self.outer_radius = max(self.inner_radius + 2.0, float(self.outer_radius))

        inner_x, inner_y, _ = self._arc_points(self.inner_radius)
        outer_x, outer_y, _ = self._arc_points(self.outer_radius)

        self.inner_curve.setData(inner_x, inner_y)
        self.outer_curve.setData(outer_x, outer_y)

        self.side_curve_1.setData([inner_x[0], outer_x[0]], [inner_y[0], outer_y[0]])
        self.side_curve_2.setData([inner_x[-1], outer_x[-1]], [inner_y[-1], outer_y[-1]])

        path = QtgGui.QPainterPath()
        path.moveTo(float(outer_x[0]), float(outer_y[0]))

        for x_val, y_val in zip(outer_x[1:], outer_y[1:]):
            path.lineTo(float(x_val), float(y_val))

        for x_val, y_val in zip(inner_x[::-1], inner_y[::-1]):
            path.lineTo(float(x_val), float(y_val))

        path.closeSubpath()
        self.fill_item.setPath(path)

        angle_mid = self._angle_mid()
        inner_mid = self.center + self.inner_radius * np.array([math.cos(angle_mid), math.sin(angle_mid)])
        outer_mid = self.center + self.outer_radius * np.array([math.cos(angle_mid), math.sin(angle_mid)])
        start_handle = self.center + self.outer_radius * np.array([math.cos(self.start_angle), math.sin(self.start_angle)])
        end_handle = self.center + self.outer_radius * np.array([math.cos(self.end_angle), math.sin(self.end_angle)])

        self.handles.set_positions(
            [inner_mid, outer_mid, start_handle, end_handle],
            ["R1", "R2", "A", "B"]
        )

        try:


            all_x = np.concatenate([inner_x, outer_x])
            all_y = np.concatenate([inner_y, outer_y])
            label_x = float(np.min(all_x))
            label_y = float(np.max(all_y) + 6.0)
            self.label_item.setText(self.label)
            self.label_item.setPos(label_x, label_y)
        except Exception:
            pass

    def _handle_moved(self, index, pos):
        """Update the arc geometry after one draggable handle moves."""
        try:
            if index == 0:

                self.inner_radius = max(2.0, np.linalg.norm(pos - self.center))
                if self.inner_radius >= self.outer_radius - 2.0:
                    self.inner_radius = self.outer_radius - 2.0

                self._update_curve_reference_width_only()

            elif index == 1:

                self.outer_radius = max(self.inner_radius + 2.0, np.linalg.norm(pos - self.center))

                self._update_curve_reference_width_only()

            elif index == 2:


                mid_angle = self._angle_mid()
                moved_angle = math.atan2(pos[1] - self.center[1], pos[0] - self.center[0])

                half_width = abs(_angle_delta_signed(mid_angle, moved_angle))
                half_width = max(math.radians(3.0), min(math.radians(85.0), half_width))

                sign = 1.0 if _angle_delta_signed(self.start_angle, self.end_angle) >= 0 else -1.0

                self.start_angle = mid_angle - sign * half_width
                self.end_angle = mid_angle + sign * half_width
                self._save_curve_reference()
                self._last_curve_value = 50.0

                try:
                    slider = ui.get_tail_arc_curve_slider(self.label)
                    slider.blockSignals(True)
                    slider.setValue(50)
                    slider.blockSignals(False)
                except Exception:
                    pass

            elif index == 3:


                mid_angle = self._angle_mid()
                moved_angle = math.atan2(pos[1] - self.center[1], pos[0] - self.center[0])

                half_width = abs(_angle_delta_signed(mid_angle, moved_angle))
                half_width = max(math.radians(3.0), min(math.radians(85.0), half_width))

                sign = 1.0 if _angle_delta_signed(self.start_angle, self.end_angle) >= 0 else -1.0

                self.start_angle = mid_angle - sign * half_width
                self.end_angle = mid_angle + sign * half_width
                self._save_curve_reference()
                self._last_curve_value = 50.0

                try:
                    slider = ui.get_tail_arc_curve_slider(self.label)
                    slider.blockSignals(True)
                    slider.setValue(50)
                    slider.blockSignals(False)
                except Exception:
                    pass

            self._dirty_version += 1
            self.update_graph()

        except Exception as exc:
            print("TailArcROI drag error:", exc)
