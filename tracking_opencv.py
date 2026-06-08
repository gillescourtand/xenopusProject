# -*- coding: utf-8 -*-
"""
tracking_opencv.py

Tracking OpenCV actuel :
- seuillage ;
- morphologie ;
- contours ;
- ellipse des yeux ;
- angle de queue.

Ce module ne doit pas gérer l'acquisition, l'affichage ou le CSV.
"""

import math
import cv2
from frame_packet import TrackingResult


def _roi_to_ints(roi):
    x, y = roi.pos()
    w, h = roi.size()
    return int(x), int(y), int(w), int(h)


def _clip_roi_to_image(x, y, w, h, img):
    img_height, img_width = img.shape[:2]

    if x < 0:
        w += x
        x = 0
    if y < 0:
        h += y
        y = 0
    if x + w > img_width:
        w -= (x + w - img_width)
    if y + h > img_height:
        h -= (y + h - img_height)

    w = max(1, int(w))
    h = max(1, int(h))

    return int(x), int(y), int(w), int(h)


def _eye_track_one(image, roi, threshold, body_axis_y, body_angle, kernel_size=9):
    img_height = image.shape[0]

    x, y, w, h = _roi_to_ints(roi)
    x, y, w, h = _clip_roi_to_image(x, y, w, h, image)

    # Conversion coordonnées PyQtGraph -> OpenCV
    x_cv = int(x)
    y_cv = int(img_height - y - h)

    y_cv = max(0, min(y_cv, img_height - 1))
    x_cv = max(0, min(x_cv, image.shape[1] - 1))

    crop = image[y_cv:y_cv + h, x_cv:x_cv + w]

    if crop.size == 0:
        return None

    blur = cv2.GaussianBlur(crop, (5, 5), 0)
    th = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY_INV)[1]

    if kernel_size is None or kernel_size < 1:
        kernel_size = 9
    if kernel_size % 2 == 0:
        kernel_size += 1

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

    contours = cv2.findContours(th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(contours) == 0:
        return None

    contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
    hull = cv2.convexHull(contour)

    if len(hull) < 5:
        return None

    ellipse = cv2.fitEllipse(hull)
    (xe, ye), (ma, MA), angle = ellipse

    anglep = angle - 90.0

    xp = int(x + xe - MA / 2.0)
    yp = int(y + h - ye - ma / 2.0)

    xc = MA / 2.0
    yc = ma / 2.0
    radangle = anglep / 180.0 * math.pi

    xrot = xc * math.cos(radangle) + yc * math.sin(radangle)
    yrot = xc * (-math.sin(radangle)) + yc * math.cos(radangle)

    vx = xrot - xc
    vy = yrot - yc

    ellipse_pos = [xp - vx + xrot, yp - vy + yrot]

    if y > body_axis_y:
        angle_corr = -anglep + 90.0 - body_angle
    else:
        angle_corr = anglep + 90.0 + body_angle

    descriptor = [xp, yp, MA, ma, -anglep, vx, vy, xrot, yrot]

    return {
        "angle": angle_corr,
        "y": ellipse_pos[1],
        "position": ellipse_pos,
        "descriptor": descriptor,
    }


def _tail_track(image, tail_threshold, tail_region, root_position, body_angle):
    img_height = image.shape[0]

    if tail_region is None:
        return None

    x0 = int(tail_region[0])
    x1 = int(tail_region[1])

    if x1 < x0:
        x0, x1 = x1, x0

    x0 = max(0, min(x0, image.shape[1] - 1))
    x1 = max(x0 + 1, min(x1, image.shape[1]))

    crop = image[:, x0:x1]

    if crop.size == 0:
        return None

    th = cv2.threshold(crop, tail_threshold, 255, cv2.THRESH_BINARY_INV)[1]
    th = cv2.dilate(th, None, iterations=5)

    contours = cv2.findContours(th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(contours) == 0:
        return None

    contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
    moments = cv2.moments(contour)

    if moments["m00"] == 0:
        return None

    tail_x_local = int(moments["m10"] / moments["m00"])
    tail_y_local = int(moments["m01"] / moments["m00"])

    tail_x = tail_x_local + x0
    tail_y = int(img_height) - tail_y_local

    root_x, root_y = root_position

    tail_angle = math.atan2((root_y - tail_y), (root_x - tail_x)) * 180.0 / math.pi
    tail_angle *= -1.0
    tail_angle_corr = tail_angle + body_angle

    return {
        "angle": tail_angle_corr,
        "x": tail_x,
        "y": tail_y,
    }


def track_frame_opencv(packet,
                       rois_eye,
                       eye_thresholds,
                       tail_threshold,
                       tail_region,
                       root_position,
                       body_axis_y,
                       body_angle,
                       kernel_size=9):
    result = TrackingResult(
        frame_id=packet.frame_id,
        timestamp=packet.timestamp,
        okr_state=packet.okr_state,
    )

    image = packet.image

    try:
        if image is None:
            result.valid = False
            result.error = "No image"
            return result

        if len(image.shape) > 2:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        if rois_eye is not None and len(rois_eye) >= 1:
            eye1 = _eye_track_one(
                gray,
                rois_eye[0],
                eye_thresholds[0],
                body_axis_y,
                body_angle,
                kernel_size,
            )
            if eye1 is not None:
                result.eye1_angle = eye1["angle"]
                result.eye1_y = eye1["y"]
                result.metadata["eye1_descriptor"] = eye1["descriptor"]

        if rois_eye is not None and len(rois_eye) >= 2:
            eye2 = _eye_track_one(
                gray,
                rois_eye[1],
                eye_thresholds[1],
                body_axis_y,
                body_angle,
                kernel_size,
            )
            if eye2 is not None:
                result.eye2_angle = eye2["angle"]
                result.eye2_y = eye2["y"]
                result.metadata["eye2_descriptor"] = eye2["descriptor"]

        tail = _tail_track(
            gray,
            tail_threshold,
            tail_region,
            root_position,
            body_angle,
        )

        if tail is not None:
            result.tail_angle = tail["angle"]
            result.tail_x = tail["x"]
            result.tail_y = tail["y"]

    except Exception as exc:
        result.valid = False
        result.error = str(exc)

    return result
