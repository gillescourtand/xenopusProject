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
import numpy as np
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



_TAIL_ARC_MASK_CACHE = {}
_TAIL_ARC_PIXEL_CACHE = {}


def _angle_delta_signed(start_angle, end_angle):
    return (end_angle - start_angle + math.pi) % (2.0 * math.pi) - math.pi


def _tail_arc_crop_mask_cached(image_shape, arc_roi):
    """
    Masque arc rapide avec cache.
    Le masque est recalculé seulement si la ROI change.
    """
    img_height, img_width = image_shape[:2]

    cx, cy = arc_roi["center"]
    cx = float(cx)
    cy = float(cy)

    inner_radius = max(1.0, float(arc_roi["inner_radius"]))
    outer_radius = max(inner_radius + 1.0, float(arc_roi["outer_radius"]))
    start_angle = float(arc_roi["start_angle"])
    end_angle = float(arc_roi["end_angle"])

    version = int(arc_roi.get("version", 0))

    key = (
        img_height,
        img_width,
        int(round(cx)),
        int(round(cy)),
        int(round(inner_radius)),
        int(round(outer_radius)),
        int(round(start_angle * 1000)),
        int(round(end_angle * 1000)),
        version,
    )

    cached = _TAIL_ARC_MASK_CACHE.get(key)
    if cached is not None:
        return cached

    # Cache limité : on garde quelques masques pour permettre plusieurs arcs R/M/C
    # sans recalculer tous les masques à chaque frame.
    if len(_TAIL_ARC_MASK_CACHE) > 12:
        _TAIL_ARC_MASK_CACHE.clear()

    margin = 3

    x0 = max(0, int(cx - outer_radius - margin))
    x1 = min(img_width, int(cx + outer_radius + margin + 1))

    y_plot_min = cy - outer_radius - margin
    y_plot_max = cy + outer_radius + margin

    y0 = max(0, int(img_height - y_plot_max))
    y1 = min(img_height, int(img_height - y_plot_min + 1))

    if x1 <= x0 or y1 <= y0:
        result = (None, None)
        _TAIL_ARC_MASK_CACHE[key] = result
        return result

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

    result = ((x0, y0, x1, y1), mask)
    _TAIL_ARC_MASK_CACHE[key] = result
    return result


def _tail_arc_mask_pixels_cached(arc_mask):
    """
    Retourne les coordonnées des pixels appartenant à un arc.

    Ces coordonnées sont mises en cache : tant que l'arc ne bouge pas,
    on évite de recalculer np.nonzero() à chaque frame.
    """
    if arc_mask is None:
        return None, None

    key = id(arc_mask)
    cached = _TAIL_ARC_PIXEL_CACHE.get(key)

    if cached is not None:
        return cached

    if len(_TAIL_ARC_PIXEL_CACHE) > 12:
        _TAIL_ARC_PIXEL_CACHE.clear()

    ys, xs = np.nonzero(arc_mask)

    if ys.size == 0:
        result = (None, None)
    else:
        result = (ys.astype(np.intp, copy=False), xs.astype(np.intp, copy=False))

    _TAIL_ARC_PIXEL_CACHE[key] = result
    return result


def _tail_track_arc_fast(image, tail_threshold, arc_roi, root_position, body_angle):
    """
    Tracking rapide d'un arc de queue.

    Version précédente :
    - seuillage image complète de l'arc ;
    - dilatation ;
    - findContours ;
    - tri des contours ;
    - moments du plus grand contour.

    Nouvelle version :
    - le masque de l'arc est pré-calculé et mis en cache ;
    - on lit uniquement les pixels de l'arc ;
    - le point détecté est le centre pondéré des pixels sous le seuil.

    Avantage : beaucoup plus rapide pour R/M/C, sans passer en mode latest-frame.
    Donc on reste en FIFO et on ne saute pas volontairement de frames.
    """
    if arc_roi is None:
        return None

    img_height = image.shape[0]

    crop_box, arc_mask = _tail_arc_crop_mask_cached(image.shape, arc_roi)

    if crop_box is None or arc_mask is None:
        return None

    x0, y0, x1, y1 = crop_box
    crop = image[y0:y1, x0:x1]

    if crop.size == 0:
        return None

    ys, xs = _tail_arc_mask_pixels_cached(arc_mask)

    if ys is None or xs is None or ys.size == 0:
        return None

    # Valeurs des pixels uniquement dans l'arc.
    values = crop[ys, xs].astype(np.int16, copy=False)

    # Pour un fond clair et une queue sombre : pixels < seuil.
    weights = int(tail_threshold) - values
    valid = weights > 0

    if not np.any(valid):
        return None

    xs_valid = xs[valid].astype(np.float32, copy=False)
    ys_valid = ys[valid].astype(np.float32, copy=False)
    weights_valid = weights[valid].astype(np.float32, copy=False)

    weight_sum = float(np.sum(weights_valid))

    if weight_sum <= 0.0:
        return None

    tail_x_local = int(np.sum(xs_valid * weights_valid) / weight_sum)
    tail_y_local = int(np.sum(ys_valid * weights_valid) / weight_sum)

    tail_x = tail_x_local + x0
    tail_y = int(img_height) - (tail_y_local + y0)

    root_x, root_y = root_position

    tail_angle = math.atan2((root_y - tail_y), (root_x - tail_x)) * 180.0 / math.pi
    tail_angle *= -1.0
    tail_angle_corr = tail_angle + body_angle

    return {
        "angle": tail_angle_corr,
        "x": tail_x,
        "y": tail_y,
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
                       kernel_size=9,
                       tail_arc_roi=None,
                       tail_arc_rois=None,
                       tail_thresholds=None):
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

        # ------------------------------------------------------------------
        # Tracking queue :
        # - nouveau mode : 3 arcs R / M / C ;
        # - ancien mode conservé : un seul arc ou une région rectangulaire.
        # ------------------------------------------------------------------
        if tail_arc_rois is not None:
            labels = ["R", "M", "C"]
            first_tail = None

            for label in labels:
                arc_roi = tail_arc_rois.get(label) if isinstance(tail_arc_rois, dict) else None

                if arc_roi is None:
                    continue

                if isinstance(tail_thresholds, dict):
                    threshold_value = tail_thresholds.get(label, tail_threshold)
                else:
                    threshold_value = tail_threshold

                tail = _tail_track_arc_fast(
                    gray,
                    threshold_value,
                    arc_roi,
                    root_position,
                    body_angle,
                )

                if tail is None:
                    continue

                setattr(result, "tail_{}_angle".format(label), tail["angle"])
                setattr(result, "tail_{}_x".format(label), tail["x"])
                setattr(result, "tail_{}_y".format(label), tail["y"])

                result.metadata["tail_{}_position".format(label)] = [tail["x"], tail["y"]]

                if first_tail is None:
                    first_tail = tail

            # Compatibilité avec les anciennes colonnes/listes :
            # on garde la mesure R comme mesure principale si elle existe.
            # Sinon on prend la première mesure disponible.
            main_tail = None
            if getattr(result, "tail_R_angle", None) is not None:
                main_tail = {
                    "angle": result.tail_R_angle,
                    "x": result.tail_R_x,
                    "y": result.tail_R_y,
                }
            elif first_tail is not None:
                main_tail = first_tail

            if main_tail is not None:
                result.tail_angle = main_tail["angle"]
                result.tail_x = main_tail["x"]
                result.tail_y = main_tail["y"]

        else:
            if tail_arc_roi is not None:
                tail = _tail_track_arc_fast(
                    gray,
                    tail_threshold,
                    tail_arc_roi,
                    root_position,
                    body_angle,
                )
            else:
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
                result.tail_R_angle = tail["angle"]
                result.tail_R_x = tail["x"]
                result.tail_R_y = tail["y"]

    except Exception as exc:
        result.valid = False
        result.error = str(exc)

    return result
