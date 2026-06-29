# -*- coding: utf-8 -*-
"""Legacy eye and rectangular-tail tracking helpers.

These functions keep compatibility with the former monolithic
``MotionAnalysis_Xenopus`` workflow. They are mainly used by interactive UI
previews and older tracking paths.

The real-time pipeline should use ``opencv_tracker.track_frame_opencv`` instead.

@author: Courtand, Kadri 
"""

import math

import cv2

from xenopus_app.rois.roi_items import testRoiInImageview


ui = None
image = None
varM = None


def set_context(**ctx):
    """Inject legacy shared objects used by the old tracking functions."""
    globals().update(ctx)


def eye_segmentation(roiIndex, roiEye, threshEye, analysimg):
    """Segment one eye inside its ROI and return the largest contour.

    Parameters
    ----------
    roiIndex : int
        Eye index used to read the matching threshold value.
    roiEye : pyqtgraph ROI
        Eye ROI in PyQtGraph coordinates.
    threshEye : list[int]
        Threshold values for both eyes.
    analysimg : numpy.ndarray
        Current frame. RGB/BGR frames are converted to grayscale.
    """
    if analysimg is None:
        return None

    if len(analysimg.shape) > 2:
        analysimg = cv2.cvtColor(analysimg, cv2.COLOR_BGR2GRAY)

    img_height, img_width = analysimg.shape[:2]

    xroi, yroi = roiEye.pos()
    wroi, hroi = roiEye.size()

    roiXc, roiYc, wroic, hroic = testRoiInImageview(
        xroi,
        yroi,
        wroi,
        hroi,
        analysimg,
    )

    image.crop = [roiXc, roiYc, wroic, hroic]

    xroicv = int(roiXc)
    yroicv = int(img_height - roiYc - hroic)
    wroicv = int(wroic)
    hroicv = int(hroic)

    cropImgData = analysimg[yroicv:yroicv + hroicv, xroicv:xroicv + wroicv]
    blur = cv2.GaussianBlur(cropImgData, (5, 5), 0)
    threshed_Eye = cv2.threshold(
        blur,
        threshEye[roiIndex],
        255,
        cv2.THRESH_BINARY_INV,
    )[1]

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    threshed_Eye = cv2.morphologyEx(threshed_Eye, cv2.MORPH_OPEN, kernel)

    cnts = cv2.findContours(
        threshed_Eye.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    if len(cnts) > 0:
        return sorted(cnts, key=cv2.contourArea, reverse=True)[:1]

    print("no eye contour")
    return None


def _fallback_eye_position(roiEye, roiIndex):
    """Return the last eye position, or the ROI center if none exists."""
    try:
        print(ui.roisEllipseEye[roiIndex].yList[-1])
        return ui.roisEllipseEye[roiIndex].yList[-1]
    except Exception:
        try:
            xroi, yroi = roiEye.pos()
            wroi, hroi = roiEye.size()
            return [
                float(xroi) + float(wroi) / 2.0,
                float(yroi) + float(hroi) / 2.0,
            ]
        except Exception:
            return [0, 0]


def eye_Rotation(roiIndex, roiEye, cnt):
    """Fit an ellipse on an eye contour and return its center and angle."""
    for contour in cnt:
        hull = cv2.convexHull(contour)

        if len(hull) > 4:
            ellipse = cv2.fitEllipse(hull)
            (xe, ye), (ma, MA), angle = ellipse
        else:
            print("eye detection failed (rotate)")
            print(roiIndex)
            print(len(ui.roisEllipseEye))
            ellipse_pos = _fallback_eye_position(roiEye, roiIndex)
            print(ellipse_pos)
            return ellipse_pos, 0

        anglep = angle - 90

        xroi, yroi = roiEye.pos()
        wroi, hroi = roiEye.size()
        xroi, yroi, wroi, hroi = image.crop

        xp = int(xroi + xe - MA / 2)
        yp = int(yroi + hroi - ye - ma / 2)

        xc = MA / 2
        yc = ma / 2
        radangle = anglep / 180 * math.pi
        xrot = xc * math.cos(radangle) + yc * math.sin(radangle)
        yrot = xc * (-math.sin(radangle)) + yc * math.cos(radangle)

        vx = xrot - xc
        vy = yrot - yc
        ui.roisEllipseEye[roiIndex].descriptor = [
            xp,
            yp,
            MA,
            ma,
            -anglep,
            vx,
            vy,
            xrot,
            yrot,
        ]
        ellipse_pos = [xp - vx + xrot, yp - vy + yrot]

        return ellipse_pos, anglep

    return _fallback_eye_position(roiEye, roiIndex), 0


def tail_Track(iframe, analysimg, thresh_tail, rgn_tail):
    """Track a tail point inside the legacy rectangular tail region."""
    if analysimg is None:
        return iframe, [0, 0], 180

    if len(analysimg.shape) > 2:
        analysimg = cv2.cvtColor(analysimg, cv2.COLOR_BGR2GRAY)

    im_height, im_width = analysimg.shape[:2]
    threshValue_Tail = thresh_tail

    xroicv = int(rgn_tail[0])
    yroicv = 0
    wroi = int(rgn_tail[1] - rgn_tail[0])
    hroi = int(im_height)

    cropImgData = analysimg[yroicv:yroicv + hroi, xroicv:xroicv + wroi]
    threshTailZone = cv2.threshold(
        cropImgData,
        threshValue_Tail,
        255,
        cv2.THRESH_BINARY_INV,
    )[1]
    threshTailZone = cv2.dilate(threshTailZone, None, iterations=5)

    tailcnts = cv2.findContours(
        threshTailZone.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    if tailcnts:
        ctail = sorted(tailcnts, key=cv2.contourArea, reverse=True)[:1]
        M = cv2.moments(ctail[0])
        tailX = int(M["m10"] / M["m00"])
        tailY = int(M["m01"] / M["m00"])

        ctailX = tailX + xroicv
        ctailY = int(im_height) - tailY

        rootx, rooty = ui.mark.data["pos"][0]
        tailAngle = math.atan2((rooty - ctailY), (rootx - ctailX)) * 180 / math.pi
        tailAngle *= -1
        tailAngleCorr = tailAngle + varM.bodyAngle
    else:
        print("tail detection failed")
        tailAngleCorr = 180
        ctailX = 0
        ctailY = 0

    tail_pos = [ctailX, ctailY]

    ui.tailAngleList.append([iframe, tailAngleCorr])
    ui.tailPosList.append([iframe, tail_pos])
    print("Tail anglelist ADD", iframe)

    return iframe, tail_pos, tailAngleCorr
