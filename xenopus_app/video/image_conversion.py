# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 11:28:38 2024

@author: Courtand, Kadri 

Image conversion helpers used by the video display widgets.
"""

import cv2
import numpy as np


def gamma_LUT(gamma):
    """Build a uint8 lookup table for gamma correction."""
    inv_gamma = 1.0 / gamma
    table = np.array([
        ((i / 255.0) ** inv_gamma) * 255
        for i in np.arange(0, 256)
    ]).astype("uint8")
    return table


def convert_imageToPyqtgraph(img, vid):
    """
    Convert an OpenCV image to the orientation expected by pyqtgraph.

    Color frames are converted from BGR to RGB before applying the current
    lookup table. Grayscale frames use the same lookup table directly.
    The converted frame is stored on ``vid.currentTFrame`` for compatibility
    with the legacy display code.
    """
    if len(img.shape) > 2:
        rgb_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        display_frame = cv2.LUT(rgb_frame, vid.LUT)
    else:
        display_frame = cv2.LUT(img, vid.LUT)

    vid.currentTFrame = cv2.transpose(display_frame[::-1, :])
    return vid.currentTFrame
