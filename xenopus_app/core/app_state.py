# -*- coding: utf-8 -*-
"""Application-wide state objects and legacy shared settings.

This module keeps the small state containers that were historically defined in
the main application file. They are intentionally lightweight and are shared by
the UI, tracking and settings modules.

The legacy class names are kept to avoid changing the existing application API.

@author: Courtand, Kadri 
"""

import time

import cv2
import pyqtgraph as pg


# Legacy shared references.
# They are populated by the main window during application startup.
ui = None
analysisSet = None


def set_context(**ctx):
    """Update legacy shared references used by older helper functions.

    Parameters
    ----------
    **ctx:
        Named objects to inject into this module namespace. Typical keys are
        ``ui`` and ``analysisSet``.
    """
    globals().update(ctx)


class Analysis_Settings:
    """Store global analysis settings shared by legacy tracking helpers.

    The class name is kept for compatibility with the historical code.
    """

    def __init__(self):
        self.framerateFactor = 1
        self.threshMode = cv2.THRESH_BINARY_INV
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        self.duration = 60
        self.nbFramesToAnalyze = 0

        self.scale = 1
        self.scaleUnit = "pixel"

    def set_framerateFactor(self, i):
        """Set the frame-rate scaling factor used by legacy analysis code."""
        self.framerateFactor = i

    def set_thresholdMethod(self, whiteIsChecked):
        """Select the OpenCV threshold mode from the background color option."""
        if whiteIsChecked:
            self.threshMode = cv2.THRESH_BINARY
        else:
            self.threshMode = cv2.THRESH_BINARY_INV


def update_settings():
    """Refresh the shared morphology kernel from the UI kernel spinbox."""
    kernelWidth = ui.openKernel_spinbox.value()
    analysisSet.kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (kernelWidth, kernelWidth),
    )


class Measure_Var:
    """Store live measurement values shared between UI and tracking code."""

    def __init__(self):
        self.var = []
        self.measuredLivefps = 0
        self.measuredPlayfps = 0
        self.lastTime_live = time.time()
        self.lastTime_play = time.time()
        self.frameCount = 0
        self.timeToUpdate = 0
        self.ctailX = []
        self.ctailY = []
        self.bodyAxis_Y = 0
        self.idxResultArray = 0
        self.bodyAngle = 0
        self.tailAngle = []
        self.tailAngleCorr = []


class Roi(pg.RectROI):
    """Rectangular ROI with a legacy position history list."""

    def __init__(self, pos, size, centered, sideScalers=False, **args):
        pg.RectROI.__init__(
            self,
            pos,
            size,
            centered,
            sideScalers=sideScalers,
            **args,
        )
        self.posList = [[0, 0]]


class Target:
    """Legacy object-tracking target description."""

    def __init__(self):
        self.name = "None"
        self.size = 0
        self.minRadius = 1
        self.alldist = 0
        self.tracker = None
        self.region = None
