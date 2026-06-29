# -*- coding: utf-8 -*-
"""
Created on Fri Nov 19 12:01:19 2021

@author: Courtand, Kadri 
"""

"""Legacy image and tracking containers.

These classes are intentionally simple. They are used by older parts of the
application to group working images and tracking options in mutable objects.
"""


analysisList = []


class Tracking:
    """Store legacy tracking mode configuration."""

    def __init__(self):
        self.mode = False
        self.type = None
        self.method = None
        self.analysisList = []


class ImageContainer:
    """Group the working images used during image processing."""

    def __init__(self):
        self.background = None
        self.backgroundCorrected = None
        self.croppedBackground = None
        self.origimage = None
        self.forAnalysis = None
        self.previousFrame = None
        self.mask = None
        self.target = None
        self.live = None
        self.croppedTarget = None
        self.lastCroppedTarget = None
        self.croppedMask = None
        self.forDisplay = None
