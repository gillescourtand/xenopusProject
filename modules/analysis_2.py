# -*- coding: utf-8 -*-
"""
Created on Fri Nov 19 12:01:19 2021

@author: courtand
"""


analysisList=[]


class Tracking():
    def __init__(self):
        
        self.mode=False
        self.type=None #simpleTrack ou multiTrack
        self.method=None #threshold, subtract background
        self.analysisList=[]

class ImageContainer:
    """classe définissant les différentes images de travail dans un même objet
    """    
    def __init__(self):
        self.background=None
        self.backgroundCorrected=None
        self.croppedBackground=None
        self.origimage= None
        self.forAnalysis=None
        self.previousFrame=None
        self.mask=None
        self.target=None
        self.live=None
        self.croppedTarget=None
        self.lastCroppedTarget=None
        self.croppedMask=None
        
        
        