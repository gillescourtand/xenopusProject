# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 11:28:38 2024

@author: courtand
"""


import cv2
import numpy as np



def gamma_LUT(gamma):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
         for i in np.arange(0, 256)]).astype("uint8")
    # apply gamma correction using the lookup table
    return table         


def convert_imageToPyqtgraph(img,vid):
    #conversion de l'image pour affichage dans le plotview
    if len(img.shape)>2 : #image couleur (3e terme pour le nombre de canaux)
         #image.currentGrayFrame=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
         
         #l'image affichée est l'image couleur
         cframe=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
         gaframe=cv2.LUT(cframe, vid.LUT)
         
    else :
        #image.currentGrayFrame = img.copy()
        #gaframe=cv2.LUT(image.currentGrayFrame, video.LUT)
        gaframe=cv2.LUT(img, vid.LUT)
           
    #rotation de l'image à 90°  : cv2.transpose
    #[::-1,:] et .T pour orienter l'image correctement
    #self.img.setImage(self.frame[::-1,:].T)
    vid.currentTFrame=cv2.transpose(gaframe[::-1,:])
    return vid.currentTFrame    