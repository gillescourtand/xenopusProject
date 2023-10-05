# -*- coding: utf-8 -*-
"""
Created on Tue May  2 13:49:31 2017

@author: courtand

Application d'analyse des mouvements de nage du xénope à différents stades
- mesure des variations angulaires des yeux et de la queue lors de la nage

version développé avec python 3.5, opencv3, 64bit et interface Qt5, pyqtgraph

This version use pyqtgraph's dock widget system.

The dockarea system allows the design of user interfaces which can be rearranged by
the user at runtime. Docks can be moved, resized, stacked, and torn out of the main
window. This is similar in principle to the docking system built into Qt, but 
offers a more deterministic dock placement API (in Qt it is very difficult to 
programatically generate complex dock arrangements). Additionally, Qt's docks are 
designed to be used as small panels around the outer edge of a window. Pyqtgraph's 
docks were created with the notion that the entire window (or any portion of it) 
would consist of dockable components.


v4 : tracking des membres
v4b : - optimisation du traitement de l'image pour l'affichage
     - acceleration de la lecture
v5 : implementation de l'analyse live par buffer
v5b : fonction convert placé dans une variable local pour alléger le traitement
v5c : utilisation d'un rawImageWidget pour améliorer les performances
v8 : version stable, affichage live QtCore.QTimer.singleShot(1000, lambda: updateLabel())
    l'affichage de l'image analysée est encore à la vitesse maximale
        The QTimer class provides repetitive and single-shot timers.
        The QTimer class provides a high-level programming interface for timers. 
        To use it, create a QTimer, connect its timeout() signal to the appropriate slots, and call start().
        From then on it will emit the timeout() signal at constant intervals.
v9 : transfert de tous les affichages dans updateLabel  (peut-être utiliser plusieurs QTimer ?)
    v9a : remplacement du thread qui lance live_Grab par un objet captureimage(threading.Thread)
    v9b : - les affichages des roi et axes sont également déportés dans un qtimer
          - implémentation du player video pour le track sous forme de thread  
          - enregistrement des yeux permettant de n'en enregistrer qu'un : pb pour la création des entêtes 
v10 : correction du bug cv2 quand la roi sort de l'image--> problème de crop
v11 : - correction du calcul de l'angle de la queue : positif au dessus de l'horizontal
      - modification du système de lecture/acquisition basé sur un thread "Queue"
      - affichage partiel des graphes pendant l'analyse pour ne pas ralentir l'analyse quand les listes s'allongent
v11b : correction de l'incrémentation de l'index d'analyse sur le nombre d'images analysées et non la position dans la video 
v11bmulti : tracking de 8 segments de queue
v12 : - amélioration de la vitesse en live
      - barre de progession de remplissage du tampon video
      
v13 : 
v13b : 2019 : utilisation de la librairie pypylon de Basler : pypylon-1.4.0 + pylon5.1.0
            utilisation de genicam pour le controle de la camera)
v13c : implementation du multi-tracking pour l'analyse des mouvements de queue
v13d : choix de la couleur de fond pour le seuillage : black background or white background
v13e : modification du calcul pour la limite du déplacement latéral
v14 : utilisation des feuilles de style pour l'interface
        pylon 6.1.1
        
--nouveau nom Swim-X        
"""

#distribuable .exe win32 : cx_Freeze --> python setup_xenopus.py build

"""
création de l'executable : pyinstaller
conda install -c conda-forge pyinstaller 
>>pyinstaller MotionAnalysis_Xenopus.py

Penser à joindre le fichier css dans le dossier de l'exe'
ainsi que opencv_ffmpeg401_64.dll
"""




import os
import time

from PyQt5 import QtWidgets,QtCore
#pyqtgraph
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore as QtgCore, QtGui as QtgGui
from pyqtgraph.dockarea import *
import pyqtgraph.ptime as ptime
#from pyqtgraph.widgets.RawImageWidget import RawImageWidget
import pyqtgraph.debug as debug
import pyqtgraph.ThreadsafeTimer

import numpy as np
import math
import csv
import cv2

from threading import Thread
import sys
# import the Queue class from Python 3
if sys.version_info >= (3, 0):
	from queue import Queue
# otherwise, import the Queue class for Python 2.7
else:
	from Queue import Queue

#from numba import jit,jitclass          # import the decorator

#ftdi (trigger)
from pylibftdi import BitBangDevice, Driver

#basler
from pypylon import genicam
from pypylon import pylon

#modules perso
import pylon_parametersListmodule2e2 as baslerParameters



analysisList=[]
roisEye = []
roisEllipseEye=[]
roisEllipseEyeAngle=[]
roisEllipseTail=[]
roisLimb = []

framesBuffer=[]
dataTest=[]



class Video:
    """classe rassemblant les différents paramètres video
    """
    def __init__(self):
       
        self.fps=None #vidProp_dict['fps']=cap.get(5)
        
        
        self.buffering=False
        self.capture=None
        self.devices=[]
        self.device=None
        self.file=None
        self.firstFrame=None
        self.grabber=None
        self.isAcquiring=False
        self.LUT=None
        self.nbFrames=None #vidProp_dict['nbFrames']=cap.get(7)
        self.playing=False
        self.pos=None #vidProp_dict['framePos']= cap.get(1) -->position du frame suivant    
        self.ToRGB=None #vidProp_dict['videoToRGB']=cap.get(16)
                
        self.pixFormat=None #cam.PixelFormat.GetValue()
        self.width=None #cam.Width.GetValue() / #vidProp_dict['videoWidth']=cap.get(3)
        self.height=None #cam.Height.GetValue() / #vidProp_dict['videoHeight']=cap.get(4)
        self.resultFrameRate=None #cam.ResultingFrameRate.GetValue()
        self.enableFrameRate=None #cam.AcquisitionFrameRateEnable.GetValue()
        self.grabFrameRate=None #cam.AcquisitionFrameRate.GetValue() / #vidProp_dict['fps']=cap.get(5)
        self.offsetX=None #cam.OffsetX.GetValue()
        self.offsetY=None #cam.OffsetY.GetValue()
        self.centerX=None #cam.CenterX.GetValue()
        self.centerY=None #cam.CenterY.GetValue()
        self.gainAuto=None #cam.GainAuto.GetValue()
        self.gain=None #cam.Gain.GetValue()
        self.gamma=None #cam.Gamma.GetValue()
        self.exposure=None #cam.ExposureTime.GetValue()
        self.sensorReadoutMode=None #cam.SensorReadoutMode.GetValue()
        
        

class ImageContainer:
    """classe définissant les différentes images de travail dans un même objet
    """    
    def __init__(self):
        self.background=None
        self.backgroundCorrected=None
        self.origimage= None
        self.currentGrayFrame =None
        self.currentFrame = None
        self.currentTFrame = None #image transposée pour affichage dans plotview
        self.forAnalysis=None
        self.mask=None
        self.target=None
        self.live=None
        self.crop=None

class Measure_Var :
    """classe définissant les varaibles utilisées pour les mesures
    """
    def __init__(self):
        
        self.var=[]
        self.measuredLivefps=00
        self.measuredPlayfps=00
        self.lastTime_live=ptime.time()
        self.lastTime_play=ptime.time()
        self.frameCount=0
        self.timeToUpdate=0
        self.ctailX=0
        self.ctailY=0
        self.bodyAxis_Y=0
        self.idxResultArray=0
        self.bodyAngle=0

        
        
class Roi(pg.RectROI):
    """classe héritant des roi rectangulaires de pyqtgraph
    régions d'intérêt pouvant être tracée par l'utilisateur :
    ajout d'un attribut : liste des positions sur une séquence
     
    """
    def __init__(self, pos, size, centered, sideScalers=False, **args):
        pg.RectROI.__init__(self, pos, size, centered, sideScalers=False, **args)
        #self.name="None" # self.__name="None" pour protéger la valeur après instanciation      
        #self.time={0:0} # {numéro_ou_nom_de_phase:temps_passé_dans_la_zone_pour_cette_phase}
        #self.function="None"
        #liste des coordonnées de la roi
        self.posList=[[0,0]]
        

class Target:
    """classe caractérisant la cible à suivre :
    taille, couleur...
    """
    def __init__(self):
        self.name="None"
        self.size=0
        self.minRadius=1
        self.alldist=0
        self.tracker=None
        self.region=None
        

class FileVideoStream_Play:
    def __init__(self, path, queueSize=300):
        # initialize the file video stream along with the boolean
        # used to indicate if the thread should be stopped or not
        print(path)
        self.stream = cv2.VideoCapture(path)
        self.stopped = False
         
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=queueSize)

    def start(self,nframe):
        # start a thread to read frames from the file video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        self.stream.set(1,nframe)
        t.start()
        return self

    def update(self):
        #boucler jusqu'à la fin de la video get(7)=nbre de frames
        endFrame=self.stream.get(7)
        i=0
        #while True:
        while self.stream.get(1)<endFrame: 
            # if the thread indicator variable is set, stop the
            # thread
            if self.stopped:
                return
            
            # otherwise, ensure the queue has room in it
            if not self.Q.full():
                # read the next frame from the file
                (grabbed, frame) = self.stream.read()
                #tFrame=convert_imageToPyqtgraph(frame)
                #print("load ",i,grabbed)
                i+=1
                # if the `grabbed` boolean is `False`, then we have
                # reached the end of the video file
                if not grabbed:
                    self.stop()
                    return

                # add the frame to the queue
                self.Q.put(frame)
                
                now = ptime.time()
                dt = now - varM.lastTime_live
                varM.lastTime_live = now
                if varM.measuredLivefps is None:
                    varM.measuredLivefps = 1.0/dt
                else:
                    s = np.clip(dt*3., 0, 1)
                    varM.measuredLivefps = varM.measuredLivefps * (1-s) + (1.0/dt) * s
                #print("load: ",dt)
                """
                #avec un framerate fps pour le maintenir on ajoute un timer pour ralentir l'affichage si besoin
                fpsTime=1/30
                timeToSleep=fpsTime-dt
                if timeToSleep>0:
                    time.sleep(timeToSleep)
                    #print(timeToSleep)
                """
            else:
                time.sleep(2.0)

    def read(self):
        # return next frame in the queue
        return self.Q.get()
        
    def more(self):
        # return True if there are still frames in the queue
        return self.Q.qsize() > 0    
        
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True     

 


class FileVideoStreamLive:
    def __init__(self, path, queueSize=2000):
        # initialize the file video stream along with the boolean
        # used to indicate if the thread should be stopped or not
        print(path)
        
        self.stream = video.device
        self.stopped = False
         
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=queueSize)

    def start(self):
        # start a thread to read frames from the file video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely
        i=0
        self.stream.StartGrabbing(pylon.GrabStrategy_OneByOne) #default strategy
        while 1 :
            if not self.stream.IsGrabbing():
                break
    
            grabResult = self.stream.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            # Camera.StopGrabbing() is called automatically by the RetrieveResult() method
    
            # When the cameras in the array are created the camera context value
            # is set to the index of the camera in the array.
            # The camera context is a user settable value.
            # This value is attached to each grab result and can be used
            # to determine the camera that produced the grab result.
            #   cameraContextValue = grabResult.GetCameraContext()
    
            # Print the index and the model name of the camera.
            #print("Camera ", cameraContextValue, ": ", cameras[cameraContextValue].GetDeviceInfo().GetModelName())
    
            # Now, the image data can be processed.
            #print("GrabSucceeded: ", grabResult.GrabSucceeded())
            #print("SizeX: ", grabResult.GetWidth())
            #print("SizeY: ", grabResult.GetHeight())
            frame = grabResult.GetArray()   
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, ensure the queue has room in it
            if not self.Q.full():
                # read the next frame from the file
                #(grabbed, frame) = self.stream.read()
                #tFrame=convert_imageToPyqtgraph(frame)
                #print(i)
                i+=1
                # if the `grabbed` boolean is `False`, then we have
                # reached the end of the video file
                """
                if not grabbed:
                    self.stop()
                    return
                """        
                # add the frame to the queue
                self.Q.put(frame)
                #calcul du framerate
                now = ptime.time()
                dt = now - varM.lastTime_live
                varM.lastTime_live = now
                if varM.measuredLivefps is None:
                    varM.measuredLivefps = 1.0/dt
                else:
                    s = np.clip(dt*3., 0, 1)
                    varM.measuredLivefps = varM.measuredLivefps * (1-s) + (1.0/dt) * s
                #print("grab :",dt)
            else : 
                print("Fuuuuuuuuuuuuuuuuuuuuuuuuuuuull")

    def read(self):
        # return next frame in the queue
        return self.Q.get()
        
    def more(self):
        # return True if there are still frames in the queue
        return self.Q.qsize() > 0    
        
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True     

""" 
class FileVideoStreamLiveCV2:
    def __init__(self, path, queueSize=1024):
        # initialize the file video stream along with the boolean
        # used to indicate if the thread should be stopped or not
        print(path)
        
        #self.stream = pypylon.factory.create_device(video.devices[0])
        #self.stream.open()
        self.stream = cv2.VideoCapture(0)
        #paramètrage de la camera
        #set_Video_Properties(video.capture)
        video.LUT=gamma_LUT(1.8) 
        #récupération des propriétés de la vidéo
        #get_Video_Properties(video.capture)
        self.stopped = False
         
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=queueSize)

    def start(self):
        # start a thread to read frames from the file video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely
        i=0
        while True:
            # if the thread indicator variable is set, stop the
            # thread
            if self.stopped:
                return
            
            # otherwise, ensure the queue has room in it
            if not self.Q.full():
                # read the next frame from the file
                (grabbed, frame) = self.stream.retrieve()
                tFrame=convert_imageToPyqtgraph(frame)
                #print(i)
                i+=1
                # if the `grabbed` boolean is `False`, then we have
                # reached the end of the video file
                if not grabbed:
                    self.stop()
                    return

                # add the frame to the queue
                self.Q.put(tFrame)
            

    def read(self):
        # return next frame in the queue
        return self.Q.get()
        
    def more(self):
        # return True if there are still frames in the queue
        return self.Q.qsize() > 0    
        
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True                 
        
"""
class CamParamWidget(baslerParameters.PyqtgParamWidget):
    """
    surcharge de la classe PyqtgParamWidget
    met à jour les objets video avec les valeurs qui ont été modifiées dans le widget
    """
    def __init__(self):
        baslerParameters.PyqtgParamWidget.__init__(self)
    ## If anything changes in the tree, print a message
    def change(self,param, changes):
        print("")
        print("change in tree (subclass):")
        for param, change, data in changes:
#            path = self.params.childPath(param)
            childName = param.name()
#            if path is not None:
#                childName = '.'.join(path)
#            else:
#                childName = param.name()
            #tester si les modifications concernent un paramètre nécessitant l'arrêt de la caméra pour être modifié
            #AccessException: Node is not writable : AccessException thrown in node 'PixelFormat' while calling 'PixelFormat.FromString()'
            if childName=="Width" or  \
                childName=="Height" or \
                childName=="Acquisition Frame Rate [Hz]" or \
                childName=="Pixel Format":
                    stopCam=True
            else : stopCam=False    
                            
            print('  parameter: %s'% childName)
            print('  change:    %s'% change)
            print('  data:      %s'% str(data))
            print('  ----------')
            
        update_video_parameters(stopCam)
        set_cam_parameters(stopCam)


#----------------------------------------------------------------------------------TRIGGER
def get_ftdi_device_list():
    """
    return a list of lines, each a colon-separated
    vendor:product:serial summary of detected devices
    """
    dev_list = []
    for device in Driver().list_devices():
        # list_devices returns bytes rather than strings
        dev_info = map(lambda x: x.decode('latin1'), device)
        # device must always be this triple
        vendor, product, serial = dev_info
        dev_list.append("%s:%s:%s" % (vendor, product, serial))
    return dev_list        
        
        
def trigger():
    try :
        devices_list=get_ftdi_device_list()
        print("ftdi : ",devices_list)
        #QtWidgets.QMessageBox.warning(None,"debug",str(devices_list),QtWidgets.QMessageBox.Ok)
        #with BitBangDevice('FTE00P4L') as bb:
        if len(devices_list)!=0:
            with BitBangDevice() as bb:
                bb.direction = 0x0F  # four LSB are output(1), four MSB are input(0)
                bb.port |= 2         # set bit 1
                bb.port &= 0xFE      # clear bit 0 
                    
    except sys.exc_info()[0] as e:
        #e = sys.exc_info()[0]
        print("error sys : ",e)
        QtWidgets.QMessageBox.warning(None,"Error",str(e)) 
        #QMessageBox.warning(self, '', str(e))
    except IOError as e :
        print(e)
        QtWidgets.QMessageBox.warning(None,"Error",str(e))

#acquisition video-----------------------------------------------------------------------------------------------LIVE VIDEO

def get_Video_Properties(video_device):
    #fps = cap.get(cv2.cv.CV_CAP_PROP_FPS)=cap.get(5)
    #initialisation du format video : niveaux de gris
    video.pixFormat= video_device.PixelFormat
    video.fps=video_device.ResultingFrameRate
    video.width=video_device.Width
    video.height=video_device.Height   
               
    #print infos
    print("w:",video.width," ; ","h:",video.height)
    print(video.fps,"fps")
    print("pixel format : ",video.pixFormat)
    #if vidProp_dict['videoWidth']<512 : image.zoom=2    
    
def set_Video_Properties(video_device):
    #------------------------------------------------------------
    # Go to full available speed
    #video.capture.properties['DeviceLinkThroughputLimitMode'] = 'Off'
    video_device.PixelFormat.SetValue(PixelFormat_Mono8)
    video_device.ExposureAuto="Off"
    #>>>cam.properties['ExposureTime'] : 10000.0
    #video_device.properties['Width']=800
    #video_device.properties['Height']=400 
    #video_device.properties['ExposureTime'] = ui.exposure_slider.value()#1000
    video_device.AcquisitionFrameRateEnable.SetValue(True)
    #video_device.properties['AcquisitionFrameRate']=200
    #video_device.properties['OffsetX']=64
                                                
    #--------------------------------------------------------------    

def set_offset_Y(offset):
    print("offset")
    #video.grabber.stream.properties['OffsetX']=float(offset)
    video.grabber.stream.properties['AcquisitionFrameRate']=2
    


        
def read_and_analyze():
                            
    # grab the frame from the threaded video file stream    
    frame = video.grabber.read()
            
    # display the size of the queue on the frame
    #cv2.putText(frame, "Queue Size: {}".format(fvs.Q.qsize()),
    #	(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)	
    image.forAnalysis= frame
                       
    #text=str(i)+"/"+str(len(framesBuffer))
    #ui.lenBuffer_label.setText(text)
    #print("buffer",len(framesBuffer))
    #------------------------------------------------------------------------------------------------------ANALYSIS
    
    if ui.track_checkBox.isChecked()==True :
        
        for analysis in analysisList:
            
            analysis(varM.frameCount,image.forAnalysis)
        varM.frameCount+=1
        #varM.idxResultArray=varM.frameCount
        varM.idxResultArray+=1
        #print("analysis : ",varM.frameCount)
          
    #---------------------------------------------------------------------------------------------------------------------
                
    # show the frame 
    #tFrame=convert_imageToPyqtgraph(frame)
    #ui.img.setImage(tFrame,autoLevels=False)
    #app.processEvents()  ## force complete redraw for every plot
    


"""       
def import_live():
    #frame = image.live
    frame=image.currentGrayFrame.copy()
    image.forAnalysis=frame
    tFrame=convert_imageToPyqtgraph(frame)
    ui.img.setImage(tFrame,autoLevels=False)            
"""       

def init_acquisitionDevice():
    print("connecting device...")
    try :
        # #>>>available_cameras : [<DeviceInfo Basler acA2040-90um (xxxxxxx)>]
        # #video.devices = pypylon.factory.find_devices()
        # # Get all attached devices and exit application if no device is found.
        # # Get the transport layer factory.
        video.devices = pylon.TlFactory.GetInstance().EnumerateDevices()
        print(f"devices: {video.devices}")
        # Only look for cameras supported by Camera_t
        info = pylon.DeviceInfo()
        info.SetDeviceClass("BaslerUsb")
        video.device = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        
        print("Connected device : ", video.device.GetDeviceInfo().GetModelName())
        ui.videoName_label.setText(video.device.GetDeviceInfo().GetModelName())
        video.device.Open()
        #récupération des propriétés de la vidéo/camera
        get_cam_parameters(video.device)
        #chargement des valeurs dans l'interface de réglage
        init_paramTree()
        #paramètrage de la camera
        set_cam_parameters(video.device)
        set_Video_Properties
        #video.LUT=gamma_LUT(1.8) 
        activate_interface("live video")
    except genicam.GenericException as e:
        # Error handling.
        print("An exception occurred.", str(e))#.GetDescription())
        #    QtWidgets.QMessageBox.warning(None,"Error",str(ValueError),QtWidgets.QMessageBox.Ok)      
            
   
            
            
#modules indépendants *************************************************************************************************

def open_video():
   
    filePath,_=QtWidgets.QFileDialog.getOpenFileName(
                    None,
                    "Open video file",
                    QtCore.QDir.homePath(),
                    "Video file (*.avi;*.mkv;*.mp4;*.mov;*.flv)" # opencv ne prend en charge que le format avi
                )
    print(filePath)
    
    return filePath




    
def gamma_LUT(gamma):
    
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
 
    # apply gamma correction using the lookup table
    return table  

       
#**********************************************************************************************************************
def activate_interface(module):
    #rend accessible les boutons pour la lecture de la video
    if module=="load video":
        ui.playVideo_btn.setEnabled(True)
        ui.playVideo_btn.setChecked(False)
        ui.timeLine_slider.setEnabled(True)
        ui.stepFwdVideo_btn.setEnabled(True)
        ui.stepBwdVideo_btn.setEnabled(True)
       
    elif module=="live video":
        ui.liveVideo_btn.setEnabled(True)
        #ui.importLive_btn.setEnabled(True)
        

def load_video():
    #tester si le bouton "track" est coché, si oui décocher
    if ui.track_checkBox.isChecked : ui.track_checkBox.setChecked(False)
    video.file=open_video()
    print("file : ",video.file)
    if video.file!="" :
        ui.videoName_label.setText(video.file)
        
        init_video(str(video.file))
        #initialisation du slider timeline
        ui.timeLine_slider.setMaximum(video.nbFrames)
        ui.timeLine_slider.setValue(video.firstFrame)
        
        #activation de l'interface
        activate_interface("load video")
        
        play_frame(video.firstFrame) #pour afficher la video car sld_TimeLine n'est pas modifié

def playpause_Video():
    #print("play video :",ui.playVideo_btn.isChecked())
    if video.file==None:
        message="You have to open a video first"
        QtWidgets.QMessageBox.warning(None,"no video",str(message),QtWidgets.QMessageBox.Ok)    
                
    else:
        if ui.playVideo_btn.isChecked()==True:
            
            video.playing=True
            
            #video_Play()
            #fvs = FileVideoStream_Play(video.file).start()
            video.grabber.stopped = False
            
            #vider le buffer
            #video.stream.task_done()
            #video.grabber.Q.queue.clear()
                         
            video.grabber.start(ui.timeLine_slider.value())
            print("grabber started")
            time.sleep(1.0)
            video_Play_Stream(video.grabber)
            print("streamplayer stopped")
            ui.playVideo_btn.setChecked(False)
            pause_Video()
            
        else :
            pause_Video()
                
                
def pause_Video():
    video.playing=False
    video.grabber.stop()
    video.grabber.Q.queue.clear()
    ui.progressBuffer.setValue(int(video.grabber.Q.qsize()/video.grabber.Q.maxsize*100))
    
    updatePlot_Full()
                
    
def forward_Video():
    #self.sld_TimeLine.setValue(self.sld_TimeLine.value()+1)
    ui.timeLine_slider.setValue(ui.timeLine_slider.value()+1)
        
        
def backward_Video():
    #self.sld_TimeLine.setValue(self.sld_TimeLine.value()-1)
    ui.timeLine_slider.setValue(ui.timeLine_slider.value()-1)
    
            
def slider_Move():
    #if dialog1.pause==True :
    if ui.playVideo_btn.isChecked()==False :
        play_frame(ui.timeLine_slider.value())
        
        
def play_frame(framePos):
    #print(framePos)
    #cv2.VideoCapture.set(propId, value) → retval
    #propId=1 : CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next
    retVal_set=video.capture.set(1,framePos)
    if retVal_set==True : (retVal_read, image.currentFrame) = video.capture.read()
    else : print("wrong frame index")
    #print(video.capture.get(0),video.capture.get(1),framePos)
    #video.capture.get(1) : CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
    
    if retVal_read==True:
               
        #conversion de l'image pour affichage dans le plotview        
        tFrame=convert_imageToPyqtgraph(image.currentFrame)
        #cv2.imshow('Visu',image.currentFrame)        
        ui.img.setImage(tFrame,autoLevels=False)
        image.forAnalysis=image.currentGrayFrame
        #---------------------------------------------------------------------------------------------------------ANALYSIS
                
        
        if ui.track_checkBox.isChecked() :
            for analysis in analysisList:
                #print(analysis)
                analysis(framePos,image.forAnalysis)
            varM.idxResultArray+=1
            update_viewBox()    
            app.processEvents()  ## force complete redraw for every plot
            #permet d'accéder à l'interface de manière fluide

        #---------------------------------------------------------------------------------------------------------------------
            
    else:
        print("can't read this frame")
        #TODO: ajouter un message d'erreur ?


def convert_imageToPyqtgraph(img):
    #conversion de l'image pour affichage dans le plotview
    if len(img.shape)>2 : #image couleur (3e terme pour le nombre de canaux)
         image.currentGrayFrame=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
         #l'image affichée est l'image couleur
         cframe=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
         gaframe=cframe#cv2.LUT(cframe, video.LUT)
         
    else :
        image.currentGrayFrame = img.copy()
        gaframe=image.currentGrayFrame#cv2.LUT(image.currentGrayFrame, video.LUT)
           
    #rotation de l'image à 90°  : cv2.transpose
    #[::-1,:] et .T pour orienter l'image correctement
    #self.img.setImage(self.frame[::-1,:].T)
    
    image.currentTFrame=cv2.transpose(gaframe[::-1,:])
    
    return image.currentTFrame    


def update_tailRegionPosition():
    #print(ui.regionlr.getRegion())
    regionPos1,regionPos2=ui.regionlr.getRegion()
#    text = "{} : {}".format(int(regionPos1), int(regionPos2))
#    print(text)
    #dialog1.txt_TailValue.setText(text)
    #ui.tailRootPos_label.setText(text)
    #TODO : dialog1.txt_TailValue.textChanged.connect(update_TailRegionPos)    
    
    segmentLen=50
    for i,region in enumerate(ui.listTailSegments[1:]):
        i+=1
        region.setRegion([regionPos1-i*segmentLen,regionPos2-i*segmentLen])
          
            
def init_analysisMode():
    
    if len(analysisList)>0 : analysisList[:]=[]
    if ui.manipType_comboBox.currentText()=="eyes-tail track":
        analysisList.append(eye_Track)
        analysisList.append(tail_Track)
        #analysisList.extend()
    elif ui.manipType_comboBox.currentText()=="eyes track only":
        analysisList.append(eye_Track)
        
    elif ui.manipType_comboBox.currentText()=="eyes-limbs track":
        analysisList.append(eye_Track)
        analysisList.append(limb_Track)

    elif ui.manipType_comboBox.currentText()=="limbs track only":
        analysisList.append(limb_Track)
        
    elif ui.manipType_comboBox.currentText()=="test live":
        analysisList.append(test_Live)
    
    else : print("aucun choix")
    
"""
-----------------------------------------------------------------Camera communication                            
"""

def init_paramTree():
    paramTree.params.param('Acquisition parameter','Pixel Format').setValue(video.pixFormat)
    paramTree.params.param('Acquisition parameter','Width').setValue(video.width)
    paramTree.params.param('Acquisition parameter','Height').setValue(video.height)
    paramTree.params.param('Acquisition parameter','Resulting Frame Rate [Hz]').setValue(video.resultFrameRate)
    paramTree.params.param('Acquisition parameter','Enable Frame Rate').setValue(video.enableFrameRate)
    paramTree.params.param('Acquisition parameter','Acquisition Frame Rate [Hz]').setValue(video.grabFrameRate)
    paramTree.params.param('AOI Control','Offset X').setValue(video.offsetX)
    paramTree.params.param('AOI Control','Offset Y').setValue(video.offsetY)
    paramTree.params.param('AOI Control','Center X').setValue(video.centerX)
    paramTree.params.param('AOI Control','Center Y').setValue(video.centerY)
    paramTree.params.param('Analog Control','Gain Auto').setValue(video.gainAuto)
    paramTree.params.param('Analog Control','Gain').setValue(video.gain)
    paramTree.params.param('Analog Control','Gamma').setValue(video.gamma)
    paramTree.params.param('Acquisition Control','Exposure Time [µs]').setValue(video.exposure)
    paramTree.params.param('Acquisition Control','Sensor Readout Mode').setValue(video.sensorReadoutMode)

def get_cam_parameters(cam):
    """
    récupérer les valeurs des paramètres d'acquisition stockés dans la caméra
    et remplir l'interface de réglage parameterTree
    """
    #TODO : if cam == basler Ace
    #// Get the parameters for setting the image area of interest (Image AOI).
    #const CIntegerPtr width = control.GetNode("Width");
    """
    width = control.GetNode("Width")
    height = control.GetNode("Height")
    offsetX = control.GetNode("OffsetX")
    offsetY = control.GetNode("OffsetY")
    fps=control.GetNode("AcquisitionFrameRate")
    """
    video.pixFormat=cam.PixelFormat.GetValue()
    video.width=cam.Width.GetValue()
    video.height=cam.Height.GetValue()
    video.resultFrameRate=cam.ResultingFrameRate.GetValue()
    video.enableFrameRate=cam.AcquisitionFrameRateEnable.GetValue()
    video.grabFrameRate=cam.AcquisitionFrameRate.GetValue()
    video.offsetX=cam.OffsetX.GetValue()
    video.offsetY=cam.OffsetY.GetValue()
    video.centerX=cam.CenterX.GetValue()
    video.centerY=cam.CenterY.GetValue()
    video.gainAuto=cam.GainAuto.GetValue()
    video.gain=cam.Gain.GetValue()
    video.gamma=cam.Gamma.GetValue()
    video.exposure=cam.ExposureTime.GetValue()
    video.sensorReadoutMode=cam.SensorReadoutMode.GetValue()

def set_cam_parameters(stopCam):
    print("set_cam_parameters")
    video.device
    if video.device.IsOpen():
        if video.isAcquiring==True and stopCam==True :
            message="can't update settings, camera is currently used"
            print(message)
            QtWidgets.QMessageBox.warning(None,"Error",str(message),QtWidgets.QMessageBox.Ok)
            QtWidgets.QMessageBox.warning(None,"Error",str("stop grabbing"),QtWidgets.QMessageBox.Ok)
            ui.liveVideo_btn.setChecked(False)
            grab_video()
            update_cam_parameters(video.device)
        elif video.isAcquiring==True and stopCam==False :
            update_cam_liveParameters(video.device)
        else :
            update_cam_parameters(video.device)
            update_cam_liveParameters(video.device)
        
    else : 
        print("No camera found !")

def update_cam_parameters(cam):
    print("")
    print("set_cam_parameter acquisition parameters")
    cam.PixelFormat.SetValue(video.pixFormat)
    cam.Width.SetValue(video.width)
    cam.Height.SetValue(video.height)
    #cam.ResultingFrameRate.SetValue(video.resultFrameRate)
    #cam.AcquisitionFrameRateEnable.SetValue(video.enableFrameRate)
    #forcer la valeur "frameRateEnable" à TRUE
    if cam.AcquisitionFrameRateEnable.GetValue()==False :
        cam.AcquisitionFrameRateEnable.SetValue(True)
    cam.AcquisitionFrameRate.SetValue(video.grabFrameRate)
    cam.SensorReadoutMode.SetValue(video.sensorReadoutMode)

def update_cam_liveParameters(cam) :
    
    #TODO : tester la valeur de Center : si il est True : impossible de changer la valeur de Offset correspondant --> erreur
    cam.CenterX.SetValue(video.centerX)
    cam.CenterY.SetValue(video.centerY)
    try :
        cam.OffsetX.SetValue(video.offsetX)
        cam.OffsetY.SetValue(video.offsetY)
    except genicam.OutOfRangeException as e :
        print(e)
    cam.GainAuto.SetValue(video.gainAuto)
    cam.Gain.SetValue(video.gain)
    cam.Gamma.SetValue(video.gamma)
    cam.ExposureTime.SetValue(video.exposure)

    
def update_video_parameters(stopCam):
    print("update_video_parameters")
    if video.device.IsOpen():
        video.pixFormat=paramTree.params.param('Acquisition parameter','Pixel Format').value()
        video.width=paramTree.params.param('Acquisition parameter','Width').value()
        video.height=paramTree.params.param('Acquisition parameter','Height').value()
        #----
        #envoyer la nouvelle valeur de taille de capteur à la camera pour le calcul du framerate automatique
        #TODO : pour cela la camera doit être déconnectée
        #if not video.device.IsGrabbing():
        if video.isAcquiring==True and stopCam==True :
            ui.liveVideo_btn.setChecked(False)
            grab_video()
        
            try :
                video.device.Width.SetValue(video.width)
                video.device.Height.SetValue(video.height)
                newValueResFps=video.device.ResultingFrameRate.GetValue()
                print("new resfps : ",newValueResFps)
                paramTree.params.param('Acquisition parameter','Resulting Frame Rate [Hz]').setValue(video.resultFrameRate)
                #TODO : ajuster la limite Offset en fonction de la taille de l'image
                newOffsetXLimit=video.device.OffsetX.GetMax()
                print("new OffsetXLimit : ",newOffsetXLimit)
                paramTree.params.param('AOI Control','Offset X').setLimits((0,newOffsetXLimit))
            #except ValueError as e :
            #   QtWidgets.QMessageBox.warning(None,"Error",str(ValueError),QtWidgets.QMessageBox.Ok)
            except genicam.GenericException as e:
                # Error handling
                print("An exception occurred.", e)
                QtWidgets.QMessageBox.warning(None,"Error",str(e),QtWidgets.QMessageBox.Ok)
            
        #----
        video.resultFrameRate=paramTree.params.param('Acquisition parameter','Resulting Frame Rate [Hz]').value()
        video.enableFrameRate=paramTree.params.param('Acquisition parameter','Enable Frame Rate').value()
        video.grabFrameRate=paramTree.params.param('Acquisition parameter','Acquisition Frame Rate [Hz]').value()
        print("new fps : ",video.grabFrameRate)
        video.offsetX=paramTree.params.param('AOI Control','Offset X').value()
        video.offsetY=paramTree.params.param('AOI Control','Offset Y').value()
        video.centerX=paramTree.params.param('AOI Control','Center X').value()
        video.centerY=paramTree.params.param('AOI Control','Center Y').value()
        video.gainAuto=paramTree.params.param('Analog Control','Gain Auto').value()
        video.gain=paramTree.params.param('Analog Control','Gain').value()
        video.gamma=paramTree.params.param('Analog Control','Gamma').value()
        video.exposure=paramTree.params.param('Acquisition Control','Exposure Time [µs]').value()
        video.sensorReadoutMode=paramTree.params.param('Acquisition Control','Sensor Readout Mode').value()
    else :
        print("no camera open")                                            
    #--------------------------------------------------------------        
"""
---------------------------------------------------------------------------------------------------------------------
"""

def init_video(videoPath):
    #video=Video()
    video.file=videoPath
    try:
        video.capture=cv2.VideoCapture(video.file)
        ret=video.capture.open(video.file)
        if ret==False: 
            print("False")
            QtWidgets.QMessageBox.warning(None,"Tracking",str("video error"),QtWidgets.QMessageBox.Ok)
    except cv2.error as e:
        print(e)
        QtWidgets.QMessageBox.warning(None,"Tracking",str(e),QtWidgets.QMessageBox.Ok)
    
    #vidProp_dict['imageType']=image.dtype
    #video.capture.set(5,200) #set fps
    video.ToRGB=video.capture.get(16) #vidProp_dict['videoToRGB']=cap.get(16)
    video.width=video.capture.get(3) #vidProp_dict['videoWidth']=cap.get(3)
    video.height=video.capture.get(4) #vidProp_dict['videoHeight']=cap.get(4)      
    video.fps=video.capture.get(5) #vidProp_dict['fps']=cap.get(5)
    video.nbFrames=video.capture.get(7) #vidProp_dict['nbFrames']=cap.get(7)
    #video.LUT=gamma_LUT(1.8)
    video.firstFrame=1
    
    #affichage des propriétés de la vidéo
    print('convert : '+str(video.ToRGB))
    print('Frames= '+str(video.nbFrames)+' at '+str(video.fps)+'fps')
    print('frame W = ' +str(video.width)+' - H = '+str(video.height))
    
    #initialisation du slider timeline
    ui.timeLine_slider.setMaximum(int(video.nbFrames))
    ui.timeLine_slider.setValue(video.firstFrame)
       
    #play_frame(video.firstFrame) #pour afficher la video car sld_TimeLine n'est pas modifié
    #intancier le FileVideoStream_Play pour la mise en buffer
    video.grabber = FileVideoStream_Play(video.file)


def video_Play_Stream(fvs):
    """
    lecture des frames accumulés dans le tampon file video stream
    """
    if ui.track_checkBox.isChecked()==True :
        QtCore.QTimer.singleShot(50, lambda: updatePlot())
    QtCore.QTimer.singleShot(50, lambda: update_analysis())
    # loop over frames from the video file stream
    while fvs.more():
        
        # grab the frame from the threaded video file stream, resize
        # it, and convert it to grayscale (while still retaining 3
        # channels)
        #condition d'arrêt
        if video.playing==False:
            break
        #prendre une image dans le buffer
        frame = fvs.read()
        
        """                    
        # display the size of the queue on the frame
        cv2.putText(frame, "Queue Size: {}".format(fvs.Q.qsize()),
        	(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)	
        """ 
        # show the frame 
        tFrame=convert_imageToPyqtgraph(frame)
        ui.img.setImage(tFrame,autoLevels=False)
        ui.timeLine_slider.setValue(ui.timeLine_slider.value()+1)
        nframe=ui.timeLine_slider.value()
        
        #-----------------------------------------------------------------------------------------------ANALYSIS
           
        if ui.track_checkBox.isChecked()==True :
            image.forAnalysis=image.currentGrayFrame
            for analysis in analysisList:
                #print(analysis)
                #varM.idxResultArray=nframe
                analysis(nframe,image.forAnalysis)
            varM.idxResultArray+=1
                  
        #---------------------------------------------------------------------------------------------------        

        now = ptime.time()
        dt = now - varM.lastTime_play
        varM.lastTime_play = now
        if varM.measuredPlayfps is None:
            varM.measuredPlayfps = 1.0/dt
        else:
            s = np.clip(dt*3., 0, 1)
            varM.measuredPlayfps = varM.measuredPlayfps * (1-s) + (1.0/dt) * s
        #print("play : ",dt)
        #avec un framerate fps pour le maintenir on ajoute un timer pour ralentir l'affichage si besoin
        fpsTime=1/35
        timeToSleep=fpsTime-dt
        if timeToSleep>0:
            time.sleep(timeToSleep)
            #print(timeToSleep)                   
        ui.measuredPlayfps_Label.setText('%0.2f fps' % varM.measuredPlayfps)
        
        app.processEvents()  ## force complete redraw for every plot
    print("fvs.more=",fvs.more())
    ui.track_checkBox.setChecked(False)
            

        
def grab_video():
    """
    récupération et affichage du flux video de la camera connectée
    #>>>pypylon.pylon_version.version : '5.0.1.build_6388'
    #tester si une caméra est disponible
    """
    #print("grab_video")
    if ui.liveVideo_btn.isChecked()==True:
        video.isAcquiring=True
        
        try :
            #>>>available_cameras : [<DeviceInfo Basler acA2040-90um (xxxxxxx)>]
            #video.devices = pypylon.factory.find_devices()
            # Get all attached devices and exit application if no device is found.
            # Get the transport layer factory.
            
            #tester si un ou plusieurs périphériques sont connectés
            tlFactory = pylon.TlFactory.GetInstance()
            video.devices = tlFactory.EnumerateDevices()
            print("video.devices : ",str(video.devices))            
            
            video.grabber = FileVideoStreamLive("live").start()
            video.file="video live"
            time.sleep(0.1)
                            
            varM.timeToUpdate=ptime.time()
            QtCore.QTimer.singleShot(50, lambda: update_analysis())
                           
            while ui.liveVideo_btn.isChecked()==True:
                #tant que le buffer de streaming n'est pas vide
                if video.grabber.more():
                    read_and_analyze()
                    #print("read")
                    #calcul du framerate
                    now = ptime.time()
                    dt = now - varM.lastTime_play
                    varM.lastTime_play = now
                    if varM.measuredPlayfps is None:
                        varM.measuredPlayfps = 1.0/dt
                    else:
                        s = np.clip(dt*3., 0, 1)
                        varM.measuredPlayfps = varM.measuredPlayfps * (1-s) + (1.0/dt) * s
                    #print("dt read:",dt)
                    #avec un framerate fps pour le maintenir on ajoute un timer pour ralentir l'affichage si besoin
                    #fpsTime=1/video.fps
                    #time.sleep(fpsTime)
                    #timeToSleep=fpsTime-dt
                    #if timeToSleep>0:
                        #time.sleep(timeToSleep)
                else:
                    #print("wait...")
                    app.processEvents() 
                #    #calcul du temps pour rafraichir l'affichage à 30 images par seconde
                #if now-varM.timeToUpdate>0.03 : 
                #    app.processEvents()
        except genicam.GenericException as e:
            # Error handling.
            print("An exception occurred.", str(e))#.GetDescription())
            
            print("stooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooop")    
           
            #-------------------------------------------------------------------------------------------------------
          
       
    else :#bouton "live" désactivé
        video.isAcquiring=False
        print("video.isAcquiring=",video.isAcquiring)
        # wait a while to allow a processor to finish
        time.sleep(0.5)
        #while video.stream.Q.size>0:
        #    read_and_analyze()
             
        video.grabber.stop()
        time.sleep(0.5)
        try :
            video.grabber.stream.StopGrabbing()
        except AttributeError as e :
            print(e)
        print("StopGrabbing")
        #video.grabber.stream.Close() #la camera ne doit pas être fermée pour modifier les paramètres d'acquisition
        #print("camera closed")
        #TODO : insérer un appel à Close() pour déconnecter la caméra avant de fermer l'application
        


def reset_Buffer():
    framesBuffer[:]=[]
    ui.lenBuffer_label.setText(str(len(framesBuffer)))

def reset_Plot():
    
    #mettre l'analyse sur pause
    #ui.playVideo_btn.setChecked(False)
    #vider toutes les listes:
    for eyeEllipse in roisEllipseEye :
        eyeEllipse.angleList[:]=[]
        #eyeEllipse.angleList=np.zeros(100000,dtype=float)
        eyeEllipse.yList[:]=[]
                
    ui.tailEllipse.angleList[:]=[]
    #ui.tailEllipse.angleList=np.zeros(100000,dtype=float)
    
    try:
        #ui.w3.plot.clear()
        dataArray = np.asarray([])
        ui.w3.plot(dataArray,clear=True)
        ui.w4.plot(dataArray,clear=True)
        ui.w5.plot(dataArray,clear=True)
        ui.w6.plot(dataArray,clear=True)
        ui.w7.plot(dataArray,clear=True)
        print("plot reset ok ")
    except ValueError as e :
            QtWidgets.QMessageBox.warning(None,"Error",str(ValueError),QtWidgets.QMessageBox.Ok) 
    

            
def updatePlot() :
    
    if ui.track_checkBox.isChecked()==True:
        #les plots ne sont plus affichés en entier pour ne pas ralentir avec la taille des tableaux
        currentIdx=varM.idxResultArray
        #print("currentIdx : ",currentIdx)
       
        #ui.tailEllipse            
        #ui.eyeEllipses[roiIndex]
        dataArray1 = np.asarray(roisEllipseEye[0].angleList[currentIdx-400: currentIdx])
        #print(dataArray[-1])
        #pen=pg.mkPen((0,255,255), width=2)
        ui.w3.plot(dataArray1,pen=ui.penCyan,clear=True) #clear=True permet de remplacer le contenu du plot par un nouveau contenu
        #autopan dans pyqtgraph recentre le graph, il n'est pas possible de voir défiler les dernières valeurs en temps réel
        #pour cela il faut recadrer le graph à chaque mise à jour
        #ui.w3.setXRange(dataArray1[-1][0]-400, dataArray1[-1][0])
        
        dataArray2 = np.asarray(roisEllipseEye[1].angleList[currentIdx-400: currentIdx])
        #pen=pg.mkPen((255,128,0), width=2)
        ui.w4.plot(dataArray2,pen=ui.penOrange,clear=True)
        #ui.w4.setXRange(dataArray2[-1][0]-400, dataArray2[-1][0])
    
        dataArray4 = np.asarray(roisEllipseEye[0].yList[currentIdx-400: currentIdx])
        #print(dataArray[-1])
        #pen=pg.mkPen((0,255,255), width=2)
        ui.w6.plot(dataArray4,pen=ui.penCyan,clear=True) #clear=True permet de remplacer le contenu du plot par un nouveau contenu
        #autopan dans pyqtgraph recentre le graph, il n'est pas possible de voir défiler les dernières valeurs en temps réel
        #pour cela il faut recadrer le graph à chaque mise à jour
        #ui.w6.setXRange(dataArray4[-1][0]-400, dataArray4[-1][0])
        
        dataArray5 = np.asarray(roisEllipseEye[1].yList[currentIdx-400: currentIdx])
        #pen=pg.mkPen((255,128,0), width=2)
        ui.w7.plot(dataArray5,pen=ui.penOrange,clear=True)
        #ui.w7.setXRange(dataArray5[-1][0]-400, dataArray5[-1][0])
        
        if len(ui.tailEllipse.angleList0)!=0:
            dataArray3 = np.asarray(ui.tailEllipse.angleList0[currentIdx-400: currentIdx])
            #pen=pg.mkPen((0,255,0), width=2)
            ui.w5.plot(dataArray3,pen=ui.penGreen,clear=True)
            #ui.w5.setXRange(dataArray3[-1][0]-400, dataArray3[-1][0])
        
        #app.processEvents()  ## force complete redraw for every plot
        
        QtCore.QTimer.singleShot(50, lambda: updatePlot())
    
def updatePlot_Full() :
    
    if ui.track_checkBox.isChecked()==False:
        if len(roisEllipseEye)>0 :
               
            dataArray1 = np.asarray(roisEllipseEye[0].angleList)
            #print(dataArray[-1])
            #pen=pg.mkPen((0,255,255), width=2)
            ui.w3.plot(dataArray1,pen=ui.penCyan,clear=True) #clear=True permet de remplacer le contenu du plot par un nouveau contenu
            #autopan dans pyqtgraph recentre le graph, il n'est pas possible de voir défiler les dernières valeurs en temps réel
            #pour cela il faut recadrer le graph à chaque mise à jour
            #ui.w3.setXRange(dataArray1[-1][0]-400, dataArray1[-1][0])
            
            dataArray2 = np.asarray(roisEllipseEye[1].angleList)
            #pen=pg.mkPen((255,128,0), width=2)
            ui.w4.plot(dataArray2,pen=ui.penOrange,clear=True)
            #ui.w4.setXRange(dataArray2[-1][0]-400, dataArray2[-1][0])
        
            dataArray4 = np.asarray(roisEllipseEye[0].yList)
            #print(dataArray[-1])
            #pen=pg.mkPen((0,255,255), width=2)
            ui.w6.plot(dataArray4,pen=ui.penCyan,clear=True) #clear=True permet de remplacer le contenu du plot par un nouveau contenu
            #autopan dans pyqtgraph recentre le graph, il n'est pas possible de voir défiler les dernières valeurs en temps réel
            #pour cela il faut recadrer le graph à chaque mise à jour
            #ui.w6.setXRange(dataArray4[-1][0]-400, dataArray4[-1][0])
            
            dataArray5 = np.asarray(roisEllipseEye[1].yList)
            #pen=pg.mkPen((255,128,0), width=2)
            ui.w7.plot(dataArray5,pen=ui.penOrange,clear=True)
            #ui.w7.setXRange(dataArray5[-1][0]-400, dataArray5[-1][0])
            
            if len(ui.tailEllipse.angleList0)!=0:
                dataArray3 = np.asarray(ui.tailEllipse.angleList0)
                #pen=pg.mkPen((0,255,0), width=2)
                ui.w5.plot(dataArray3,pen=ui.penGreen,clear=True)
                #ui.w5.setXRange(dataArray3[-1][0]-400, dataArray3[-1][0])
            
        
    

def update_viewBox():
    
    tFrame=convert_imageToPyqtgraph(image.forAnalysis)
    ui.img.setImage(tFrame,autoLevels=False)
             
    #mise à jour de l'affichage des tracés sur l'image
    mark.data['pos'][2] = [varM.ctailX[0],varM.ctailY[0]]
    mark.updateGraph()
    #définir une roi centrée sur le marqueur "mark" pour bénéficier du signal de changement de position de la roi pour tracer le graphe
    #roi définie par rapport à son coin bas gauche
    #ui.tailEllipse.stateChanged()
    
    #ui.eyeEllipses[0].setState(ui.eyeEllipses[0].savedState,update=True)
    #TODO : si un oeil n'est pas détecté aucune ellipse n'est créée ce qui génère une erreur à cause d'un manque de valeurs
    #dans roiEllipseEye.descriptor
    for index,roiEllipseEye in enumerate(roisEllipseEye):
        #roiEllipseEye.stateChanged()
        x,y,MA,ma,angle,vx,vy,xrot,yrot=roiEllipseEye.descriptor
        roiEllipseEye.setPos((x,y),update=False)
        roiEllipseEye.setSize((MA, ma),update=False)
        roiEllipseEye.setAngle(angle,update=False)
        roiEllipseEye.translate(-vx,-vy,update=False)
        roiEllipseEye.stateChanged()
        #position ellipse angle :
          
        
        ui.eyeAxeLines[index].setPos((x-vx+xrot,y-vy+yrot))
        ui.eyeAxeLines[index].setAngle(angle+90)
           
    for roiEye in roisEye:
        roiEye.stateChanged()
    #print("view_box updated")
    
        
def update_analysis():
    
    ui.progressBuffer.setValue(video.grabber.Q.qsize()/video.grabber.Q.maxsize*100)
    if ui.liveVideo_btn.isChecked()==True :
        tLive=convert_imageToPyqtgraph(image.forAnalysis)
        #ui.img_Live.setImage(tLive,autoLevels=False)
        ui.img.setImage(tLive,autoLevels=False)
        ui.measuredLivefps_Label.setText('%0.2f fps' % varM.measuredLivefps)
        ui.lenBuffer_label.setText(str(video.grabber.Q.qsize()))
        
        if ui.track_checkBox.isChecked()==True:
            
            #image.forAnalysis=framesBuffer[-1]
            update_viewBox()
            """          
            text=str(varM.frameCount)+"/"+str(len(framesBuffer))
            ui.lenBuffer_label.setText(text)
            """
            #print("refresh analysis")
            
        #print("live")
        #app.processEvents()  ## force complete redraw for every plot
        QtCore.QTimer.singleShot(50, lambda: update_analysis())
                
     
    elif ui.playVideo_btn.isChecked()==True :
        
        #ui.measuredPlayfps_Label.setText('%0.2f fps' % varM.measuredPlayfps)
        if ui.track_checkBox.isChecked()==True:
            update_viewBox()
            app.processEvents()  ## force complete redraw for every plot
            QtCore.QTimer.singleShot(50, lambda: update_analysis())
        else:
            #tFrame=convert_imageToPyqtgraph(image.currentFrame)
            #ui.img.setImage(tFrame,autoLevels=False)
            #app.processEvents()  ## force complete redraw for every plot
            QtCore.QTimer.singleShot(40, lambda: update_analysis())
              
    else : print("stop refresh live display")

        
#--------------------------------------------------------------------------------------------------------------
#                                                                                    ANALYSE
#--------------------------------------------------------------------------------------------------------------       

#click "track" button
def check_AnalysisParameters():
    
    if ui.manipType_comboBox.currentIndex()==0:
        message="You have to select an analysis mode"
        QtWidgets.QMessageBox.warning(None,"analysis mode",str(message),QtWidgets.QMessageBox.Ok)
        ui.track_checkBox.setChecked(False)
    
    else:
        if ui.track_checkBox.isChecked()==True:
            if ui.liveVideo_btn.isChecked()==True : 
                if ui.manipType_comboBox.currentText()=="test live": test_Live()    
                else :
                    #réinitialiser les graphes
                    reset_Plot()
                    #envoyer un trigger vers le système d'enregistrement
                    if ui.trigger_checkBox.isChecked()==True:
                        trigg=trigger()
                    #initialiser le nombre d'images acquises pour l'analyse
                    varM.frameCount=0
                    #initialiser le nombre d'images analysées
                    varM.idxResultArray=0
                    #QtCore.QTimer.singleShot(50, lambda: updatePlot())
                    video.buffering=True
                    #démarrage d'un thread dédié à l'acquisition live-------------------------------------liveGrabThread 
                    #print("liveAnalysisThread")
                    #liveAnalysisThread= threading.Thread(name = 'liveAnalysisThread', target = live_Routine_Track, args = ())
                    #liveAnalysisThread.start()
                    #liveAnalysis=ImageProcessor()
                    
                    
            else :
                #analyse sur video enregistrée
                #si le bouton play n'est pas activé : l'activer
                if ui.playVideo_btn.isChecked()==False :
                    ui.playVideo_btn.setChecked(True)
                    video.playing=True
                    reset_Plot()
                    #initialiser le nombre d'images analysées
                    varM.idxResultArray=0
                    #QtCore.QTimer.singleShot(50, lambda: updatePlot())
                    
                    playpause_Video()
                    #print("play analysis ok")
                    #analysisVideoFile=VideoPlayer()
                    
                #else:
                    
            
            #démarrer un timer qui raffraichit l'image analysée et les tracé régulièrement pour ne pas trop occuper l'interface
            #!!! déjà démarré avec le grabvideo pour rafraichir l'affichage du live (dans le meme qtimer)
            #QtCore.QTimer.singleShot(50, lambda: updateAnalysis())
            #attendre que les tableaux de mesure se remplissent avant de les afficher
            time.sleep(0.1)
            print("update plot")
            QtCore.QTimer.singleShot(0, lambda: updatePlot())    
        
        else : #STOP TRACKING-----------------------------------------------------------------------------STOP TRACKING
            if ui.liveVideo_btn.isChecked()==True :
                #print(ui.track_checkBox.isChecked())
                video.buffering=False
                print("stop buffer appending")
            elif ui.playVideo_btn.isChecked()==True :
                ui.playVideo_btn.setChecked(False)
                video.playing=False
                video.grabber.stop()
                
            updatePlot_Full()
        #vérification de la présence d'un axe du corps pour la mesure de la variation des axes des yeux
        """
        if len(mark.data)==0:
            message="You have to define a body axis"
            QtWidgets.QMessageBox.warning(None,"analysis mode",str(message),QtWidgets.QMessageBox.Ok)
            ui.track_checkBox.setChecked(False)
        """
        
        
        


    
def test_Live():
    if ui.track_checkBox.isChecked()==True:
        print("test_Live")
        pen=pg.mkPen((0,255,255), width=2)
        #selected = roi.getArrayRegion(data, img)
        dataTest = np.random.normal(size=(1, 600, 600), loc=1024, scale=64).astype(np.uint8)
        dataArray = np.random.normal(size=(50,500))
        curve1 = ui.w3.plot(pen='r')
        curve2 = ui.w4.plot(pen='b')
        curve3 = ui.w5.plot(pen='y')
        curve4 = ui.w6.plot(pen='g')
        curve5 = ui.w7.plot(pen='y')
        #for i in range(10000000):
        ui.img.setImage(dataTest[0])
        ptr = np.random.randint(0,10)
        curve1.setData(dataArray[ptr%10])
        curve2.setData(dataArray[ptr%10])
        curve3.setData(dataArray[ptr%10])
        curve4.setData(dataArray[ptr%10])
        curve5.setData(dataArray[ptr%10])
        """
        ui.w3.plot(dataArray,pen=pen,clear=True)
        ui.w4.plot(dataArray,pen=pen,clear=True)
        ui.w5.plot(dataArray,pen=pen,clear=True)
        ui.w6.plot(dataArray,pen=pen,clear=True)
        ui.w7.plot(dataArray,pen=pen,clear=True)
        """
        app.processEvents()  ## force complete redraw for every plot
        QtCore.QTimer.singleShot(1, lambda: test_Live())
    
        
def complete_skelet():
   
    print("complete ----------------")
    #try:
    #image.forAnalysis=image.currentGrayFrame
    #récupération des valeurs des marqueurs
    #try ne récupère pas l'erreur générée par l'abscence de marqueur de queue
    
    if len(mark.data)>0:
        tailRoot=mark.data['pos'][0]
        nose=mark.data['pos'][1]
        tail=mark.data['pos'][2]
        #Définition de l'axe du corps -----------------------------------------------------------------------------
        #print("tracé de l'axe du corps entre les points nose et tailRoot")
        #v vecteur eye1-eye2
        noseX=nose[0]
        noseY=nose[1]
        rootX=tailRoot[0]
        rootY=tailRoot[1]
        tailX=tail[0]
        tailY=tail[1]
        #définition de l'ordonnée de l'axe du corps pour identidier les yeux (au dessus/dessous)
        varM.bodyAxis_Y=noseY
        #détermination des coordonnées de 2 points pour tracer la ligne de l'axe du corps
        #détermination du vecteur défini par l'axe du corps nez-queue
        xv= noseX-rootX#-noseX
        yv= noseY-rootY#-noseY
        #calcul de l'angle par rapport à l'horizontale
        #cosθ = (u1 • v1 + u2 • v2) / (√(u12 • u22) • √(v12 • v22)).
        varM.bodyAngle = math.atan2(yv, xv)* 180 / math.pi
        """
        #normalisation du vecteur
        mag=math.sqrt(xv*xv+yv*yv)
        xv=xv/mag
        yv=yv/mag
        #détermination de l'équation de l'axe du corps y=ax+b
        #coef directeur de l'axe du corps
        axeCorps.a=yv/xv
        #ordonnée à l'origine de l'axe du corps
        axeCorps.b=noseY-axeCorps.a*noseX
        """
        print("angle de l'axe du corps : ",varM.bodyAngle)#POSITIF EN DESSOUS DE L'HORIZONTALE
        #tracé de l'axe du corps
        linePen=pg.mkPen(color='y', width=2)
        #axeLine = pg.InfiniteLine(pen=linePen,angle=varM.bodyAngle, movable=False)
        #ui.bodyAxeLine.setPos((rootX,rootY))
        #ui.bodyAxeLine.setAngle(varM.bodyAngle)
        #ui.bodyAxeLine.setPen(linePen)
        #ui.plotView.addItem(ui.bodyAxeLine, ignoreBounds=True)
        
        #positionnement par défaut de la region pour le segment de queue
        #création des régions supplémentaires
        ui.add_tailRegion()
        
        for region in ui.listTailSegments :
            region.setBounds([0,rootX])  #déplacé dans update_TailRegion_Position()
        ui.regionlr.setRegion([tailX-10,tailX+10])
#        ui.listTailSegments[0].setRegion([tailX-10,tailX+10]) 
        print("coordonnées : ",rootX, tailX)    


        #redéfinition des limites des régions de la liste
        update_tailRegionPosition()
            
        #détection de la queue
        target.region=ui.regionlr.getRegion()
        tail_Track(0,image.forAnalysis)
#        if len(ui.listTailSegments)>0:
#            for i,segment in enumerate(ui.listTailSegments) :
#                target.region=segment.getRegion()
#                tail_Track(0,image.forAnalysis)    
        print("init angle tail : ",ui.tailEllipse.angleList[-1])
    
        
        #-------------------------------------------------------------------------------------------------------
        
        ##Détection des yeux et positionnement des contours de part et d'autre de l'axe du corps----------------------
        ##mise à zéro des listes de coordonnées des yeux et de la queue
        #init_Target()
        ##tester la présence de roi pour les yeux
        if len(roisEye)>0 :
            for roiIndex,roiEye in enumerate(roisEye):
                #roiEye.stateChanged()
                xroi,yroi=roiEye.pos()
                wroi,hroi=roiEye.size()
                #-----------------------------------------------------------------------------------------
                threshEye=[ui.threshEye1_slider.value(),ui.threshEye2_slider.value()]
                #recherche du contour de l'oeil pour définir une ellipse
                cnt=eye_segmentation(roiIndex,roiEye,threshEye,image.forAnalysis) #*************************************************
                
                if cnt : 
                    ellipse_pos,anglep=eye_Rotation(roiIndex,roiEye,cnt)#********************************************************
                    
                    #tracé de l'axe de l'oeil
                    #oeil1:
                    if roiIndex==0 : 
                        linePen=pg.mkPen(color='c', width=2)
                    elif roiIndex==1 :
                        linePen=pg.mkPen(color=(255,128,0), width=2)
                    #ajouter une ligne infinie pour donner l'axe de loeil
                    #eyeAxeLine = pg.InfiniteLine(pos=[xp-vx+xrot,yp-vy+yrot],pen=linePen,angle=angle,movable=False)
                    ui.eyeAxeLines[roiIndex].setPos((ellipse_pos[0],ellipse_pos[1]))
                    ui.eyeAxeLines[roiIndex].setAngle(-anglep+90)
                    ui.eyeAxeLines[roiIndex].setPen(linePen)
                    ui.plotView.addItem(ui.eyeAxeLines[roiIndex], ignoreBounds=True)
                                        
                    x,y,MA,ma,angle,vx,vy,xrot,yrot=roisEllipseEye[roiIndex].descriptor
                    roisEllipseEye[roiIndex].setPos((x,y),update=False)
                    roisEllipseEye[roiIndex].setSize((MA, ma),update=False)
                    roisEllipseEye[roiIndex].setAngle(angle,update=False)
                    roisEllipseEye[roiIndex].translate(-vx,-vy,update=False)
                    roisEllipseEye[roiIndex].stateChanged()
                    
                    #positionner et dimensionner les ellipses pour figurer l'axe de l'oeil-direction du regard
                    roisEllipseEyeAngle[roiIndex].setPos((x,y),update=False)
                    roisEllipseEyeAngle[roiIndex].setSize((MA, 2),update=False)
                    roisEllipseEyeAngle[roiIndex].setAngle(angle,update=False)
                    roisEllipseEyeAngle[roiIndex].translate(-vx,-vy,update=False)
                    roisEllipseEyeAngle[roiIndex].stateChanged()
                    
                    ui.eyeAxeLines[roiIndex].setPos((x-vx+xrot,y-vy+yrot))
                    ui.eyeAxeLines[roiIndex].setAngle(angle+90)
                    
                    #correction de l'angle des yeux en fonction de l'axe du corps
                    #redéfinir l'angle de l'oeil "haut" pour avoir une valeur symétrique à l'oeil bas
                    
                    if yroi>varM.bodyAxis_Y: #oeil au dessus de l'axe du corps
                        angleCorr=-anglep+90-varM.bodyAngle
                    else:
                        angleCorr=anglep+90+varM.bodyAngle
                                                   
                    #-----------------------------------------------------------------------------------------
                    print("init angle eye",roiIndex," : ",angleCorr)
                #si aucun contour n'est trouvé
                else :
                    print("eye detection failed, change threshold value")
                    
        #tail_Track()
            
        """
        #calcul des coordonnées des petits axes des ellipses pour les tracer (direction regard)
        #eye1 haut
        dx1,dy1=int(math.cos(math.radians(eye1angle))*eye1.gdAxe/2),int(math.sin(math.radians(eye1angle))*eye1.gdAxe/2)
        #eye2 bas
        dx2,dy2=int(math.cos(math.radians(eye2angle))*eye2.gdAxe/2),int(math.sin(math.radians(eye2angle))*eye2.gdAxe/2)
        """
    else :
        msg="no reference for body axe. You can add one with 'tail-root'"
        QtWidgets.QMessageBox.warning(None,"warning",str(msg),QtWidgets.QMessageBox.Ok)
            
#    except ValueError as e :
#        QtWidgets.QMessageBox.warning(None,"Error",str(ValueError),QtWidgets.QMessageBox.Ok) 
               


def tail_Track(iframe,analysimg):
    #print("tail_Track",iframe)
    #récupération des coordonnées de la région d'analyse de la queue
#    rgn=ui.regionlr.getRegion()
    #TODO : récupération de la région circulaire définie par la roi annulaire
#    rgn=target.region

#    xroicv=int(rgn[0])
#    yroicv=0
#    wroi=int(rgn[1])-int(rgn[0])
#    hroi=int(video.height)
#    cropImgData=analysimg[yroicv:yroicv+hroi,xroicv:xroicv+wroi]
#    cv2.imshow("crop",cropImgData)
    
    #créer les listes de coordonnées en fonction du nombre de région choisi par l'utilisateur
    varM.ctailX=[0]*ui.tailSegNumber_spinBox.value()
    varM.ctailY=[0]*ui.tailSegNumber_spinBox.value()
    tailAngle=[0]*ui.tailSegNumber_spinBox.value()
    tailAngleCorr=[0]*ui.tailSegNumber_spinBox.value()
    
    #détection de la cible par seuillage et contour
    #--------------------
    threshValue_Tail=ui.threshTail_slider.value()
    
    #récupération des coordonnées de la région d'analyse de la queue
    #rgn=ui.regionlr.getRegion()
    #TODO : récupération de la région circulaire définie par la roi annulaire
    for n,rgn in enumerate(ui.listTailSegments) :
        xroicv=int(rgn.getRegion()[0])
        #print("xroicv : ",xroicv)
        yroicv=0
        wroi=int(rgn.getRegion()[1]-rgn.getRegion()[0])
        hroi=int(video.height)
        #print("i,w,h = ",i, wroi,hroi)
        cropImgData=analysimg[yroicv:yroicv+hroi,xroicv:xroicv+wroi]
#        cv2.imshow("crop",cropImgData)
        #détection de la cible par seuillage et contour
        #--------------------
        
#        threshTailZone = cv2.threshold(cropImgData, threshValue_Tail, 255, cv2.THRESH_BINARY_INV)[1]
        #TODO : inverse thresh tétard sur fond noir
        if ui.whiteBgd_radioButton.isChecked():
            threshTailZone = cv2.threshold(cropImgData, threshValue_Tail, 255, cv2.THRESH_BINARY_INV)[1]    
        else :
            threshTailZone = cv2.threshold(cropImgData, threshValue_Tail, 255, cv2.THRESH_BINARY)[1]
        threshTailZone = cv2.dilate(threshTailZone, None, iterations=5)
        #cv2.imshow("crop",threshTailZone)
        #kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
        #thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        #thresh = cv2.convertScaleAbs(thresh)       
        (tailcnts, _) = cv2.findContours(threshTailZone.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
        #classement des contours par taille décroissante, ne conserver que les 2 premiers
             
        if tailcnts :
            #cv2.drawContours(cropImgData,tailcnts,-1,255,2)
            #cv2.imshow("crop",cropImgData)                  
            ctail = sorted(tailcnts, key = cv2.contourArea, reverse = True)[:1]
            #cv2.drawContours(tailZone,ctail,0,150,2)
            #calcul du centre du contour retenu dans la region analysée       
            M = cv2.moments(ctail[0])
            tailX = int(M["m10"] / M["m00"])
            tailY = int(M["m01"] / M["m00"])
            
            varM.ctailX[n]=tailX+xroicv
            varM.ctailY[n]=int(video.height)-tailY
              
            #cv2.drawContours(frame,[contour],0,255,2)
            #repositionnement du marqueur de queue en fonction de la zone détectée
            #mark est défini par rapport à son centre
            #mark.data['pos'][2] = [varM.ctailX,varM.ctailY]
            #mark.updateGraph()
            #----------------------------------------------------------------------------
            
            """
            #définir une roi centrée sur le marqueur "mark" pour bénéficier du signal de changement de position de la roi pour tracer le graphe
            #roi définie par rapport à son coin bas gauche
            
            #définir la taille de l'ellipse en fonction de celle du marqueur
            roiSize=mark.size*1.5
            ui.tailEllipse.setSize((roiSize, roiSize),update=False, finish=False)
            #roiSize=ui.tailEllipse.size()
            ui.tailEllipse.setPos((varM.ctailX-roiSize/2,varM.ctailY-roiSize/2),y=None, update=False, finish=False)
            #ui.tailCircle.setPos([ctailX,ctailY], update=True, finish=True)
            """
            #--------------------------------------------------------------------------------------
            
            #calcul de l'angle de la queue par rapport à l'horizontale
            #positif en dessous, négatif au dessus
            #récupération de la position du marqueur de la racine de la queue
            rootx,rooty=mark.data['pos'][0]
            #tailAngle=(math.atan((rooty-varM.ctailY)/(rootx-varM.ctailX))) * 180 / math.pi
#            #tailAngle=(math.atan2((varM.ctailY-rooty),(varM.ctailX-rootx))) * 180 / math.pi
#            #mesure de l'angle de la queue par rapport à l'horizontale coté tête
#            #prendre l'opposé pour la valeur côté queue 
#            tailAngle=((math.atan2((rooty-varM.ctailY),(rootx-varM.ctailX))) * 180 / math.pi)*(-1)
#            #print("tailAngle H : ",tailAngle)
#            #tailAngle2=((math.atan2((varM.ctailY-rooty),(varM.ctailX-rootx))) * 180 / math.pi)
#            #print("tailAngle H2 : ",tailAngle2)
#            #correction de l'angle par rapport à l'axe du corps (positif lorsque la tête est à droite et au dessus de l'horizontale (yroot))
#            #tailAngleCorr=tailAngle-varM.bodyAngle
#            tailAngleCorr=tailAngle+varM.bodyAngle
#            #print("tail angle : ",tailAngleCorr)
            if n==0 :
                tailAngle[n]=((math.atan2((rooty-varM.ctailY[0]),(rootx-varM.ctailX[0]))) * 180 / math.pi)*(-1)
                #correction de l'angle par rapport à l'axe du corps (positif lorsque la tête est à droite et au dessus de l'horizontale (yroot))
                #tailAngleCorr=tailAngle-varM.bodyAngle
                tailAngleCorr[n]=tailAngle[n]+varM.bodyAngle
                print("corrected tail first angle: ",tailAngleCorr[n])
            else :
                tailAngle[n]=((math.atan2((varM.ctailY[n-1]-varM.ctailY[n]),(varM.ctailX[n-1]-varM.ctailX[n]))) * 180 / math.pi)*(-1)
                tailAngleCorr[n]=tailAngle[n]-tailAngle[n-1]
                print("tail angle : ",tailAngleCorr[n])
            """
            #calcul de l'angle entre l'axe du corps défini par le segment nose-root et l'axe de la queue défini par le segment root-tail
            nosex,nosey=mark.data['pos'][1]
            #produit scalaire:u.v=xu*xv+yu*yv
            #cos(angle)=u.v/|u|.|v|
            v_noseX=rootx-nosex
            v_noseY=rooty-nosey
            v_tailX=rootx-ctailX
            v_tailY=rooty-ctailY
            #print(v_noseX, v_noseY, v_tailX, v_tailY)
            scal=v_noseX*v_tailX+v_noseY*v_tailY
            print("scal : ",scal)
            norm=math.sqrt(v_noseX*v_noseX+v_noseY*v_noseY)*math.sqrt(v_tailX*v_tailX+v_tailY*v_tailY)
            print("norm : ",norm)
            tailAngleR=(math.acos(scal/norm))* 180 / math.pi
            #!!!la valeur du cos est toujours positive lorsque la queue passe au dessus/dessous la verticale !
            print("tailAngleR : ",tailAngleR)
            """
            #ui.tailEllipse.angleList.append([iframe,tailAngleCorr[n]])
            #ui.tailEllipse.angleList[iframe]=tailAngleCorr
            
            
       
        else :
            #si la queue n'est pas détectée prendre la valeur précédente... plutot 0
            print("tail detection failed")
            tailAngleCorr[n]=180#ui.tailEllipse.angleList[-1][1]
            #print("tailAngle : ",tailAngle)
            #ui.tailEllipse.angleList.append([ui.timeLine_slider.value(),tailAngle])
            #ui.tailEllipse.angleList.append([iframe,tailAngle])
            #solution 2 : prendre une valeur aberrante : 180 degrés
        
    #création d'une liste contenant tous les angles des différents segments
    angleListNow=[iframe]
    for n in range (len(ui.listTailSegments)):
        angleListNow.append(tailAngleCorr[n])
    ui.tailEllipse.angleList.append(angleListNow) 
    ui.tailEllipse.angleList0.append([iframe,tailAngleCorr[0]])
    #tracer les repères des centroid pour chaque zone
    #ui.plotView.clear()
    xplot = np.asarray(varM.ctailX)
    yplot = np.asarray(varM.ctailY)
    #plot.plot([0, 1, 2, 3, 4], pen=(0,0,200), symbolBrush=(0,0,200), symbolPen='w', symbol='o', symbolSize=14, name="symbol='o'")
    #ui.plotView.plot(x=xplot, y=yplot, symbol='o')
    #p.repaint()    
    ui.curveTail.setData(x=xplot, y=yplot, symbolPen='b', symbol='+')    
    app.processEvents()  ## force complete redraw for every plot    






#tracking des yeux-----------------------------------------------------------------------------------------------------EYES TRACKING    
    
def eye_Track(iframe,analysimg):
    #print("eye_Track",iframe)
    threshEye=[ui.threshEye1_slider.value(),ui.threshEye2_slider.value()]
    for roiIndex,roiEye in enumerate(roisEye):
        #print(roiIndex)
        #roiEye.stateChanged()
        xroi,yroi=roiEye.pos()
        wroi,hroi=roiEye.size()
        #extraction du contour des yeux
        cnt=eye_segmentation(roiIndex,roiEye,threshEye,analysimg)#------------------------------SEGMENTATION
        #print("segmentation ok")
        if cnt : 
            ellipse_pos,anglep=eye_Rotation(roiIndex,roiEye,cnt)#-------------------------------ROTATION
            #print("eye_rotation ok")
            #tracé de l'axe de l'oeil
            ui.eyeAxeLines[roiIndex].setPos((ellipse_pos[0],ellipse_pos[1]))
            ui.eyeAxeLines[roiIndex].setAngle(-anglep+90)
            
        else :
            #print("eye detection failed")
            anglep=0
            ellipse_pos=[0,0]
            
        
        #définition de la valeur de l'angle en fonction de la position de l'oeil par raport à l'axe du corps    
        if yroi>varM.bodyAxis_Y: #oeil au dessus de l'axe du corps
            angleCorr=-anglep+90-varM.bodyAngle
            #print("oeil haut : ",angleCorr)
        else:
            angleCorr=anglep+90+varM.bodyAngle
            #print("oeil bas: ",angleCorr)
                                             
        roisEllipseEye[roiIndex].yList.append([iframe,ellipse_pos[1]])
        roisEllipseEye[roiIndex].angleList.append([iframe,angleCorr])
        #roisEllipseEye[roiIndex].angleList[iframe]=angleCorr
        #print("eye track",roiIndex,":",iframe)            
     
#@jit   
def eye_segmentation(roiIndex,roiEye,threshEye,analysimg):
    #print("eye_segmentation")    
    #définir la partie de l'image concernée par la roi
    xroi,yroi=roiEye.pos()
    wroi,hroi=roiEye.size()
    #print(xroi,yroi,wroi,hroi)     
    #TODO : vérifier que les coordonnées de la roi ne soient pas hors de l'image
    #recaler les coordonnées de crop pour ne pas croper en dehors de l'image
    roiXc,roiYc,wroic,hroic=testRoiInImageview(xroi,yroi,wroi,hroi)
    #print(roiXc,roiYc,wroic,hroic)
    image.crop=[roiXc,roiYc,wroic,hroic]
    #xroicv,yroicv,wroi,hroi=int(xroi),int(video.height-yroi-hroi),int(wroi),int(hroi) #pour y les coordonnées sont inversées dans cv2
    xroicv,yroicv,wroicv,hroicv=int(roiXc),int(video.height-roiYc-hroic),int(wroic),int(hroic)
    #print("position roicv : ",xroicv,yroicv,wroi,hroi)
    #imgData=ui.img.image[x:x+w,y:y+h]
    #cv2.imshow("roi_definition",imgData)
    #roi.getArrayRegion(imgData,ui.img)
    
    #segmentation des yeux dans chaque roi
    cropImgData=analysimg[yroicv:yroicv+hroicv,xroicv:xroicv+wroicv]
    #try:
    blur = cv2.GaussianBlur(cropImgData, (5, 5), 0)
    
    threshed_Eye = cv2.threshold(blur, threshEye[roiIndex], 255, cv2.THRESH_BINARY_INV)[1]
    #thresh_Img1 = cv2.threshold(gray[0:noseY,:], threshEye1_Value, 255, cv2.THRESH_BINARY_INV)[1]
    #thresh_Img2 = cv2.threshold(gray[noseY:,:], threshEye2_Value, 255, cv2.THRESH_BINARY_INV)[1]
    #thresh = cv2.erode(thresh, None, iterations=2)
    #cv2.imshow("thresh",threshed_Eye)
    
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
    threshed_Eye = cv2.morphologyEx(threshed_Eye, cv2.MORPH_OPEN, kernel)
    
        
    #TODO : ajuster la taille de la matrice à la taille de la roi
    #TODO : fillholes pour éliminer les reflets sur l'oeil    
    
    #except cv2.error as e:
    #    print(e)
    #    QtWidgets.QMessageBox.warning(None,"Tracking",str(e),QtWidgets.QMessageBox.Ok)
    
    #définir le contour du globe de l'oeil sans la pupille        
    (cnts, _) = cv2.findContours(threshed_Eye.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
    
    if len(cnts)>0:
        #classement des contours par taille décroissante, ne conserver que le premier
        cnt = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]
        
        return cnt
         
    else :
        print("no eye contour")



    
#@jit    
def eye_Rotation(roiIndex,roiEye,cnt):
    #print("eye_Rotation")
    for contour in cnt :
        #cv2.drawContours(frame,[contour],0,255,2)
        #approx = cv2.approxPolyDP(contour,0.05*cv2.arcLength(contour,True),True)
        #rect=cv2.boundingRect(contour)
       
        hull = cv2.convexHull(contour)
        if len(hull)>4 :   #il faut au moins 5 points pour définir une ellipse estimée à partir de l'enveloppe
            ellipse= cv2.fitEllipse(hull)
            (xe,ye),(ma,MA),angle =ellipse
            #print(xe,ye,ma,MA)
        else : #la valeur de seuillage est trop élevée (sélection de la roi complète : contour = 4 points (rectangle))
                #ou aucun objet détecté à cause d'un seuil trop faible
            print("eye detection failed (rotate)")
            print(roiIndex)
            print(len(roisEllipseEye))
            print(roisEllipseEye[roiIndex].yList[-1])
            ellipse_pos=roisEllipseEye[roiIndex].yList[-1]#[0,0]
            print(ellipse_pos)
            return(ellipse_pos,0)
            break
        
        #ellipseH=(x,y),(ma,MA),angle
        #cv2.ellipse(currentFrame,ellipseH,(0,0,255),1)            
        
        """
        pour l'ellipse l'angle est calculé par raport à la verticale dans le sens horaire 
        à partir de l'origine du rectangle minimum
        angle=angle-90    
        """
        anglep=angle-90
        """
        si l'angle est positif dans le sens horaire par raport à la verticale, 
        la transposition par rapport à l'horizontale est angle = 90-angle
        cela correspond à l'angle du grand axe de l'ellipse inscrite dans le rectangle
        pour avoir l'angle du petit axe il faut ajouter 90 : anglep=180-angle
              
        if angle>180 : angle=180-angle
        print("angle ellipse : ", angle)
        """
              
        #xe,ye sont les coordonnées du centre de l'ellipse dans le crop (=roiEye) de l'image analysée dans cv2
        #positionner l'ellipse dans pyqtgraph par raport à son centre
        #calculer la position de son origine avant rotation : xp,yp
        xroi,yroi=roiEye.pos()
        wroi,hroi=roiEye.size()
        xroi,yroi,wroi,hroi=image.crop
        xp,yp=int(xroi+xe-MA/2),int(yroi+hroi-ye-ma/2)
        #print("xp,yp :",xp,yp )
              
        """           
        la rotation de la roi se fait autour de l'origine dans pyqtgraph
        pour conserver le positionnement obtenu dans opencv il faut
        calculer le vecteur de translation appliqué au centre pendant la rotation
        et déplacer la roi selon le vecteur opposé
        """
        #calcul des nouvelles coordonnées du centre après une rotation de "angle"
        #avec comme centre l'origine
        xc=MA/2
        yc=ma/2
        radangle=anglep/180*math.pi
        xrot=xc*math.cos(radangle) + yc*math.sin(radangle)
        yrot=xc*(-math.sin(radangle))+yc*math.cos(radangle)
        #print("coord : ",xc,yc,xrot,yrot)
        vx=xrot-xc
        vy=yrot-yc
        """
        #translation de la roi selon ce vecteur
        #après rotation car la rotation n'est pas centrée
        roisEllipseEye[roiIndex].translate([-vx,-vy],update=False)
        roisEllipseEye[roiIndex].stateChanged()
        """
        roisEllipseEye[roiIndex].descriptor=[xp,yp,MA,ma,-anglep,vx,vy,xrot,yrot]
        ellipse_pos=[xp-vx+xrot,yp-vy+yrot]

        #ui.eyeEllipses[roiIndex].savedState=ui.eyeEllipses[roiIndex].saveState()
    
        #recaler la roieye centrée sur le centre de l'ellipse
        newroiX,newroiY=int(xroi+xe-wroi/2),int(yroi+hroi-ye-hroi/2)
       
        
        roiEye.setPos([newroiX,newroiY], update=False)
        #roiEye.stateChanged()
        
        #TODO: ellipse[-1].setParentItem(roiseye[i])
        
        ##cv2.drawContours(image.currentimage,[hull],0,(255,0,255),2)
        #cv2.imshow("ellipse",currentFrame)
        return(ellipse_pos,anglep)            
    

def testRoiInImageview(x,y,w,h):
    if x<0 :
        w=w+x
        x=0
    elif x+w > video.width:
        diff=x+w-video.width
        w=w-diff
    if y<0 :
        h=h+y
        y=0
    elif y+h > video.height:
        diff=y+h-video.height
        h=h-diff
    return(x,y,w,h)                


def init_track():
    """
    # Set up tracker.
    # Instead of MIL, you can also use
    # BOOSTING, KCF, TLD, MEDIANFLOW or GOTURN
    """
    #liste des roi de tracking
    bboxes=[]
    if video.nbFrames!=None:
        currentFrame=video.capture.get(1)-1
        retVal=video.capture.set(1,currentFrame)
        
        (ret, frame) = video.capture.read() 
        #target.tracker = cv2.Tracker_create("MIL")
        #target.tracker =cv2.MultiTracker("KFC")#bug opencv3.1 python
        #target.tracker =cv2.MultiTracker("TLD")
        target.tracker =cv2.MultiTracker("MIL")
        """
        # methode BOOSTING : fonctionne très bien pour une souris entière
        #---si la zone est trop petite(?) python cesse de fonctionner
        # methode MIL : plus efficace sur les petites zones contrastées
        """
        if len(roisLimb)>0:
            #ind = pts[0].data()[0]
            #self.dragOffset = self.data['pos'][ind]
            #print(rois[0].data['pos'][0])
            for roi in roisLimb :
                print(roi.size())
                print(roi.pos())
                x,y=roi.pos()
                w,h=roi.size()
                # Define an initial bounding box
                #bbox = (x, y, w, h)
                # l'origine des coordonnées est différente entre opencv et pyqt
                #en haut à gauche vs en bas à gauche
                ycv2=video.height-y-h
                bbox=(x, ycv2, w, h)
                bboxes.append(bbox)
                # Uncomment the line below to select a different bounding box
                # bbox = cv2.selectROI(frame, False)
                # Initialize tracker with first frame and bounding box
        ok = target.tracker.add(frame,(bboxes))
        #ok = target.tracker.add(frame,(bboxes[0],bboxes[1]))
        #ok = target.tracker.init(frame, bboxes)
  
        print("tracker : ",ok)

           
def limb_Track(iframe):
    
    # TODO : Update tracker
    """
    Ptr<TrackerModel> model = tracker->getModel();
    Ptr<TrackerTargetState> lastTargetstate = getLastTargetState();
    
    // Make changes to lastTargetState (update position etc)
    
    // Set lastTargetState, I am not sure if you need to actually set it
    // or just editing the object through pointer should work.
    model->setLastTargetState(lastTargetstate);
    modelUpdate()
    """
    """
    if len(varM.var)==0 : varM.var=[10,10,500,200]
    #TODO : définir une zone de recherche plus petite que l'image entière pour accélérer le traitement
    xb,yb,wb,hb=varM.var#[10,10,100,80]
    print(xb)
    xs=xb-2*wb
    if xs<0 : xs=0
    ys=yb-2*hb
    if ys<0 : ys=0
    
    imageSearch=image.currentGrayFrame[xs:xb+3*wb,ys:yb+3*hb].copy()
    cv2.imshow("Tracking", imageSearch)
    
    ok, bboxes = target.tracker.update(imageSearch)
    """
    ok, bboxes = target.tracker.update(image.forAnalysis)
    #TODO : fournir en argument l'image définie avec le masque annulaire
    # Draw bounding box
    #print(target.tracker.getTargetPosition())
    if ok:
        """
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(image, p1, p2, (0,0,255))
        # Display result
        cv2.imshow("Tracking", image)
        """
        #for bbox in bboxes:
        for b in range(len(bboxes)):
            x,ycv2,w,h=int(bboxes[b][0]),int(bboxes[b][1]),int(bboxes[b][2]),int(bboxes[b][3])
            
            varM.var=[x,ycv2,w,h]
            #print(x,ycv2,w,h)
            y=video.height-h-ycv2
            #centre de la boite
            roisLimb[b].setPos([x,y], update=True, finish=True)
            #print(rois[0].pos())
           
            x=x+w/2
            y=y+h/2
            """
            #calcul de l'angle de la patte avec l'horizontale
            angle=calculAngle([x,y],[550,370])#--> pour cela il faut déterminer un point de repère sur l'animal
            #pour l'exemple les coordonnées sont définies en "dur"
            #xo,yo=560,370 #(réferentiel pyqtgraph)
            roisLimb[b].angleList.append([iframe,angle])
            dataArray = np.asarray(rois[b].angleList)
            ui.plotView_Step.plot(dataArray)
            """







        
        
def save_ResultsNow():
    filename,_ = QtWidgets.QFileDialog.getSaveFileName(None,"Save file", "", ".csv")
    print(filename)
    resultFile=create_Result(filename,0)
    print(resultFile)
    save_Results(resultFile)


def create_Result(filePath,phaseDuration):
    
    #TODO : test si le fichier est en cours d'utilisation
    #création d'un fichier résultat dont l'entête est spécifique à une manip
    #csvResult=csv.writer(open(filePath+"_result.csv","w"))
    try :
        csvResult=csv.writer(open(filePath+".csv","w", newline=''))
        #pour éviter les sauts de lignes non voulus
        #csvResult=csv.writer(open(filePath+".csv","w", newline="\n", encoding="utf-8"))
        #TODO :utilisation d'un context manager pour gérer la fermeture du fichier en cas d'exception
        #with csv.writer(open(dialog1.lb_VideoName.text()+"_result.csv","wb")) as csvResult :
        csvResult.writerow([video.file])
        csvResult.writerow(["framerate : "+str(video.fps)+' fps'])
        #csvResult.writerow(["sampling factor : "+str(analyze.framerateFactor)])
        csvResult.writerow(["video size : "+str(video.width)+" ; "+str(video.height)])
        csvResult.writerow([" "])
        """
        csvResult.writerow([" "])
        csvResult.writerow(["analysis duration : "+str(phaseDuration)+"sec"])
        csvResult.writerow([" "])
        """
        if ui.manipType_comboBox.currentText()=="eyes-tail track":
            csvResult.writerow(["eye1 angle","eye2 angle","tail angle","eye1 Ymove","eye2 Ymove"])
             
        elif ui.manipType_comboBox.currentText()=="eyes track only":
            csvResult.writerow(["eye1 angle","eye2 angle","eye1 Ymove","eye2 Ymove"])
         
        return csvResult
    except:
        #e = sys.exc_info()[0]
        exctype, value = sys.exc_info()[:2] #[0]:type, [1]:value, [2]:traceback
        QtWidgets.QMessageBox.information(None,"Error",str(value),QtWidgets.QMessageBox.Ok)
        return None

def save_Results(result_f):
    try :
        if ui.manipType_comboBox.currentText()=="eyes-tail track":
            for i in range(len(ui.tailEllipse.angleList)):
                #result_f.writerow([textline])
                #enregistrement des valeurs frame par frame
                result_f.writerow([roisEllipseEye[0].angleList[i][1],
                                   roisEllipseEye[1].angleList[i][1],
                                   ui.tailEllipse.angleList0[i][1],
                                   roisEllipseEye[0].yList[i][1],
                                   roisEllipseEye[1].yList[i][1]])
                    
    
        elif ui.manipType_comboBox.currentText()=="eyes track only":
            
            for i in range(len(roisEllipseEye[0].angleList)):
                #result_f.writerow([textline])
                #enregistrement des valeurs frame par frame
                result_f.writerow([roisEllipseEye[0].angleList[i][1],
                                   roisEllipseEye[1].angleList[i][1],
                                   roisEllipseEye[0].yList[i][1],
                                   roisEllipseEye[1].yList[i][1]])
                
    except:
        #e = sys.exc_info()[0]
        exctype, value = sys.exc_info()[:2] #[0]:type, [1]:value, [2]:traceback
        QtWidgets.QMessageBox.information(None,"Error",str(value),QtWidgets.QMessageBox.Ok)
        return None  
        

   
#------------------------------------------------------------------------------------------
#                                                               affichage sur l'image marqueur de position, roi
#----------------------------------------------------------------------------------------       

class graphMark(pg.GraphItem):
    def __init__(self):
        self.dragPoint = None
        self.dragOffset = None
        self.textItems = []
        pg.GraphItem.__init__(self)
        self.scatter.sigClicked.connect(self.clicked)
        #liste des données décrivants les positions précédentes des marqueurs
        self.lastConformation=[]
        self.size=0
        
    def setData(self, **kwds):
        self.text = kwds.pop('text', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = np.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = np.arange(npts)
        self.setTexts(self.text)
        self.updateGraph()
        
    def setTexts(self, text):
        for i in self.textItems:
            i.scene().removeItem(i)
        self.textItems = []
        for t in text:
            item = pg.TextItem(t,color="g")
            self.textItems.append(item)
            item.setParentItem(self)
        
    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        for i,item in enumerate(self.textItems):
            item.setPos(*self.data['pos'][i])
        
        
    def mouseDragEvent(self, ev):
        
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        
        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first 
            # pressed:
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) == 0:
                ev.ignore()
                return
            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.dragOffset = self.data['pos'][ind] - pos
        elif ev.isFinish():
            self.dragPoint = None
            return
        else:
            if self.dragPoint is None:
                ev.ignore()
                return
        
        ind = self.dragPoint.data()[0]
        self.data['pos'][ind] = ev.pos() + self.dragOffset
        self.updateGraph()
        ev.accept()
        
        
    def clicked(self, pts):
        print("clicked: %s" % pts)    

        
def init_markNoseRootTail(x0,y0):
    ## Define positions of nodes
    #si les marqueurs ont été créés
    if len(mark.data) != 0 :
        #positionnement des marqueurs relativement à la position de tailroot
        #récupérer les coordonnées des marqueurs nez et racine queue
        #print(mark.data['pos'])
        
        
        tailRoot=mark.data['pos'][0]
        nose=mark.data['pos'][1]
        tailpos=mark.data['pos'][2]
        
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
        
    ## Define the set of connections in the graph
    adj = np.array([
        [0,1],
        [0,2]
        ])
        
    ## Define the symbol to use for each node (this is optional)
    symbols = ['o','o','o']#,'t','+']
    
    ## Define symbol color rvb
    symbolBrushes=[(255,0,255),
                   (255,0,255),
                   (0,255,0)
                   ]
    
    ## Define the line style for each connection (this is optional)
    lines = np.array([
        (255,0,255,255,2),
        (0,255,0,255,2)
        ], dtype=[('red',np.ubyte),('green',np.ubyte),('blue',np.ubyte),('alpha',np.ubyte),('width',float)])
    
    ## Define text to show next to each symbol
    #texts = ["Point %d" % i for i in range(3)]
    texts=["root","nose","tail"]

    ## Update the graph
    mark.size=8
    mark.setData(pos=pos, adj=adj, pen=lines, size=mark.size, symbolBrush=symbolBrushes,symbolPen='w',symbol=symbols, pxMode=False, text=texts)
           
        
        
        

#----------------------------------------------------------------------------gestion de l'affichage de l'image dans un plot   
    

def mouseClicked(evt):
    """
    ajouter un marqueur ou une roi sur l'image
    """
    """
    #indépendamment du zoom ou du pan
    mousePoint = view.mapSceneToView(pos)
    print(mousePoint)
    """
    pos = evt[0].pos()
    #print(pos)
       
    #conversion des coordonnées en coordonnées de scene        
    if ui.plotView.sceneBoundingRect().contains(pos):
        #imc = img.mapFromScene(pos)=coordonnées dans le widget view
        #pos=img.mapToScene(imc.x(),imc.y()) 
        """
        mousePoint = ui.viewBox.mapSceneToView(pos)
        x,y = int(mousePoint.x()),int(mousePoint.y())
        """
        mousePointFromScene=ui.viewBox.mapToView(pos)
        #coordonnées dans le plot lié à l'image
        x0,y0=mousePointFromScene.x(),mousePointFromScene.y()
        #print(x0,y0)
        """#ça marche aussi quand une image est chargée
        mouseImg=ui.img.mapFromScene(pos)
        print(mouseImg.x(),mouseImg.y())
        """
        #mode tailroot--------------------------------------------------------------------------
        
        if evt[0].button() == 1 and ui.selectTailRoot_radioButton.isChecked():                   
            #définition des propriétés des marqueurs-------------------------------------------
            init_markNoseRootTail(x0,y0)
            
        #mode roi eye---------------------------------------------------------------------------------   
        
        if evt[0].button() == 1 and ui.selectEyes_radioButton.isChecked():
            #print("rectangle checked")
            #rois.append(pg.RectROI([x1-w/2, y1-h/2], [w, h], sideScalers=False, pen=(0,10),removable=True))
        
            w=100
            h=80
            
            #ajout d'une roi pour définir la zone où se situe l'oeil
            #roi sans poignée de modification
            #rois.append(pg.ROI([x1-w/2, y1-h/2], [w, h], pen=(0,10),removable=True))
            #roi modifiable
            #class pyqtgraph.RectROI(pos, size, centered=False, sideScalers=False, **args)
            #pos : 0,0 en bas à gauche
            #pen(color,width), color 0=rouge, 1=orange....
            #rois.append(pg.RectROI([x1-w/2, y1-h/2], [w, h],centered=True, pen=(0,10),removable=True))            
            roi_limits=QtCore.QRectF(0,0,video.width,video.height)
            #roisEye.append(Roi([x0-w/2, y0-h/2], [w, h],centered=True, pen=("b"),removable=True)) 
            roisEye.append(Roi([x0-w/2, y0-h/2], [w, h],maxBounds=roi_limits,centered=True, pen=("b"),removable=True)) 
            roisEye[-1].sigRemoveRequested.connect(remove_ROI)
            ui.plotView.addItem(roisEye[-1])
            
            #ajout d'une ellipse pour le contour de l'oeil
            roisEllipseEye.append(EllipseROI_centered_noHandle(pos=roisEye[-1].pos(),size=[1,1],pen=(3,5)))
            ui.plotView.addItem(roisEllipseEye[-1])
            #ajout d'une ellipse comme indicateur de l'orientation de l'oeil
            roisEllipseEyeAngle.append(EllipseROI_centered_noHandle(pos=roisEye[-1].pos(),size=[1,1],pen=(3,5)))
            ui.plotView.addItem(roisEllipseEyeAngle[-1])
        
        #mode trackLimb-----------------------------------------------------------------------------------------------------    
        if evt[0].button() == 1 and ui.selectLimbs_radioButton.isChecked():
            #print("rectangle checked")
            #rois.append(pg.RectROI([x1-w/2, y1-h/2], [w, h], sideScalers=False, pen=(0,10),removable=True))
        
            w=100
            h=80
            #roi sans poignée de modification
            #rois.append(pg.ROI([x1-w/2, y1-h/2], [w, h], pen=(0,10),removable=True))
            #roi modifiable
            #class pyqtgraph.RectROI(pos, size, centered=False, sideScalers=False, **args)
            #pos : 0,0 en bas à gauche
            #pen(color,width), color 0=rouge, 1=orange....
            #rois.append(pg.RectROI([x1-w/2, y1-h/2], [w, h],centered=True, pen=(0,10),removable=True))            
            roisLimb.append(Roi([x0-w/2, y0-h/2], [w, h],centered=True, pen=("m"),removable=True)) 
            roisLimb[-1].sigRemoveRequested.connect(remove_ROI)
            ui.plotView.addItem(roisLimb[-1])
                
            
    else:
        #evt.ignore()
        return
        
    #----------------------------------------------------------------------------------
           
    """
    mapFromItemToView(item, obj)        
        Maps obj from the local coordinate system of item to the view coordinates
    
    mapFromView(obj)        
        Maps from the coordinate system displayed inside the ViewBox to the local coordinates of the ViewBox
    
    mapFromViewToItem(item, obj)        
        Maps obj from view coordinates to the local coordinate system of item.
    
    mapSceneToView(obj)        
        Maps from scene coordinates to the coordinate system displayed inside the ViewBox
    
    mapToView(obj)        
        Maps from the local coordinates of the ViewBox to the coordinate system displayed inside the ViewBox
    
    mapViewToScene(obj)        
        Maps from the coordinate system displayed inside the ViewBox to scene coordinates

    """
           
def remove_ROI(evt):
    print("remove:",evt)
    ui.plotView.scene().removeItem(evt)           
    #retirer la zone de la liste ou refaire la liste ?
    roisEye.remove(evt)
    
    
    
    
#ROIs----------------------------------------------------------------------------------------------ROIs    
    
class EllipseROI_centered_noHandle(pg.ROI):
    """
    Elliptical ROI subclass with one scale handle and one rotation handle.
    
    
    ============== =============================================================
    **Arguments**
    pos            (length-2 sequence) The position of the ROI's origin.
    size           (length-2 sequence) The size of the ROI's bounding rectangle.
    \**args        All extra keyword arguments are passed to ROI()
    ============== =============================================================
    
    """
    def __init__(self, pos, size, **args):
        #QtGui.QGraphicsRectItem.__init__(self, 0, 0, size[0], size[1])
        pg.ROI.__init__(self, pos, size, **args)
        #super().__init__(self, pos, size, **args)
        #self.addRotateHandle([1.0, 0.5], [0.5, 0.5])
        #self.addScaleHandle([0.5*2.**-0.5 + 0.5, 0.5*2.**-0.5 + 0.5], [0.5, 0.5])
        self.angleList=[[0,0]]#np.zeros(100000,dtype=float)#
        self.angleList0=[[0,0]]
        self.yList=[[0,0]]
        self.savedState=None
        self.descriptor=[0,0,0,0,0,0,0,0,0]#[xp,yp,Ma,ma,angle,vx,vy,xrot,yrot]
            
    def paint(self, p, opt, widget):
        r = self.boundingRect()
        p.setRenderHint(QtgGui.QPainter.Antialiasing)
        p.setPen(self.currentPen)
        
        p.scale(r.width(), r.height())## workaround for GL bug
        r = QtgCore.QRectF(r.x()/r.width(), r.y()/r.height(), 1,1)
        
        p.drawEllipse(r)
        
    
    def shape(self):
        self.path = QtgGui.QPainterPath()
        self.path.addEllipse(self.boundingRect())
        return self.path
    
        
    """    
    def setAngle(self, angle, update=True, finish=True):
        #Set the angle of rotation (in degrees) for this ROI.
        #See setPos() for an explanation of the update and finish arguments.
        
        print("roipos : ",self.pos())
        print("roisize : ",self.size())
        self.state['angle'] = angle
        tr = QtgGui.QTransform()
        #--------------------------------------
        #tr.translate(posx-sizew/2,posy-sizeh/2)
        #tr.translate(sizew/2,sizeh/2)
        #--------------------------------------
        tr.rotate(angle)
        #--------------------------------------
        #tr.translate(-r.width()/2,-r.height()/2)
        #--------------------------------------
        self.setTransform(tr)
        if update:
            self.stateChanged(finish=finish)
    """



    
#---------------------------------------------------------------------------------------------------------------
#                                                                             DYNAMIC PLOT GUI
#---------------------------------------------------------------------------------------------------------------      
class PyqtgWindow(QtWidgets.QMainWindow):
    listTailSegments=[]
    
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent=None)
        
        self.setWindowIcon(QtgGui.QIcon('Imagys_blue\logoAnimotion-square-112.png'))
        ## Switch to using white background and black foreground
        #pg.setConfigOption('background', 'w')
        #pg.setConfigOption('foreground', 'k')
        
#        self.win = QtgGui.QMainWindow()
        self.area = DockArea()
        self.setCentralWidget(self.area)
        self.resize(1500,800)
        self.setWindowTitle('Xenopus project - beta')
        
        self.penCyan=pg.mkPen((0,255,255), width=2)
        self.penOrange=pg.mkPen((255,128,0), width=2)
        self.penGreen=pg.mkPen((255,128,0), width=2)
        
        ## Create docks, place them into the window one at a time.
        ## Note that size arguments are only a suggestion; docks will still have to
        ## fill the entire dock area and obey the limits of their internal widgets.
        self.d1 = Dock("Image", size=(1000,500))
        #self.d12 = Dock("Live", size=(1000,500))
        self.d2 = Dock("Analysis Settings", size=(500,200))#(1, 1))     ## give this dock the minimum possible size
        #d2 = Dock("Dock2 - Console", size=(500,300), closable=True)
        self.d3 = Dock("Eye 1", size=(500,200))
        self.d4 = Dock("Eye 2", size=(500,200))
        self.d5 = Dock("Tail", size=(500,200))
        self.d6 = Dock("Eye 1-Y", size=(500,200))
        self.d7 = Dock("Eye 2-Y", size=(500,200))
        self.d8 = Dock("Regions of interest", size=(500,200))
        self.d9 = Dock("Video player", size=(500,200))
        self.d10 = Dock("Segmentation settings", size=(500,200))
        #self.d11 = Dock("Capture settings",size=(500,200))
        self.d13 = Dock("Cam Settings", size=(500,200))#(1, 1))     ## give this dock the minimum possible size
        #self.area.addDock(self.d12, 'left')      ## place d12 at left edge of dock area (it will fill the whole space since there are no other docks yet)
        self.area.addDock(self.d1, 'left')
        self.area.addDock(self.d2, 'bottom', self.d1)## place d2 at bottom edge of d1
        self.area.addDock(self.d3, 'right')     ## place d3 at right edge of dock area
        self.area.addDock(self.d4, 'bottom', self.d3)  ## place d4 at bottom edge of d3
        self.area.addDock(self.d5, 'bottom', self.d4)
        self.area.addDock(self.d6, 'bottom', self.d5)
        self.area.addDock(self.d7, 'bottom', self.d6)
        
        #self.area.addDock(self.d11, 'bottom', self.d1)
        self.area.addDock(self.d9, 'bottom', self.d1)
        self.area.addDock(self.d10, 'above', self.d2)
        self.area.addDock(self.d8, 'above', self.d10) # positionne le dock d8-"console" en onglet au-dessus de d2-"setting"
        self.area.addDock(self.d13, 'above', self.d2)
        """
        ## Test ability to move docks programatically after they have been placed
        area.moveDock(d4, 'top', d2)     ## move d4 to top edge of d2
        area.moveDock(d6, 'above', d4)   ## move d6 to stack on top of d4
        area.moveDock(d5, 'top', d2)     ## move d5 to top edge of d2
        """
        
        ## Add widgets into each dock----------------------------------------------------------------------------VIDEO DISPLAY
        ## image/video widget
        """
        self.w1 = pg.ImageView()
        self.w1.setImage(np.random.normal(size=(100,100)))
        self.d1.addWidget(self.w1)
        """
        self.w1=pg.GraphicsLayoutWidget()
        self.plotView=self.w1.addPlot()
        ## lock the aspect ratio so pixels are always square
        self.plotView.setAspectLocked(True)
        ## Create image item
        self.img = pg.ImageItem()
        #self.plotView.addItem(self.img)
        #self.view.scene().addItem(self.img)
        
        self.viewBox=self.plotView.vb
        
        self.viewBox.addItem(self.img)
        self.d1.addWidget(self.w1)
        
        self.proxy1 = pg.SignalProxy(self.plotView.scene().sigMouseClicked, rateLimit=60, slot=mouseClicked)
        
        #marqueurs de position mark
        self.plotView.addItem(mark)
        #infiniteline pour marquer l'axe du corps
        #self.bodyAxeLine = pg.InfiniteLine(movable=False)
        #arrow pour marquer l'axe de chaque oeil
        #a = pg.ArrowItem(angle=-120, tipAngle=30, baseAngle=20, headLen=40, tailLen=40, tailWidth=8, pen=None, brush='y')
        self.eyeAxeLine1 = pg.InfiniteLine(movable=False)
        ##self.proxy2 = pg.SignalProxy(self.eyeAxeLine1.sigPositionChanged, rateLimit=60, slot=plotDataEye)
        self.eyeAxeLine2 = pg.InfiniteLine(movable=False)
        self.eyeAxeLines=[self.eyeAxeLine1,self.eyeAxeLine2]
                
        """
        #TODO : générer les ellipse lors de la création des roi autour des yeux--------------------------------------------------------------------------------------
        self.eyeEllipse1=EllipseROI_centered_noHandle(pos=[1,1],size=[1,1],pen=(3,5))
        #self.proxy2 = pg.SignalProxy(self.eyeEllipse1.sigRegionChanged, rateLimit=30, slot=plotDataEye1)
        #self.proxy5 = pg.SignalProxy(self.eyeEllipse1.sigRegionChanged, rateLimit=30, slot=plotDataEye1Y)
        self.plotView.addItem(self.eyeEllipse1)
        
        self.eyeEllipse2=EllipseROI_centered_noHandle(pos=[1,1],size=[1,1],pen=(3,5))
        #self.proxy3 = pg.SignalProxy(self.eyeEllipse2.sigRegionChanged, rateLimit=30, slot=plotDataEye2)
        #self.proxy6 = pg.SignalProxy(self.eyeEllipse2.sigRegionChanged, rateLimit=30, slot=plotDataEye2Y)
        #rateLimit= signals per second
        self.plotView.addItem(self.eyeEllipse2)
        
        #self.eyeEllipses=[self.eyeEllipse1,self.eyeEllipse2]
        roisEllipseEye=[self.eyeEllipse1,self.eyeEllipse2]
        """
        #----------------------------------------------------------------------------------------
        
               
        #ajouter une région de sélection horizontale pour définir un segment de queue
        #sa position sera définie dans "complete_skeleton"
        self.regionlr = pg.LinearRegionItem([0, 0], bounds=[0,0], movable=True)
        self.regionlr.sigRegionChanged.connect(update_tailRegionPosition)
        self.plotView.addItem(self.regionlr)
        self.listTailSegments.append(self.regionlr)
        
        #l'ajout des autres régions sera fait lors du "complete"
#        #ajout des régions en fonction de la valeur par défaut de la spinbox
#        self.add_tailRegion()
        
        self.curveTail =pg.PlotDataItem(x=[], y=[], pen=pg.mkPen(color='#3c02fc'))
        self.plotView.addItem(self.curveTail)
        
              
        #roi centrée sur le centre du contour détecté sur la queue
        self.tailEllipse=EllipseROI_centered_noHandle(pos=[1,1],size=[1,1],pen=(3,5))
        #self.proxy4 = pg.SignalProxy(self.tailEllipse.sigRegionChanged, rateLimit=30, slot=plotDataTail)
        self.plotView.addItem(self.tailEllipse)
        #self.tailCircle=pg.CircleROI(pos=[0,0],size=[30,30],pen=(3,5))
        #self.proxy4 = pg.SignalProxy(self.tailCircle.sigRegionChanged, rateLimit=30, slot=plotDataTail)
        #self.plotView.addItem(self.tailCircle)
        #-----------------------------------------------------------------------------------------------------------------------
        
        
        """               
        ##----------------------------------------------------------------------------------------------LIVE DISPLAY
        ## image/video widget
        
        self.w12=pg.GraphicsLayoutWidget()
        self.plotView_Live=self.w12.addPlot()
        ## lock the aspect ratio so pixels are always square
        self.plotView_Live.setAspectLocked(True)
        ## Create image item
        self.img_Live = pg.ImageItem()
        #self.plotView.addItem(self.img)
        #self.view.scene().addItem(self.img)
        
        self.viewBox_Live=self.plotView_Live.vb
        
        self.viewBox_Live.addItem(self.img_Live)
        self.d12.addWidget(self.w12)
        """
        """
        self.page_2 = QtWidgets.QWidget()
        self.gridLayout_4 = QtWidgets.QGridLayout(self.page_2)
        self.rawImage_Live = RawImageWidget(self.page_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.rawImage_Live.sizePolicy().hasHeightForWidth())
        self.rawImage_Live.setSizePolicy(sizePolicy)
        self.rawImage_Live.setObjectName("rawImg")
        self.gridLayout_4.addWidget(self.rawImage_Live, 0, 0, 1, 1)
        self.d12.addWidget(self.page_2)
        """
        
        #-----------------------------------------------------------------------------------------------------------------------
        

        ## dock2 gets save/restore buttons-------------------------------------------------------------------dock2
        self.w2 = pg.LayoutWidget()
        self.label = QtWidgets.QLabel(""" -- DockArea use -- 
        This window has 7 Dock widgets in it. Each dock can be dragged
        by its title bar to occupy a different space within the window.
        Additionally, the borders between docks may be dragged to resize. Docks that are dragged on top
        of one another are stacked in a tabbed layout. Double-click a dock title
        bar to place it in its own window.
        """)
        saveBtn = QtWidgets.QPushButton('Save dock state')
        restoreBtn = QtWidgets.QPushButton('Restore dock state')
        restoreBtn.setEnabled(False)
        self.w2.addWidget(self.label, row=0, col=0)
        self.w2.addWidget(saveBtn, row=1, col=0)
        self.w2.addWidget(restoreBtn, row=2, col=0)
        self.d2.addWidget(self.w2)
        state = None
        def save():
            global state
            state = self.area.saveState()
            restoreBtn.setEnabled(True)
        def load():
            global state
            self.area.restoreState(state)
        saveBtn.clicked.connect(save)
        restoreBtn.clicked.connect(load)
        
        
        ## Hide title bar on dock 3
        #d3.hideTitleBar()
        self.w3 = pg.PlotWidget(title="Eye 1")
        #self.w3.plot(np.random.normal(size=100))
        self.d3.addWidget(self.w3)
        
        self.w4 = pg.PlotWidget(title="Eye 2")
        #self.w4.plot(np.random.normal(size=100))
        self.d4.addWidget(self.w4)
        
        self.w5 = pg.PlotWidget(title="tail")
        #self.w5.plot(np.random.normal(size=100))
        self.d5.addWidget(self.w5)
        
        self.w6 = pg.PlotWidget(title="Eye 1 Ymove")
        #self.w6.plot(np.random.normal(size=100))
        self.d6.addWidget(self.w6)
        
        self.w7 = pg.PlotWidget(title="Eye 2 Ymove")
        #self.w7.plot(np.random.normal(size=100))
        self.d7.addWidget(self.w7)
        
               
        #------------------------------------------------------------------------------------------dock8 - SELECTIONS
        ## dock8 Regions of interest : 
        self.w8 = pg.LayoutWidget()
        self.label8 = QtWidgets.QLabel(""" Selection type  """)
        splitter_Selection = QtWidgets.QSplitter()
        splitter_Selection.setOrientation(QtCore.Qt.Vertical)
        self.selectEyes_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectEyes_radioButton.setToolTip("just click on the eye to trace the roi")
        self.selectEyes_radioButton.setChecked(True)
        self.selectEyes_radioButton.setText("Eye")
        self.selectTailRoot_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectTailRoot_radioButton.setText("Tail root")
        #self.tailRootPos_label = QtWidgets.QLabel("-- : --") #bornes de la zone d'analyse de la queue
        self.selectLimbs_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectLimbs_radioButton.setText("Limb")
        self.selectExclusion_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectExclusion_radioButton.setText("Circle (exclusion)")
        
        splitter_Background = QtWidgets.QSplitter()
        splitter_Background.setOrientation(QtCore.Qt.Horizontal)
        self.whiteBgd_radioButton = QtWidgets.QRadioButton(splitter_Background)
        self.whiteBgd_radioButton.setToolTip("background color")
        self.whiteBgd_radioButton.setChecked(True)
        self.whiteBgd_radioButton.setText("White background")
        
        self.blackBgd_radioButton = QtWidgets.QRadioButton(splitter_Background)
        self.blackBgd_radioButton.setToolTip("background color")
        self.blackBgd_radioButton.setChecked(False)
        self.blackBgd_radioButton.setText("Black background")
        
        completeSegmentation_btn= QtWidgets.QPushButton('Complete')
        completeSegmentation_btn.clicked.connect(complete_skelet)
        initLimbTrack_btn= QtWidgets.QPushButton('init limbs track')
        initLimbTrack_btn.clicked.connect(init_track)
        #--------------------------------------------------------------------splitter_thresh
        
        splitter_ThreshTail=QtWidgets.QSplitter()
        tailSplitter_label= QtWidgets.QLabel(splitter_ThreshTail)
        tailSplitter_label.setText("Tail segments")
#        self.spinbox_tailSegNumber =  pg.SpinBox(value=6,int=True, minStep=1,step=1,bounds=[1,12])
#        self.spinbox_tailSegSize =  pg.SpinBox(value=6,int=True, minStep=2,step=2,bounds=[6,20])
        tailSegNumber_label= QtWidgets.QLabel(splitter_ThreshTail)
        tailSegNumber_label.setText("number")
        self.tailSegNumber_spinBox =  QtWidgets.QSpinBox(splitter_ThreshTail)
        self.tailSegNumber_spinBox.setValue(1)
        self.tailSegNumber_spinBox.setMinimum(1)
        self.tailSegNumber_spinBox.setMaximum(12)
        self.tailSegNumber_spinBox.setEnabled(False)
        tailSegSize_label= QtWidgets.QLabel(splitter_ThreshTail)
        tailSegSize_label.setText("size")
        self.tailSegSize_spinBox =  QtWidgets.QSpinBox(splitter_ThreshTail)
        self.tailSegSize_spinBox.setValue(6)
        self.tailSegSize_spinBox.setMinimum(6)
        self.tailSegSize_spinBox.setMaximum
        
        threshTailSlider_label= QtWidgets.QLabel(splitter_ThreshTail)
        threshTailSlider_label.setText("threshold")
        self.threshTail_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.threshTail_slider.setMaximum(255)
        self.threshTail_slider.setPageStep(10)
        self.threshTail_slider.setOrientation(QtCore.Qt.Horizontal)
        threshTailValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        threshTailValue_label.setText("00")
        threshTailValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.threshTail_slider.valueChanged.connect(threshTailValue_label.setNum)

        
        splitter_ThreshEyes=QtWidgets.QSplitter()
        #splitter_Thresh.setOrientation(QtCore.Qt.Vertical)
        threshEye1Slider_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye1Slider_label.setText("Eye1")
        self.threshEye1_slider = QtWidgets.QSlider(splitter_ThreshEyes)
        self.threshEye1_slider.setMaximum(255)
        self.threshEye1_slider.setPageStep(10)
        self.threshEye1_slider.setOrientation(QtCore.Qt.Horizontal)
        threshEye1Value_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye1Value_label.setText("00")
        threshEye1Value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.threshEye1_slider.valueChanged.connect(threshEye1Value_label.setNum)
        
        threshEye2Slider_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye2Slider_label.setText("Eye2")
        self.threshEye2_slider = QtWidgets.QSlider(splitter_ThreshEyes)
        self.threshEye2_slider.setMaximum(255)
        self.threshEye2_slider.setPageStep(10)
        self.threshEye2_slider.setOrientation(QtCore.Qt.Horizontal)
        threshEye2Value_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye2Value_label.setText("00")
        threshEye2Value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.threshEye2_slider.valueChanged.connect(threshEye2Value_label.setNum)
        
        self.threshTail_slider.setValue(90)
        self.threshEye1_slider.setValue(60)
        self.threshEye2_slider.setValue(60)
        
        #-----------------------------------------------------------------------------splitter_thresh
        saveConfig_btn = QtWidgets.QPushButton('Save config')
        #saveConfig_btn.clicked.connect(save_Config)
        loadConfig_btn = QtWidgets.QPushButton('Load config')
        #loadConfig_btn.clicked.connect(load_Config)
                
        self.w8.addWidget(self.label8, row=0, col=0)
        self.w8.addWidget(splitter_Background, row=0,col=1,colspan=2)
        self.w8.addWidget(splitter_Selection, row=1, col=0,rowspan=4)
        self.w8.addWidget(splitter_ThreshEyes, row=1, col=1,colspan=3)
        self.w8.addWidget(splitter_ThreshTail, row=2, col=1,colspan=3)
        #self.w8.addWidget(self.tailRootPos_label, row=1,col=1)
        self.w8.addWidget(completeSegmentation_btn, row=0, col=3)
        self.w8.addWidget(initLimbTrack_btn, row=3, col=1)
        self.w8.addWidget(loadConfig_btn, row=4, col=2)
        self.w8.addWidget(saveConfig_btn, row=4, col=3)
        self.d8.addWidget(self.w8)
        #----------------------------------------------------------------------------------
        
        #---------------------------------------------------------------------------------------dock10 - SEGMENTATION
        ## dock10 Segmentation
        
        self.w10 = pg.LayoutWidget()
       
        self.d10.addWidget(self.w10)
        #-------------------------------------------------------------------------------------
        
        #-----------------------------------------------------------------------------------------dock9 - VIDEO PLAYER
        ## dock9 video player : 
        self.w9 = pg.LayoutWidget()
        self.videoName_label = QtWidgets.QLabel(""" No video """)
        
        loadVideo_btn = QtWidgets.QPushButton('Load video')
        loadVideo_btn.clicked.connect(load_video)
        
        self.connectCam_btn = QtWidgets.QPushButton('Connect camera')
        #self.connectCam_btn.setCheckable(True)
        #self.connectCam_btn.setChecked(False)
        self.connectCam_btn.clicked.connect(init_acquisitionDevice)
        
        self.liveVideo_btn = QtWidgets.QPushButton('Live')
        self.liveVideo_btn.setCheckable(True)
        self.liveVideo_btn.setChecked(False)
        self.liveVideo_btn.setEnabled(False)
        self.liveVideo_btn.setMaximumWidth(100) 
        self.liveVideo_btn.clicked.connect(grab_video)
        
        self.measuredLivefps_Label=QtWidgets.QLabel(""" 00 fps """)
        self.measuredLivefps_Label.setMaximumWidth(100) 
        
        splitter_live= QtWidgets.QSplitter()
        splitter_live.setOrientation(QtCore.Qt.Horizontal)
        splitter_live.addWidget(self.liveVideo_btn)
        splitter_live.addWidget(self.measuredLivefps_Label)
                
        """        
        self.importLive_btn = QtWidgets.QPushButton('import Live')
        self.importLive_btn.clicked.connect(import_live)
        self.importLive_btn.setEnabled(False)
        """
        resetPlots_btn = QtWidgets.QPushButton('Reset plots')
        resetPlots_btn.clicked.connect(reset_Plot)
        resetBuffer_btn = QtWidgets.QPushButton('Reset buffer')
        resetBuffer_btn.clicked.connect(reset_Buffer)
        saveResults_btn = QtWidgets.QPushButton('Save results')
        saveResults_btn.clicked.connect(save_ResultsNow)
        
        self.playVideo_btn = QtWidgets.QPushButton('Play/Pause')
        self.playVideo_btn.setCheckable(True)
        self.playVideo_btn.setChecked(False)
        self.playVideo_btn.clicked.connect(playpause_Video)
        self.playVideo_btn.setEnabled(False)
        
        self.progressBuffer=QtWidgets.QProgressBar()
        self.progressBuffer.setMinimumWidth(20)
        self.progressBuffer.setMaximumWidth(200)
        
        #self.track_checkBox = QtWidgets.QCheckBox("Track")
        self.track_checkBox = QtWidgets.QPushButton('Track')
        self.track_checkBox.setCheckable(True)
        self.track_checkBox.setChecked(False)
        self.track_checkBox.clicked.connect(check_AnalysisParameters)
        
        self.trigger_checkBox = QtWidgets.QCheckBox("Trigger")
                
        self.stepFwdVideo_btn = QtWidgets.QPushButton('step >')
        self.stepFwdVideo_btn.clicked.connect(forward_Video)
        self.stepFwdVideo_btn.setEnabled(False)
        self.stepBwdVideo_btn = QtWidgets.QPushButton('< step')
        self.stepBwdVideo_btn.clicked.connect(backward_Video)
        self.stepBwdVideo_btn.setEnabled(False)
        
        self.manipType_comboBox = QtWidgets.QComboBox()
        self.manipType_comboBox.addItem("choose an analysis mode...")
        self.manipType_comboBox.addItem("eyes-tail track")
        self.manipType_comboBox.addItem("eyes track only")
        self.manipType_comboBox.addItem("eyes-limbs track")
        self.manipType_comboBox.addItem("limbs track only")
        self.manipType_comboBox.addItem("test live")
        self.manipType_comboBox.activated.connect(init_analysisMode)
        
        
        numFrame_label = QtWidgets.QLabel("00")
        numFrame_label.setAlignment(QtCore.Qt.AlignCenter)
        
        self.lenBuffer_label = QtWidgets.QLabel("00")
        self.lenBuffer_label.setAlignment(QtCore.Qt.AlignCenter)
                
        self.timeLine_slider = QtWidgets.QSlider()
        self.timeLine_slider.setPageStep(100)
        self.timeLine_slider.setProperty("value", 1)
        self.timeLine_slider.setOrientation(QtCore.Qt.Horizontal)
        self.timeLine_slider.valueChanged.connect(slider_Move)
        self.timeLine_slider.valueChanged['int'].connect(numFrame_label.setNum)
        self.timeLine_slider.setEnabled(False)
        
        self.measuredPlayfps_Label=QtWidgets.QLabel(""" 00 fps """)
        
        self.w9.addWidget(self.videoName_label, row=0, col=0,colspan=5)
        self.w9.addWidget(loadVideo_btn, row=1, col=0)
        self.w9.addWidget(self.connectCam_btn,row=2,col=0)
        #self.w9.addWidget(self.liveVideo_btn,row=2,col=0)
        #self.w9.addWidget(self.measuredLivefps_Label,row=2,col=1)
        self.w9.addWidget(splitter_live,row=2,col=1)
        #self.w9.addWidget(self.importLive_btn,row=3,col=0)
        self.w9.addWidget(self.trigger_checkBox, row=3,col=1)
        self.w9.addWidget(self.manipType_comboBox,row=1,col=1)
        self.w9.addWidget(self.progressBuffer,row=1,col=2)
        self.w9.addWidget(self.lenBuffer_label,row=2,col=2)
        self.w9.addWidget(resetPlots_btn,row=1,col=3)
        self.w9.addWidget(resetBuffer_btn,row=2,col=3)
        self.w9.addWidget(saveResults_btn,row=1,col=4)
        self.w9.addWidget(self.playVideo_btn,row=4,col=0)
        self.w9.addWidget(self.track_checkBox,row=3,col=2)
        self.w9.addWidget(self.stepBwdVideo_btn,row=4,col=2)
        self.w9.addWidget(self.stepFwdVideo_btn,row=4,col=4)
        self.w9.addWidget(numFrame_label,row=4,col=3)
        self.w9.addWidget(self.timeLine_slider,row=5,colspan=5)
        self.w9.addWidget(self.measuredPlayfps_Label,row=4,col=1)
        
        self.d9.addWidget(self.w9)
        #-------------------------------------------------------------------------------------
        
        
        
        
        #------------------------------------------------------------------------------------------------- dock13 "Camera Settings"
        ## dock13 cam Settings : 
        self.w13 = pg.LayoutWidget()
        """
        self.labelImageFormat=QtWidgets.QLabel("Image Format : ")
        #width
        self.labelWidth=QtWidgets.QLabel("Width : ")
        self.spinbox_sensorAOIWidth =  pg.SpinBox(value=1920,int=True, minStep=1,step=1,bounds=[1,None])
        #height
        self.labelHeight=QtWidgets.QLabel("Height : ")
        self.spinbox_sensorAOIHeight =  pg.SpinBox(value=1200,int=True, minStep=1,step=1,bounds=[1,None])
        #offsetX
        self.labelOffsetX=QtWidgets.QLabel("OffsetX : ")
        self.spinbox_sensorAOIOffsetX =  pg.SpinBox(value=32,int=True, minStep=1,step=1,bounds=[1,None])
        #offsetY
        self.labelOffsetY=QtWidgets.QLabel("OffsetY : ")
        self.spinbox_sensorAOIOffsetY =  pg.SpinBox(value=32,int=True, minStep=1,step=1,bounds=[1,None])
        self.labelAnalogControl=QtWidgets.QLabel("Analog Control : ")
        
        self.labelAcquisitionControl=QtWidgets.QLabel("Acquisition Control : ")
        """
        #module pylon_parametersListmodule2b        
        self.w13.addWidget(paramTree.tree,rowspan=0,col=0)
        self.d13.addWidget(self.w13)        
        #-------------------------------------------------------------------------------------
        
        # loads and sets the Qt stylesheet
        qss = QSSHelper.open_qss(os.path.join('Imagys_blue', 'Imagys-blue.qss'))
        self.setStyleSheet(qss)                   
        self.show()
        
    def add_tailRegion(self):
        #"nettoyage" puis ajout de nouvelles régions d'analyse pour les segments de queue
        if len(self.listTailSegments)>1:
            for tailSegment in self.listTailSegments[1:] :
                self.plotView.removeItem(tailSegment)    
            del self.listTailSegments[1:]
                        
        for i in range(self.tailSegNumber_spinBox.value()-1): #value-1 regionlr est par défaut dans la liste
            tailSegment= pg.LinearRegionItem([0, 0], bounds=[0,0], movable=False)   
            self.plotView.addItem(tailSegment)
            self.listTailSegments.append(tailSegment)
            
        print("nombre de zone sur la queue : ", len(self.listTailSegments))
                   
   
def clean_app():
    print("")
    print("quit app")
    #TODO: close resultFile (resultFile.close())    
    #varA.quitApp=True
    try :
        print("close stream")
        video.grabber.stream.Close()
    except AttributeError as e :
        #QtWidgets.QMessageBox.warning(None,"Error",str(ValueError),QtWidgets.QMessageBox.Ok)
        print("no stream was opened")
    try :
        print("close camera")
        video.device.Close()
    except AttributeError as e :
        #QtWidgets.QMessageBox.warning(None,"Error",str(ValueError),QtWidgets.QMessageBox.Ok)
        print("no camera was opened")
    
    print("close cv2 windows")
    cv2.destroyAllWindows()
    #PylonTerminate()
    
    #sys.exit(app.exec_())    
    #os._exit(1) 


    
    
    ## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    from Imagys_blue.qsshelper import QSSHelper
    
    app = QtWidgets.QApplication([])
   
    #initialisation de PyQt
    #app = QtWidgets.QApplication(sys.argv)
    mark = graphMark()
    #interface utilisateur
    #paramTree=baslerParameters.PyqtgParamWidget()
    paramTree=CamParamWidget()
    #ui=PyqtgWindow()
    ui=PyqtgWindow()
    #ouvrir l'onglet "method"
    ui.d8.raiseDock()
    
    video=Video()
    image=ImageContainer()
    
    #roi pour le tracking des membres
    target=Target()
    varM=Measure_Var()
    
    
    app.exec_()
    clean_app()
    print("the end")
    app.quit()