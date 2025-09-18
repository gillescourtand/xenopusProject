# -*- coding: utf-8 -*-
"""
Created on Wed Nov 10 14:22:52 2021

@author: courtand
"""

"""
v_2 : bouton rescale pour réduire le temps d'affichage de la video'
        signal emit loadVideoSignal
v_2c : correction plantage en fin de video
v_2d : gestion des signaux, ajout playSignal.emit à l'appui du bouton playvideo'
v_3 : ajout du controle par le clavier
v_3b : rotation de la video en lecture (90, -90, 180)
v_3c : lecture avec saut de frame paramétrable : emit signal change playframerate
v_3d : lecture des frames fluide pour les petites videos
v_3e : signal de déplacement du slider timeline

2024
v_4 : remplacement de Pyqtgraph.QtGui par pyQt5.QWidgets (mise à jour pyqtgraph 0.13, python 10)

TODO : envoyer un signal en fin de video

"""

import os
from pathlib import Path

import time
import psutil
from PyQt5 import QtWidgets,QtCore
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSignal
#pyqtgraph
import pyqtgraph as pg
from pyqtgraph.dockarea import *
# import pyqtgraph.ptime as ptime
#from pyqtgraph.widgets.RawImageWidget import RawImageWidget
import pyqtgraph.debug as debug
import pyqtgraph.ThreadsafeTimer

import cv2
import numpy as np

from threading import Thread
import keyboard
import sys
# import the Queue class from Python 3
if sys.version_info >= (3, 0):
	from queue import Queue
# otherwise, import the Queue class for Python 2.7
else:
	from Queue import Queue

# sys.path.append(os.path.dirname(os.path.abspath(__file__)))
# from modules.Imagys_blue.qsshelper import QSSHelper


# image processor -----------------------------------------------------
class ImageProcessor:
   
    def gamma_LUT(self,gamma):
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
             for i in np.arange(0, 256)]).astype("uint8")
        # apply gamma correction using the lookup table
        return table         
    
    
    def convert_imageToPyqtgraph(self,img,vid):
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

#------------------------------------------------------------------------


def track_display():
    pass

def open_file(ui,fileType,fileFormat):
    """
    video : fileType= "Video file", fileFormat =  "(*.avi;*.mpg;*.mp4;*.mov;*.flv)"
    data : fileType = "Data file", fileFormat = "(*.csv)"
    state : fileType="Workspace state file", fileFormat="(*.wkp)"
    """
    filePath,_=QtWidgets.QFileDialog.getOpenFileName(
            ui,
            "Open "+ fileType,
            QtCore.QDir.homePath(),
            fileType + fileFormat 
                )
    return filePath      

def extract_fileName(filePath):
    splitPath=os.path.split(filePath)
    fileName=splitPath[-1]
    return fileName



def set_videoRotation(vid,frame):
    if vid.rotation=="90_clockwise":
        rframe = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)    
    elif vid.rotation=="90_counterclockwise":
        rframe = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)    
    elif vid.rotation=="180": 
        rframe = cv2.rotate(frame, cv2.ROTATE_180)
    return rframe


class Video:
    """classe rassemblant les différents paramètres video
    """
    def __init__(self):
        self.path=None
        
        self.capture=None
        #vidProp_dict['imageType']=image.dtype
          
        self.firstFrame=None
        self.LUT=None
        self.isAcquiring=False
        self.buffering=False
        self.playing=False
        self.grabber=None
        self.frameWeight=0
        self.measuredLivefps=0
        
        self.currentGrayFrame =None
        self.currentFrame = None
        self.currentTFrame = None #image transposée pour affichage dans plotview
        
        """Acquisition camera
        ---------------"""
        self.devices=[]
        self.device=None
        self.featuresFile=None
        
        self.pixFormat=None
        self.ToRGB=None #vidProp_dict['videoToRGB']=cap.get(16)
        self.width=None #vidProp_dict['videoWidth']=cap.get(3)
        self.height=None #vidProp_dict['videoHeight']=cap.get(4)      
        self.fps=None #vidProp_dict['fps']=cap.get(5)
        self.nbFrames=None #vidProp_dict['nbFrames']=cap.get(7)
        # self.pos=None #vidProp_dict['framePos']= cap.get(1) -->position du frame suivant     
        
        # self.resultFrameRate=None #cam.ResultingFrameRate.GetValue()
        # self.enableFrameRate=None #cam.AcquisitionFrameRateEnable.GetValue()
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
        
        self.rescale=1
        self.rotation="No rotation" #("90_clockwise", "90_counterclockwise", "180")
        """-------------------------------"""
             
        self.pos=None #vidProp_dict['framePos']= cap.get(1) -->position du frame suivant   
        
        
        

class FileVideoStream_ToPlay:
    """
    chargement des frames de la vidéo dans le buffer avant affichage
    """
    def __init__(self, videoStream, video, queueSize=200):
        """
        initialize the file video stream along with the boolean
        used to indicate if the thread should be stopped or not
        """
        self.stream = videoStream
        self.vid=video
        self.stopped = False
        self.frameStep=1
         
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=queueSize)

    def start(self,nframe,framerateFactor):
        """
        start a thread to read frames from the file video stream
        """
        self.frameStep=framerateFactor
        t = Thread(target=self.update, args=())
        t.daemon = True
        self.stream.set(1,nframe)
        t.start()
        return self

    def update(self):
        """
        boucler jusqu'à la fin de la video get(7)=nbre de frames cv2 videocapture
        """
        endFrame=self.stream.get(7)
        print("last frame : ",endFrame)
        print("frame factor : ", self.frameStep)
#        i=0
        try:
            #while True:
            while self.stream.get(1)<endFrame:
                # currentFrame=self.stream.get(1)
                # print(currentFrame)
                # if the thread indicator variable is set, stop the
                # thread
                if self.stopped:
                    print("videostreamToPlay stopped")
                    return
                
                # otherwise, ensure the queue has room in it
                if not self.Q.full():
                    # read the next frame from the file
                    # cap.get(1) means CAP_PROP_POS_FRAMES - 0-based index of the frame to be decoded/captured next.
                    #prendre .get(1)-1 pour avoir le frame courant
                    if self.frameStep!=1 :
                        self.stream.set(cv2.CAP_PROP_POS_FRAMES, self.stream.get(1)-1+self.frameStep)
                    #la ligne précédente rend la video saccadée (chargement trop long par rapport à la lecture ?)
                    (grabbed, frame) = self.stream.read()
                    currentFrame=self.stream.get(1)
                    # print("current frame read : ",currentFrame)
                    # if the `grabbed` boolean is `False`, then we have
                    # reached the end of the video file or there is an error reading this frame 
                    if not grabbed:
                        print("error : frame ",currentFrame, " not grabbed")
                        # self.stop()
                        return
                        
                    if self.vid.rescale!=1 :
                        frame=cv2.resize(frame, None, fx = self.vid.rescale, fy = self.vid.rescale,interpolation = cv2.INTER_CUBIC)

                    if self.vid.rotation!="No rotation" :
                        frame=set_videoRotation(self.vid,frame)
                    
                    # add the frame to the queue
                    self.Q.put(frame)
                    
                    # now = ptime.time()
                    # dt = now - varM.lastTime_live
                    # varM.lastTime_live = now
                    # if varM.measuredLivefps is None:
                    #     varM.measuredLivefps = 1.0/dt
                    # else:
                    #     s = np.clip(dt*3., 0, 1)
                    #     varM.measuredLivefps = varM.measuredLivefps * (1-s) + (1.0/dt) * s
                    # #print("load time: ",dt)
                   
                else:
                    time.sleep(2.0)
                    print("fulllll")
        # except sys.exc_info()[0] as e:
        #     #e = sys.exc_info()[0]
        #     print(e)
        #     QtWidgets.QMessageBox.warning(None,"Error",str(e),QtWidgets.QMessageBox.Ok)
        finally:
            print('read frame ended')
            self.stop()

    def read(self):
        """
        return next frame in the queue
        """
        return self.Q.get()
        
    def more(self):
        """
        return True if there are still frames in the queue
        """
        return self.Q.qsize() > 0    
        
    def stop(self):
        """
        indicate that the thread should be stopped
        """
        self.stopped = True     





class UIVideoPlayer(pg.LayoutWidget):
    
    analysisSignal=pyqtSignal(int)
    loadVideoSignal=pyqtSignal()
    playSignal=pyqtSignal()
    pauseSignal=pyqtSignal()
    updatePlotSignal=pyqtSignal()
    updateTrackMapSignal=pyqtSignal()
    playFramerateSignal=pyqtSignal()
    timeLineMoveSignal=pyqtSignal(int)
    
    
    def __init__(self,videoDisplay,video, icon_path, parent=None):
        super(UIVideoPlayer,self).__init__()
        
        self.vid=video
        self.vidDisplay=videoDisplay
        self.image_processor=ImageProcessor()

       
        self.framerateFactor=1
        self.idxResultArray=0 
                
        self.loadVideo_btn = QtWidgets.QPushButton('Load video')
        self.loadVideo_btn.clicked.connect(self.load_video)
        self.lb_videoName = QtWidgets.QLabel(""" No video """)
        self.lb_videoName.setMaximumWidth(200)
        self.lb_videoName.setMinimumWidth(100)
        
                
        self.playVideo_btn = QtWidgets.QPushButton()
        self.playVideo_btn.setCheckable(True)
        self.playVideo_btn.setChecked(False)
        self.playVideo_btn.clicked.connect(self.playpause_video)
        self.playVideo_btn.setEnabled(False)
        # Set initial icon for the unchecked state
        play_icon_path=os.path.join("Imagys_blue","play_button_off.png")
        play_icon = resource_path(play_icon_path)
        # play_icon=os.path.join(icon_path,"play_button_off.png")
        self.playVideo_btn.setIcon(QIcon(play_icon))
        self.playVideo_btn.setMaximumWidth(80)
        self.playVideo_btn.setMinimumWidth(40)
        self.playVideo_btn.setMaximumHeight(20)

        self.stepFwdVideo_btn = QtWidgets.QPushButton('>')
        # self.stepFwdVideo_btn.setStyleSheet(''' font-size: 18px; ''')
        self.stepFwdVideo_btn.clicked.connect(self.forward_video)
        self.stepFwdVideo_btn.setEnabled(False)
        self.stepFwdVideo_btn.setMaximumWidth(40)
        self.stepFwdVideo_btn.setMinimumWidth(40)
        self.stepBwdVideo_btn = QtWidgets.QPushButton('<')
        # self.stepBwdVideo_btn.setStyleSheet(''' font-size: 18px; ''')
        self.stepBwdVideo_btn.clicked.connect(self.backward_video)
        self.stepBwdVideo_btn.setEnabled(False)
        self.stepBwdVideo_btn.setMaximumWidth(40)
        self.stepBwdVideo_btn.setMinimumWidth(40)
        
        # -------------------------------------------------------nb frames
        self.numFrame_label = QtWidgets.QLabel("00")
        self.numFrame_label.setAlignment(QtCore.Qt.AlignCenter)
        self.numFrame_label.setMaximumWidth(60)
        numFrameSeparator_label = QtWidgets.QLabel("/")
        numFrameSeparator_label.setAlignment(QtCore.Qt.AlignCenter)
        numFrameSeparator_label.setMaximumWidth(3)
        self.numTotalFrame_label = QtWidgets.QLabel("00 frame")
        self.numTotalFrame_label .setAlignment(QtCore.Qt.AlignCenter)
        self.numTotalFrame_label.setMaximumWidth(60)
        splitter_numFrame = QtWidgets.QSplitter()
        splitter_numFrame.setOrientation(QtCore.Qt.Horizontal)
        splitter_numFrame.addWidget(self.numFrame_label)
        splitter_numFrame.addWidget(numFrameSeparator_label)
        splitter_numFrame.addWidget(self.numTotalFrame_label)
                   
        # -------------------------------------------------------timecode
        self.currentTime_label = QtWidgets.QLabel("00")
        self.currentTime_label.setAlignment(QtCore.Qt.AlignCenter)
        self.currentTime_label.setMaximumWidth(60)
        timeSeparator_label = QtWidgets.QLabel("/")
        timeSeparator_label.setAlignment(QtCore.Qt.AlignCenter)
        timeSeparator_label.setMaximumWidth(3)
        self.totalTime_label = QtWidgets.QLabel("00 sec.")
        self.totalTime_label .setAlignment(QtCore.Qt.AlignCenter)
        self.totalTime_label.setMaximumWidth(60)
        splitter_timeCode = QtWidgets.QSplitter()
        splitter_timeCode.setOrientation(QtCore.Qt.Horizontal)
        splitter_timeCode.addWidget(self.currentTime_label)
        splitter_timeCode.addWidget(timeSeparator_label)
        splitter_timeCode.addWidget(self.totalTime_label)
        
        splitter_frame_time= QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter_frame_time.addWidget(splitter_numFrame)
        splitter_frame_time.addWidget(splitter_timeCode)

        
        self.timeLine_slider = QtWidgets.QSlider()
        self.timeLine_slider.setPageStep(100)
        self.timeLine_slider.setProperty("value", 1)
        self.timeLine_slider.setOrientation(QtCore.Qt.Horizontal)
        self.timeLine_slider.valueChanged.connect(self.slider_move)
        self.timeLine_slider.valueChanged['int'].connect(self.numFrame_label.setNum)
        self.timeLine_slider.setEnabled(False)
       
        splitter_frameplayBtn = QtWidgets.QSplitter()
        splitter_frameplayBtn.setOrientation(QtCore.Qt.Horizontal)
        splitter_frameplayBtn.addWidget(self.timeLine_slider)
        # splitter_frameplayBtn.addWidget(splitter_numFrame)
        splitter_frameplayBtn.addWidget(self.playVideo_btn)
        splitter_frameplayBtn.addWidget(self.stepBwdVideo_btn)
        splitter_frameplayBtn.addWidget(self.stepFwdVideo_btn)

                    
        
        # player options-------------------------------------------------------------------------
        #video speed
        self.playSpeed_label = QtWidgets.QLabel("each n frame")
        self.playSpeed_label.setMaximumWidth(80)
        #self.playSpeed_spinbox = pg.SpinBox(value=1,int=True, minStep=1,step=1,bounds=[1,None]) #taille du widget démesurée !
        self.playSpeed_spinbox = QtWidgets.QSpinBox()
        self.playSpeed_spinbox.setMaximumWidth(50)
        self.playSpeed_spinbox.setMinimum(1)
        self.playSpeed_spinbox.setMaximum(200)
        self.playSpeed_spinbox.setValue(1)
        self.playSpeed_spinbox.valueChanged.connect(self.set_framerateFactor)
        self.splitter_playSpeed = QtWidgets.QSplitter()
        self.splitter_playSpeed.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_playSpeed.addWidget( self.playSpeed_label)
        self.splitter_playSpeed.addWidget(self.playSpeed_spinbox)
        # splitter_playSpeed.setEnabled(False)
        
        #video scale
        self.videoScale_label = QtWidgets.QLabel("video scale 1/")
        self.videoScale_label.setMaximumWidth(80)
        self.videoScale_label.setMinimumWidth(80)
        #self.playSpeed_spinbox = pg.SpinBox(value=1,int=True, minStep=1,step=1,bounds=[1,None]) #taille du widget démesurée !
        self.videoScale_spinbox = QtWidgets.QSpinBox()
        self.videoScale_spinbox.setMaximumWidth(50)
        self.videoScale_spinbox.setMinimum(1)
        self.videoScale_spinbox.setMaximum(6)
        self.videoScale_spinbox.setValue(1)
        self.videoScale_spinbox.valueChanged.connect(self.set_videoScale)
        splitter_videoScale = QtWidgets.QSplitter()
        splitter_videoScale.setOrientation(QtCore.Qt.Horizontal)
        splitter_videoScale.addWidget( self.videoScale_label)
        splitter_videoScale.addWidget(self.videoScale_spinbox)
        splitter_videoScale.setEnabled(True)
        
        #video rptation
        self.videoRotation_comboBox = QtWidgets.QComboBox()
        self.videoRotation_comboBox.addItem("No rotation")
        self.videoRotation_comboBox.addItem("90_clockwise")
        self.videoRotation_comboBox.addItem("90_counterclockwise")
        self.videoRotation_comboBox.addItem("180")
        self.videoRotation_comboBox.activated.connect(self.apply_frameRotation)
        
        splitter_playerOptions = QtWidgets.QSplitter()
        splitter_playerOptions.setOrientation(QtCore.Qt.Horizontal)
        splitter_playerOptions.addWidget(self.splitter_playSpeed)
        splitter_playerOptions.addWidget(splitter_videoScale)
        splitter_playerOptions.addWidget(self.videoRotation_comboBox)

        #-------------------------------------------------------------------------------------
        #---------------------------------------------------------------------------------------
        
        #video buffer informations ---------------------------------------------------------
        availableMemory_label= QtWidgets.QLabel("Available memory (MB) : ")
        availableMemory_label.setMinimumWidth(120)
        availableMemory_label.setMaximumWidth(120)
        self.availableMemValue_label= QtWidgets.QLabel("0000")
        # self.availableMemValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.availableMemValue_label.setMinimumWidth(40)
        self.availableMemValue_label.setMaximumWidth(60)
        self.bufferSize_label= QtWidgets.QLabel("Buffer size (frames) ")
        self.bufferSize_label.setMinimumWidth(100)
        self.bufferSize_label.setMaximumWidth(100)
        self.bufferSizeValue_label= QtWidgets.QLabel("00000")
        self.bufferSizeValue_label.setMinimumWidth(40)
        self.bufferSizeValue_label.setMaximumWidth(60)
        self.bufferSizeValue_label.setAlignment(QtCore.Qt.AlignCenter)
        
        self.bufferSizeValMB_label= QtWidgets.QLabel("000MB")
        self.bufferSizeValMB_label.setMinimumWidth(60)
        self.bufferSizeValMB_label.setMaximumWidth(60)
        self.bufferSizeValMB_label.setAlignment(QtCore.Qt.AlignCenter)
        # bufferSizeMB_label= QtWidgets.QLabel("MB")
        # bufferSizeMB_label.setMinimumWidth(20)
        self.bufferSizeValue_slider= QtWidgets.QSlider()
        self.bufferSizeValue_slider.setProperty("value", 0)
        self.bufferSizeValue_slider.setOrientation(QtCore.Qt.Horizontal)
        self.bufferSizeValue_slider.setMinimumWidth(100)
        self.bufferSizeValue_slider.valueChanged.connect(self.bufferSizeMB_update)
        self.bufferSizeValue_slider.valueChanged['int'].connect(self.bufferSizeValue_label.setNum)
                
        self.buffer_progress=QtWidgets.QProgressBar()
        self.buffer_progress.setMinimumWidth(60)
        self.buffer_progress.setMaximumWidth(200)
        self.buffer_progress.setMaximumHeight(6)
        self.buffer_progress.setTextVisible(False)
        
        splitterBuffer = QtWidgets.QSplitter()
        splitterBuffer.setOrientation(QtCore.Qt.Horizontal)
        splitterBuffer.addWidget(availableMemory_label)
        splitterBuffer.addWidget(self.availableMemValue_label)
        splitterBuffer.addWidget(self.bufferSize_label)
        splitterBuffer.addWidget(self.bufferSizeValue_slider)
        splitterBuffer.addWidget(self.bufferSizeValue_label)
        splitterBuffer.addWidget(self.bufferSizeValMB_label)
        # splitterBuffer.addWidget(bufferSizeMB_label)
        splitterBuffer.addWidget(self.buffer_progress)
        splitterBuffer.setEnabled(False)
        
        #--------------------------------------------------------------------------------------------
        # layout --------------------------------------------------------------------------------------
        # self.addWidget(self.timeLine_slider,row=0,col=0,colspan=5)
        # self.addWidget(splitter_numFrame,row=0,col=5)
        # self.addWidget(self.stepBwdVideo_btn,row=0,col=6)
        # self.addWidget(self.stepFwdVideo_btn,row=0,col=7)
        self.addWidget(splitter_frameplayBtn,row=0,col=0,colspan=6)
        self.addWidget(self.loadVideo_btn,row=1,col=0)
        self.addWidget(self.lb_videoName, row=1,col=1,colspan=3)
        # self.addWidget(splitterBuffer,row=2,col=1,colspan=6)
        # self.addWidget(self.playVideo_btn,row=3,col=0)
        # self.addWidget(self.playSpeed_spinbox,row=3,col=1)
        self.addWidget(splitter_playerOptions,row=1,col=4,colspan=2)
        # self.addWidget(self.splitter_playSpeed,row=3,col=1)
        # self.addWidget(splitter_videoScale,row=3,col=2)
        # self.addWidget(self.videoRotation_comboBox,row=3,col=3)
        self.addWidget(splitter_frame_time,row=0,col=6,rowspan=2)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.video_play_framerate)
        #----------------------------------------------------------------------------------------LAYOUT-
        
        
    def load_video(self):
        print("load...")
        # global video
        # video= Video()  
        self.vid.path=open_file(None,'Video file',' *.mpg;*.mp4;*.mov;*.mkv;*.avi')
        print("open :", self.vid.path)

        #if self.vid.path!="" or self.vid.path!=" "or self.vid.path!=None:
        if len(self.vid.path)>3:
            self.init_video(self.vid.path)    
        else : 
            print("no image ")
        
    
    def init_video(self,videoPath):
        """
        récupération des infos sur la video    
        """
        
        #extraction du nom de la vidéo
        videoName=extract_fileName(videoPath)
        self.lb_videoName.setText(videoName)
               
        try:
            self.vid.capture=cv2.VideoCapture(videoPath)
            ret=self.vid.capture.open(videoPath)
            if ret==False: 
                print("False")
                QtWidgets.QMessageBox.warning(None,"Tracking",str("video error"),QtWidgets.QMessageBox.Ok)
        except cv2.error as e:
            print("error : ",e)
            QtWidgets.QMessageBox.warning(None,"Tracking",str(e),QtWidgets.QMessageBox.Ok)
        
        #vidProp_dict['imageType']=image.dtype
        #video.capture.set(5,200) #set fps
        self.vid.codec=self.vid.capture.get(cv2.CAP_PROP_FOURCC) #CV_CAP_PROP_FOURCC
        #The list of available codes can be found in fourcc.org.
        #It is platform dependent.
        # cv.FOURCC( *"XVID" )    1145656920
        # cv.FOURCC( *"MJPG" )    1196444237
        # cv.FOURCC( *"X264" )     875967064
        # cv.FOURCC( *"DIB " )     541215044
        # cv2.cv.FOURCC( *"WMV1" )     827739479
        # cv2.cv.FOURCC( *"WMV2" )     844516695
        self.vid.ToRGB=self.vid.capture.get(16) #vidProp_dict['videoToRGB']=cap.get(16)
        self.vid.width=self.vid.capture.get(3) #CV_CAP_PROP_FRAME_WIDTH
        self.vid.height=self.vid.capture.get(4) #CV_CAP_PROP_FRAME_HEIGHT     
        self.vid.fps=self.vid.capture.get(5) #CV_CAP_PROP_FPS
        self.vid.nbFrames=self.vid.capture.get(7) #CV_CAP_PROP_FRAME_COUNT
        self.numTotalFrame_label.setText(str(int(self.vid.nbFrames)))
        self.vid.LUT=self.image_processor.gamma_LUT(1.8)
        self.vid.firstFrame=1
        
        #affichage des propriétés de la vidéo
        print('codec : ', str(self.vid.codec))
        print('convert : '+str(self.vid.ToRGB))
        print('Frames= '+str(self.vid.nbFrames)+' at '+str(self.vid.fps)+'fps')
        print('frame W = ' +str(self.vid.width)+' - H = '+str(self.vid.height))
        
        #éditer le tooltip du label avec les caractéristiques de la vidéo
        self.update_toolTip(self.lb_videoName)
        self.loadVideoSignal.emit()
        #activation de l'interface
        self.activate_interface("load video")
        print("player activated")
        # afficher la video car sld_TimeLine n'est pas modifié
        self.play_frame(self.vid.firstFrame)
        #initialisation du lecteur, taille du buffer...
        self.init_player()
        
    
    def init_player(self) :
        #initialisation du slider timeline
        self.timeLine_slider.setMaximum(int(self.vid.nbFrames))
        self.timeLine_slider.setValue(int(self.vid.firstFrame))
        
        #calcul de la mémoire disponible
        svmem=psutil.virtual_memory().available/1024.0**2
        print("Available memory : ", svmem,"MB")
        # self.availableMemValue_label.setText(str(round(svmem,2)))
        #calculer la taille du buffer en fonction du poids de chaque frame
        fshape=self.vid.currentFrame.shape
        #taille d'un frame en MB
        self.vid.frameWeight=fshape[0]*fshape[1]*fshape[2]/1024/1024
        print("frame size (MB) : ",self.vid.frameWeight)
        
        bufferSizeMax=2/3*svmem
        #en nombre de frame
        nbFramesBufferMax=int(bufferSizeMax/self.vid.frameWeight)
        # self.bufferSizeValue_slider.setMaximum(nbFramesBufferMax)
        # self.bufferSizeValue_slider.setValue(nbFramesBufferMax)   
        #intancier le FileVideoStream_ToPlay pour la mise en buffer
        # video.grabber = FileVideoStream_ToPlay(video.capture,ui.bufferSizeValue_slider.value()) 
        self.vid.grabber = FileVideoStream_ToPlay(self.vid.capture,self.vid,nbFramesBufferMax)
        
        
    def update_toolTip(self,item):
        videoInfo='{0} \n size : {1}, {2} \n frames : {3} at {4} fps'.format(self.vid.path,int(self.vid.width),int(self.vid.height),int(self.vid.nbFrames),int(self.vid.fps))
        item.setToolTip(videoInfo)
    
    def playpause_video(self):
        # print("play video :",videoPlayer_Widget.playVideo_btn.isChecked())
        if self.vid.path==None:
            message="You have to open a video first"
            QtWidgets.QMessageBox.warning(None,"no video",str(message),QtWidgets.QMessageBox.Ok)    
                    
        else:
            if self.playVideo_btn.isChecked()==True:
                
                self.vid.playing=True
                self.playSignal.emit()
                self.splitter_playSpeed.setEnabled(False)
                #video_Play()
                #fvs = FileVideoStream_Play(video.file).start()
                self.vid.grabber.stopped = False
                
                #vider le buffer
                #video.stream.task_done()
                #video.grabber.Q.queue.clear()
                             
                self.vid.grabber.start(self.timeLine_slider.value(),self.framerateFactor)
                print("grabber started")
                time.sleep(1.0)
                # self.video_play_stream(self.vid.grabber)
                timer_interval = int(1000 / self.vid.fps)
                self.timer.start(timer_interval)
                
                # print("streamplayer stopped")
                # self.playVideo_btn.setChecked(False)
                # self.pause_video()
                
            else :
                self.pause_video()
                    
    
    def pause_video(self):
        print("pause_video")
        self.vid.playing=False
        self.vid.grabber.stop()
        self.vid.grabber.Q.queue.clear()
        # self.buffer_progress.setValue(int(self.vid.grabber.Q.qsize()/self.vid.grabber.Q.maxsize*100))

        self.playVideo_btn.setChecked(False)
        self.splitter_playSpeed.setEnabled(True)
        self.pauseSignal.emit()          
        
    def forward_video(self):
        #self.timeLine_slider.setValue(self.timeLine_slider.value()+1)
        self.timeLine_slider.setValue(self.timeLine_slider.value()+self.framerateFactor)
        track_display()
            
    def backward_video(self):
        #self.timeLine_slider.setValue(self.timeLine_slider.value()-1)
        self.timeLine_slider.setValue(self.timeLine_slider.value()-self.framerateFactor)
        track_display()
                
    def slider_move(self):
        if self.vid.capture != None :
            if self.playVideo_btn.isChecked()==False :
                self.play_frame(self.timeLine_slider.value())
                self.timeLineMoveSignal.emit(self.timeLine_slider.value())
    
    def apply_frameRotation(self):
        self.vid.rotation=self.videoRotation_comboBox.currentText()
        self.play_frame(self.timeLine_slider.value())
            
            
    def play_frame(self,iframe):
    
        #cv2.VideoCapture.set(propId, value) → retval
        #propId=1 : CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next
        retVal_set=False
        if self.vid.capture != None :
            retVal_set=self.vid.capture.set(1,iframe)
        if retVal_set==True : 
            (retVal_read, self.vid.currentFrame) = self.vid.capture.read()
            currentTime=self.vid.capture.get(cv2.CAP_PROP_POS_MSEC)
            # milliseconds = 86400001 # a day and a millisecond... for example
            seconds, milliseconds = divmod(currentTime, 1000)
            minutes, seconds = divmod(seconds, 60)
            # print(f'{int(minutes):02d}:{int(seconds):02d}.{int(milliseconds):03d}')
            label_time=f'{int(minutes):02d}:{int(seconds):02d}.{int(milliseconds):03d}'
            # 1440:00.001
        else :
            print("wrong frame index")
            retVal_read=False
        #print(video.capture.get(0),video.capture.get(1),iframe)
        #video.capture.get(1) : CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
        #str(cap.get(cv2.CAP_PROP_POS_MSEC)) time code in msec
        if retVal_read==True:
            if self.vid.rescale!=1 :
                self.vid.currentFrame=cv2.resize(self.vid.currentFrame, None, fx = self.vid.rescale, fy = self.vid.rescale,interpolation = cv2.INTER_CUBIC)

            if self.videoRotation_comboBox.currentText()!="No rotation" :
                self.vid.currentFrame=set_videoRotation(self.vid,self.vid.currentFrame)
            #conversion de l'image pour affichage dans le plotview        
            tFrame=self.image_processor.convert_imageToPyqtgraph(self.vid.currentFrame,self.vid)
            #cv2.imshow('Visu',image.currentFrame)        
            self.vidDisplay.img.setImage(tFrame,autoLevels=False)
            self.numFrame_label.setNum(iframe)
            self.currentTime_label.setText(label_time)
         
            #envoyer un signal pour l'affichage de l'historique des zones object-tracking
            self.analysisSignal.emit(iframe)
        else:
            print("can't read this frame")
            #TODO: ajouter un message d'erreur ?
    
    def video_play_framerate(self):
        # prendre une image dans le buffer si il n'est pas vide
        if self.vid.grabber.more():
            self.vid.currentFrame = self.vid.grabber.read()
            
        else :
            for wait in range(2):
                print("waiting for new frame...")
                time.sleep(1)
                if self.vid.grabber.more():
                    self.vid.currentFrame = self.vid.grabber.read()
                    break
            if self.vid.grabber.more()==False:
                print("no more frame")
                self.timer.stop()
        # self.buffer_progress.setValue(int(fvs.Q.qsize()/fvs.Q.maxsize*100))
        """                    
        # display the size of the queue on the frame
        cv2.putText(frame, "Queue Size: {}".format(fvs.Q.qsize()),
        	(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)	
        """ 
        
        # show the frame 
        self.vidDisplay.show_frame_in_pyqtgraph(self.vid.currentFrame)
        # tFrame=convert_imageToPyqtgraph(image.currentFrame)
        # ui.img.setImage(tFrame,autoLevels=False)
        iframe=self.timeLine_slider.value()
        # print("read :",iframe)
        
        #-----------------------------------------------------------------------------------------------ANALYSIS
           
        self.analysisSignal.emit(iframe)
        
        #----------------------------------------------------------------------------------------------
        self.timeLine_slider.setValue(self.timeLine_slider.value()+self.framerateFactor)  
        #update video time
        currentTime=(self.timeLine_slider.value()/self.vid.fps)*1000
        # milliseconds = 86400001 # a day and a millisecond... for example
        seconds, milliseconds = divmod(currentTime, 1000)
        minutes, seconds = divmod(seconds, 60)
        # print(f'{int(minutes):02d}:{int(seconds):02d}.{int(milliseconds):03d}')
        label_time=f'{int(minutes):02d}:{int(seconds):02d}.{int(milliseconds):03d}'
        self.currentTime_label.setText(label_time)

        # app.processEvents()  ## force complete redraw for every plot
        QtWidgets.QApplication.instance().processEvents()
        # print("fvs.more=",self.vid.grabber.more())
        
    def video_play_stream(self,fvs):
        """
        lecture des frames accumulés dans le tampon file video stream
        """
     
        print("video position : ", self.vid.firstFrame)
        print("video nb frames : ", self.vid.nbFrames)
       
        
        for i in range(int(self.vid.firstFrame),int(self.vid.nbFrames)) :
            """
            grab the frame from the threaded video file stream, resize
            it, and convert it to grayscale (while still retaining 3
            channels)
            """
            #condition d'arrêt
            if self.vid.playing==False:
                break
            
            # prendre une image dans le buffer si il n'est pas vide
            if fvs.more():
                self.vid.currentFrame = fvs.read()
                
            else :
                for wait in range(2):
                    print("waiting for new frame...")
                    time.sleep(1)
                    if fvs.more():
                        self.vid.currentFrame = fvs.read()
                        break
                if fvs.more()==False:
                    print("no more frame")
                    break
            # self.buffer_progress.setValue(int(fvs.Q.qsize()/fvs.Q.maxsize*100))
            """                    
            # display the size of the queue on the frame
            cv2.putText(frame, "Queue Size: {}".format(fvs.Q.qsize()),
            	(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)	
            """ 
            
            # show the frame 
            self.vidDisplay.show_frame_in_pyqtgraph(self.vid.currentFrame)
            # tFrame=convert_imageToPyqtgraph(image.currentFrame)
            # ui.img.setImage(tFrame,autoLevels=False)
            iframe=self.timeLine_slider.value()
            # print("read :",iframe)
            #-----------------------------------------------------------------------------------------------ANALYSIS
               
            self.analysisSignal.emit(iframe)
            
            #----------------------------------------------------------------------------------------------
            self.timeLine_slider.setValue(self.timeLine_slider.value()+self.framerateFactor)  
            
            # app.processEvents()  ## force complete redraw for every plot
            QtWidgets.QApplication.instance().processEvents()
        print("fvs.more=",fvs.more())
           
                
    
    def set_framerateFactor(self,i):
        self.framerateFactor = i
        # self.playFramerateSignal.emit(i)
        #print(self.framerateFactor)
    
    def set_videoScale(self,i):
        # print (self.vid.currentFrame)
        if self.vid.capture != None :
            self.vid.rescale=1/i
            iframe=self.timeLine_slider.value()
            self.play_frame(iframe)
        else :
            print("no video")
        
        
    def bufferSizeMB_update(self) :
        if self.vid.frameWeight>0:
            self.bufferSizeValMB_label.setText(str(round(self.bufferSizeValue_slider.value()*self.vid.frameWeight,2))+"MB")       

     
    def activate_interface(self,context):
         #rend accessible les boutons pour la lecture de la video
        if context=="load video":
            self.playVideo_btn.setEnabled(True)
            self.playVideo_btn.setChecked(False)
            self.timeLine_slider.setEnabled(True)
            self.stepFwdVideo_btn.setEnabled(True)
            self.stepBwdVideo_btn.setEnabled(True)   






class UIVideoDisplay(pg.GraphicsLayoutWidget):
    def __init__(self,video, parent=None):
        super(UIVideoDisplay,self).__init__()
        
        self.vid=video
        self.image_processor=ImageProcessor()
        
        #instance de la viewbox spécifique pour le dessin de roi intéractif
        self.gviewBox=pg.ViewBox()#gViewBox_InteractROI()
        #ajout d'un plotView avec comme viewbox la gviewBox
        self.plotView=self.addPlot(viewBox=self.gviewBox)
        ## lock the aspect ratio so pixels are always square
        self.plotView.setAspectLocked(True)
        
        ## Create image item
        self.img = pg.ImageItem()
        
        self.gviewBox.addItem(self.img)
                      
        # self.proxy1 = pg.SignalProxy(self.plotView.scene().sigMouseClicked, rateLimit=60, slot=mouse_clicked)
    def show_frame_in_pyqtgraph(self,cv2frame):
        """
        convert the cv2 frame to pyqtgraph format
        """
        tFrame=self.image_processor.convert_imageToPyqtgraph(cv2frame,self.vid)
        self.img.setImage(tFrame,autoLevels=False)

    def contextMenuEvent(self, event):
        # Create a context menu
        menu = QtWidgets.QMenu(self)

        # Add actions to the context menu
        load_action = QtWidgets.QAction('Load video', self)

        # Connect actions to their respective slots
        load_action.triggered.connect(self.load_video)

        # Add actions to the context menu
        menu.addAction(load_action)
    
        # Show the context menu at the cursor position
        menu.exec_(event.globalPos())
        
    def load_video(self):
        print("load video...")


### classe et méthodes nécessaires pour faire fonctionner le code en application principale

class PyqtgWindow(QtWidgets.QMainWindow):
    
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent=None)
        
        
        self.setWindowIcon(QIcon(r'Imagys_blue\logoAnimotion-square-112.png'))
        ## Switch to using white background and black foreground
        #pg.setConfigOption('background', 'w')
        #pg.setConfigOption('foreground', 'k')
        
#        self.win = QtWidgets.QMainWindow()
        self.area = DockArea()
        self.setCentralWidget(self.area)
        self.resize(1500,800)
        self.setWindowTitle('Video Player - beta')

        self.d1 = Dock("Image", size=(1000,500))
        self.d9 = Dock("Video player", size=(500,200))
        
        self.area.addDock(self.d1, 'left')
        self.area.addDock(self.d9, 'bottom', self.d1)
        
        
        ## Add widgets into each dock----------------------------------------------------------------------------VIDEO DISPLAY
        ## image/video widget
        # self.w1=pg.GraphicsLayoutWidget()
        # self.d1.addWidget(self.w1)
        
 #-----------------------------------------------------------------------------------------------------------dock9 - VIDEO PLAYER
        ## dock9 video player : 
#         self.w9 = pg.LayoutWidget()
        # self.d9.addWidget(videoPlayer_Widget)
        #-------------------------------------------------------------------------------------      
     
        # loads and sets the Qt stylesheet
        qss_path=resource_path('Imagys_blue\Imagys-blue.qss')
        qss = QSSHelper.open_qss(qss_path)
        self.setStyleSheet(qss)                   
        self.show()

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    if getattr(sys, 'frozen', False):  # Check if the app is frozen
        base_path = sys._MEIPASS
    else:
        base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))  # Go up one level
    return os.path.join(base_path, relative_path) 


def clean_app():
    print("")
    print("quit app ...")
    #TODO: close resultFile (resultFile.close())    
    #varA.quitApp=True
    try :
        print("close stream")
        if videoPlayer_Widget.vid.capture!=None :
            videoPlayer_Widget.vid.capture.release()
    except ValueError as e :
        QtWidgets.QMessageBox.warning(None,"something went wrong",str(e),QtWidgets.QMessageBox.Ok)
        print("no stream was opened")
    
    print("close cv2 windows")
    cv2.destroyAllWindows()
   
        


if __name__ == '__main__':
    
    # setting path
    # sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    # sys.path.append('D:\Developpements\Python\Freezing\git_freezing\Imagys_blue')
    qss_path=resource_path('Imagys_blue')
    sys.path.append(qss_path)
    # importing
    from qsshelper import QSSHelper
    # qss = QSSHelper.open_qss('Imagys_blue\Imagys-blue.qss')
    app = QtWidgets.QApplication([])
    # app.setStyleSheet(qss)
    # app.setStyleSheet(Path('D:\Developpements\Python\Freezing\git_freezing\Imagys_blue\Imagys-blue.qss').read_text())

      
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # qss_file_path = os.path.join(current_dir,'Imagys_blue', 'Imagys-blue.qss')
    # app.setStyleSheet(Path(qss_file_path).read_text())
    
    video=Video()
    
    # icon_path=os.path.join(current_dir,'Imagys_blue')
    icon_path=os.path.join("Imagys_blue","freezed_mouse-4-300x400.jpg")
    icon = resource_path(icon_path)
    videoDisplay_Widget=UIVideoDisplay(video)
    videoPlayer_Widget=UIVideoPlayer(videoDisplay_Widget,video,icon) 
    #interface utilisateur
    ui=PyqtgWindow()
    ui.d1.addWidget(videoDisplay_Widget)
    ui.d9.addWidget(videoPlayer_Widget)
    
    """
    controle des commandes avec le clavier
    """
    keyboard.add_hotkey('right', lambda: videoPlayer_Widget.forward_video())
    keyboard.add_hotkey('left', lambda: videoPlayer_Widget.backward_video())

   
    
    app.exec_()
    clean_app()
    print("the end")
    app.quit()