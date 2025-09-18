# -*- coding: utf-8 -*-
"""
Created on Mon Mar  7 08:52:52 2022

@author: courtand

v3c : use multiple buffers : to analyze / to display
"""


import os
import sys
import time

import psutil

# basler
from pypylon import pylon
from pypylon import genicam

from PyQt5 import QtWidgets,QtCore,QtGui
from PyQt5.QtCore import pyqtSignal, QTimer

import pyqtgraph as pg
# import pyqtgraph.ptime as ptime
from pyqtgraph.Qt import QtGui as QtgGui
from pyqtgraph.dockarea import *

from Imagys_blue.qsshelper import QSSHelper

import numpy as np
# import cv2
import math

from threading import Thread, Lock
# import the Queue class from Python 3
import queue

from collections import deque    

import video_player_6 as vp
import image_processor
import analysis_2 as analysis


class SimpleFrameBuffer:
    """
    Buffer ultra-simple avec deux queues séparées
    """
    
    def __init__(self, analysis_maxsize, display_maxsize=5):
        # Buffer pour l'analyse - FIFO simple
        self.analysis_frames = deque(maxlen=analysis_maxsize) #decrease for weak computer = 10
        self.analysis_lock = Lock()
        
        # Buffer pour l'affichage - FIFO simple  
        self.display_frames = deque(maxlen=display_maxsize)
        self.display_lock = Lock()
        
        # Statistiques
        self.total_frames = 0
        self.dropped_analysis = 0
        self.dropped_display = 0
    
    def add_frame(self, frame_data, for_analysis=True, for_display=False):
        """Ajouter une frame aux buffers appropriés"""
        self.total_frames += 1
        
        # Ajouter au buffer d'analyse
        if for_analysis:
            with self.analysis_lock:
                if len(self.analysis_frames) >= self.analysis_frames.maxlen:
                    self.dropped_analysis += 1
                    # print("frame dropped")
                
                # Copie pour analyse
                analysis_frame = {
                    'image': frame_data['image'].copy(),
                    'timestamp': frame_data['timestamp'],
                    'frame_id': frame_data['frame_id']
                }
                self.analysis_frames.append(analysis_frame)
        
        # # Ajouter au buffer d'affichage
        # if for_display:
        #     with self.display_lock:
        #         if len(self.display_frames) >= self.display_frames.maxlen:
        #             self.dropped_display += 1
                
        #         # Copie pour affichage
        #         display_frame = {
        #             'image': frame_data['image'].copy(),
        #             'timestamp': frame_data['timestamp'],
        #             'frame_id': frame_data['frame_id']
        #         }
        #         self.display_frames.append(display_frame)
    def get_frame_for_analysis(self):
        """Récupérer UNE frame pour analyse (sans la supprimer pour éviter les problèmes)"""
        with self.analysis_lock:
            if self.analysis_frames:
                return self.analysis_frames[-1]  # Prendre la plus récente
                # return self.analysis_frames.pop()
            return None
    
    # def get_frame_for_display(self):
    #     """Récupérer une frame pour affichage"""
    #     with self.display_lock:
    #         if self.display_frames:
    #             return self.display_frames.popleft()  # FIFO pour affichage
    #         return None
    
    def get_analysis_buffer_size(self):
        with self.analysis_lock:
            return len(self.analysis_frames)
    
    def get_display_buffer_size(self):
        with self.display_lock:
            return len(self.display_frames)
    
    def get_stats(self):
        return {
            'analysis_size': self.get_analysis_buffer_size(),
            'display_size': self.get_display_buffer_size(),
            'total_frames': self.total_frames,
            'dropped_analysis': self.dropped_analysis,
            'dropped_display': self.dropped_display
        }

class FileVideoStreamLive(Thread):
    """Thread d'acquisition"""
    
    def __init__(self, video,queueSize=500):
        super().__init__()
        self.daemon = True
        
        self.camera = video.device
        self.video=video
        self.framerate=self.video.device.ResultingFrameRate.GetValue()
        
        self.lastTime_live=0
        
        # Contrôles
        self.running = False
        self.frame_count = 0
        self.acquisition_fps = 0
        
        # Statistiques
        self.last_fps_time = time.perf_counter()#  time.time()
        self.fps_counter = 0
        
        self.camera.Open()
        # Print the index and the model name of the camera.
        #print("Camera ", cameraContextValue, ": ", cameras[cameraContextValue].GetDeviceInfo().GetModelName())
        
        #--------------------------------------------------------------------
        # Récupération de la taille des images pour pré-allocation
        self.camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
        grab_result = self.camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
        if grab_result.GrabSucceeded():
            frame_shape = grab_result.Array.shape
            grab_result.Release()
        else:
            frame_shape = (1024, 1280)  # Valeur par défaut
        self.camera.StopGrabbing()
        #---------------------------------------------------------------------
         
        # Buffer simple avec queues séparées
        self.frame_buffer = SimpleFrameBuffer(analysis_maxsize=5, display_maxsize=2)
        
        # Queue simple pour affichage (backup)
        self.display_queue = queue.Queue(maxsize=5)
        # augmenter la taille du buffer si le pc est moins rapide (!?)
        
        # Buffer temporaire
        self._temp_frame = np.zeros(frame_shape, dtype=np.uint8)


    def run(self):
        """Main acquisition loop"""
        # keep looping infinitely
        display_freq = max(1, int(self.framerate / 25))
        
        self.camera.StartGrabbing(pylon.GrabStrategy_OneByOne) #default strategy
        self.running = True
        
        while self.running : #self.stream.IsGrabbing() :
            try:
                grab_result = self.camera.RetrieveResult(50, pylon.TimeoutHandling_ThrowException)
                # retrieve the result using a timeout of 100 milliseconds : if 2 seconds pass and there is no result, it would simply return
                # Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        
                # When the cameras in the array are created 
                # the camera context value is set to the index of the camera in the array.
                # The camera context is a user settable value.
                # This value is attached to each grab result and can be used
                # to determine the camera that produced the grab result.
                #   cameraContextValue = grabResult.GetCameraContext()
        
                if grab_result.GrabSucceeded():
                    # Copie directe sans allocation supplémentaire
                    np.copyto(self._temp_frame, grab_result.Array)
                    timestamp = time.perf_counter()#time.time()
                
                    # Données de la frame
                    frame_data = {
                        'image': self._temp_frame,
                        'timestamp': timestamp,
                        'frame_id': self.frame_count
                    }
                    
                    # Décider si cette frame va à l'affichage
                    for_display = (self.frame_count % display_freq == 0)
                    
                    # SIMPLE: Ajouter au buffer
                    self.frame_buffer.add_frame(
                        frame_data, 
                        for_analysis=True,  # Toutes les frames pour analyse
                        # for_display=for_display  # Sous-échantillonnage pour affichage
                    )
                    
                    # BACKUP: Ajouter aussi à la queue d'affichage pour compatibilité
                    if for_display:
                        try:
                            with self.frame_buffer.display_lock:
                                # Vider la queue si pleine
                                while self.display_queue.qsize() >= 1:
                                    try:
                                        self.display_queue.get_nowait()
                                    except queue.Empty:
                                        break
                                
                                display_data = {
                                    'image': self._temp_frame.copy(),
                                    'timestamp': timestamp,
                                    'frame_id': self.frame_count
                                }
                                self.display_queue.put_nowait(display_data)
                            
                        except queue.Full:
                            pass
                    
                    self.frame_count += 1
                    self.fps_counter += 1
                    
                    # Calcul framerate
                    now = time.time()
                    dt = now - self.lastTime_live + 0.0000001
                    self.lastTime_live = now
                    if self.video.measuredLivefps is None:
                        self.video.measuredLivefps = 1.0/dt
                    else:
                        s = np.clip(dt*3., 0, 1)
                        self.video.measuredLivefps = self.video.measuredLivefps * (1-s) + (1.0/dt) * s
                
                grab_result.Release()
                
                # Pause minimale
                time.sleep(0.001)
            
            except Exception as e:
                print(f"Erreur acquisition: {e}")
                time.sleep(0.01)            
        
        self.camera.StopGrabbing()
        print("Acquisition arrêtée")   

    def get_frame_for_analysis(self):
        """Interface pour le thread d'analyse"""
        return self.frame_buffer.get_frame_for_analysis()
    
    def get_buffer_usage(self):
        """Pourcentage d'utilisation du buffer"""
        analysis_size = self.frame_buffer.get_analysis_buffer_size()
        return (analysis_size / 50) * 100  # Sur 30 max
    
    def get_buffer_stats(self):
        """Statistiques détaillées"""
        return self.frame_buffer.get_stats()
    
    def stop(self):
        """Arrêt de l'acquisition"""
        self.running = False


class UIVideoCapture(pg.LayoutWidget):
    """Display interface"""
    
    # analysisSignal=pyqtSignal(np.ndarray)
    # pauseSignal=pyqtSignal()
    # updatePlotSignal=pyqtSignal()
    # updateTrackMapSignal=pyqtSignal()
    
    # testSignal=pyqtSignal(int)
    
    
    def __init__(self,videoDisplay_Widget,video,display_updater,parent=None):
        super().__init__()#(UIVideoCapture,self).__init__()
        
        self.acquisition_thread=None
        self.video=video
        self.videoDisplayer=videoDisplay_Widget
        self.videoDisplayer_updater=display_updater
        
        self.frame_count = 0
        self.fps_counter = 0 
        
        self.setup_ui()
        
        
        # self.live_timer = QtCore.QTimer()
        # self.live_timer.timeout.connect(self.refresh_live_display)
        
    def setup_ui(self):
        """user interface Configuration"""
        #connexion camera équivalent bouton load_video dans le player
        self.connectCam_btn = QtWidgets.QPushButton('Connect camera')
        self.connectCam_btn.setMaximumWidth(150)
        self.connectCam_btn.setMinimumWidth(100)
        self.connectCam_btn.setCheckable(True)
        self.connectCam_btn.setChecked(False)
        self.connectCam_btn.clicked.connect(self.connect_disconnect)
        
        self.videoName_label= QtWidgets.QLabel("No camera connected")
        self.videoName_label.setMaximumWidth(150)
        
        splitter_connect= QtWidgets.QSplitter()
        splitter_connect.setOrientation(QtCore.Qt.Horizontal)
        splitter_connect.setMaximumWidth(150)
        splitter_connect.addWidget(self.connectCam_btn)
        splitter_connect.addWidget(self.videoName_label)        
         
        saveConfig_btn = QtWidgets.QPushButton('Save config')
        saveConfig_btn.clicked.connect(self.save_config)
        loadConfig_btn = QtWidgets.QPushButton('Load config')
        loadConfig_btn.clicked.connect(self.load_config)
        
        
        #affichage de la video acquise par la camera live : équivalent bouton play du player
        self.liveVideo_btn = QtWidgets.QPushButton('Live')
        self.liveVideo_btn.setCheckable(True)
        self.liveVideo_btn.setChecked(False)
        self.liveVideo_btn.setMaximumWidth(150)
        self.liveVideo_btn.setMinimumWidth(100)
        self.liveVideo_btn.setEnabled(False)
        self.liveVideo_btn.clicked.connect(self.start_stop_acquisition_toggle)
        
        self.measuredLivefps_Label=QtWidgets.QLabel(""" 00 fps """)
        self.measuredLivefps_Label.setMaximumWidth(40)
        self.measuredLivefps_Label.setMinimumWidth(40)
        
        splitter_live= QtWidgets.QSplitter()
        splitter_live.setOrientation(QtCore.Qt.Horizontal)
        splitter_live.setMaximumWidth(150)
        splitter_live.addWidget(self.liveVideo_btn)
        splitter_live.addWidget(self.measuredLivefps_Label)        
        
        self.trigger_checkBox = QtWidgets.QCheckBox("Trigger")
        self.trigger_checkBox.setMaximumWidth(150)

        availableMemory_label= QtWidgets.QLabel("Available memory (MB) : ")
        availableMemory_label.setMinimumWidth(100)
        availableMemory_label.setMaximumWidth(120)
        self.availableMemValue_label= QtWidgets.QLabel("00")
        # self.availableMemValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.availableMemValue_label.setMaximumWidth(100)
        self.availableMemValue_label.setMinimumWidth(40)

        bufferSize_label= QtWidgets.QLabel("Buffer size (frames) ")
        bufferSize_label.setMinimumWidth(100)
        bufferSize_label.setMaximumWidth(120)
        self.bufferSizeValue_label= QtWidgets.QLabel("000")
        self.bufferSizeValue_label.setMinimumWidth(20)
        self.bufferSizeValue_label.setMaximumWidth(40)
        self.bufferSizeValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.bufferSizeValMB_label= QtWidgets.QLabel("00MB")
        self.bufferSizeValMB_label.setMinimumWidth(20)
        self.bufferSizeValMB_label.setMaximumWidth(40)
        self.bufferSizeValMB_label.setAlignment(QtCore.Qt.AlignCenter)
  
        self.bufferSizeFrames_spinbox = QtWidgets.QSpinBox()
        self.bufferSizeFrames_spinbox.setMaximumWidth(80)
        self.bufferSizeFrames_spinbox.setMinimum(1)
        self.bufferSizeFrames_spinbox.setMaximum(2000)
        self.bufferSizeFrames_spinbox.setValue(10)
        self.bufferSizeFrames_spinbox.valueChanged.connect(self.bufferSizeMB_update)
                
        self.buffer_progress=QtWidgets.QProgressBar()
        self.buffer_progress.setMinimumWidth(100)
        self.buffer_progress.setMaximumWidth(200)
        self.buffer_progress.setMaximumHeight(6)
        self.buffer_progress.setTextVisible(False)
        
        self.lenBuffer_label = QtWidgets.QLabel("00")
        self.lenBuffer_label.setMinimumWidth(20)
        self.lenBuffer_label.setMaximumWidth(40)
        self.lenBuffer_label.setAlignment(QtCore.Qt.AlignCenter)
        
        splitterMemory = QtWidgets.QSplitter()
        splitterMemory.setOrientation(QtCore.Qt.Horizontal)
        splitterMemory.setMaximumWidth(150)
        splitterMemory.addWidget(availableMemory_label)
        splitterMemory.addWidget(self.availableMemValue_label)
        
        splitterBuffer = QtWidgets.QSplitter()
        splitterBuffer.setOrientation(QtCore.Qt.Horizontal)
        splitterBuffer.setMaximumWidth(150)
        splitterBuffer.addWidget(bufferSize_label)
        splitterBuffer.addWidget(self.bufferSizeFrames_spinbox)
        splitterBuffer.addWidget(self.bufferSizeValMB_label)
        # splitterBuffer.addWidget(bufferSizeMB_label)
        
        splitterBufProgress = QtWidgets.QSplitter()
        splitterBufProgress.setOrientation(QtCore.Qt.Horizontal)
        splitterBufProgress.setMaximumWidth(150)
        splitterBufProgress.addWidget(self.buffer_progress)
        splitterBufProgress.addWidget(self.lenBuffer_label)
        splitterBufProgress.setEnabled(False)
        
        self.line1 = QtWidgets.QFrame()
        # self.line.setGeometry(QtCore.QRect(60, 110, 751, 20))
        self.line1.width=20
        self.line1.midLineWidth=10
        self.line1.minimumWidth=10
        self.line1.setMaximumWidth(20)
        self.line1.setFrameShape(QtWidgets.QFrame.VLine)
        # self.line1.setFrameShadow(QtWidgets.QFrame.Sunken)
        
        self.line2 = QtWidgets.QFrame()
        # self.line2.setGeometry(QtCore.QRect(60, 110, 751, 20))
        self.line2.width=20
        self.line2.midLineWidth=10
        self.line2.minimumWidth=10
        self.line2.setMaximumWidth(20)
        self.line2.setFrameShape(QtWidgets.QFrame.VLine)
        # self.line2.setFrameShadow(QtWidgets.QFrame.Sunken)

    #----------------------------------------------------------camera parameters
    
        self.pixFormat_label=QtWidgets.QLabel("pixel format")
        # self.pixFormat_label.setAlignment(QtCore.Qt.AlignCenter)
        self.pixFormat_label.setMinimumWidth(30)
        self.pixFormat_label.setMaximumWidth(200)
        # self.pixFormat_label.setMaximumHeight(20)
        self.pixFormat_comboBox = QtWidgets.QComboBox()
        self.pixFormat_comboBox.setMaximumWidth(120)
        self.pixFormat_comboBox.addItem("Mono8")
        self.pixFormat_comboBox.addItem("BGR8")
        self.pixFormat_comboBox.currentTextChanged.connect(self.cam_change_pixelFormat)
        
        self.sensorReadMode_label=QtWidgets.QLabel("sensor read out mode")
        # self.pixFormat_label.setAlignment(QtCore.Qt.AlignCenter)
        self.sensorReadMode_label.setMinimumWidth(30)
        self.sensorReadMode_label.setMaximumWidth(200)
        # self.pixFormat_label.setMaximumHeight(20)
        self.sensorReadMode_comboBox = QtWidgets.QComboBox()
        self.sensorReadMode_comboBox.setMinimumWidth(100)
        self.sensorReadMode_comboBox.setMaximumWidth(120)
        self.sensorReadMode_comboBox.addItem("Normal")
        self.sensorReadMode_comboBox.addItem("Fast")
        self.sensorReadMode_comboBox.currentTextChanged.connect(self.cam_change_sensorReadMode)
        
        self.frameWidth_label = QtWidgets.QLabel("frame width")
        self.frameWidth_label.setMinimumWidth(30)
        self.frameWidth_label.setMaximumWidth(200)
        self.frameWidth_slider = StepSlider(32,1952,32)
        self.frameWidth_slider.setOrientation(QtCore.Qt.Horizontal)
        # self.frameWidth_slider.setTickInterval(32)
        # self.frameWidth_slider.setSingleStep(32) #arrow keys
        # self.frameWidth_slider.setPageStep(96) #pageup/down key
        self.frameWidth_slider.setMinimumWidth(30)
        self.frameWidth_slider.setMaximumWidth(200)
        # self.frameWidth_slider.setMaximum(1932)
        # self.frameWidth_slider.setMinimum(32)
        # self.frameWidth_slider.setInterval(32)
        # self.frameWidth_slider.setValue(1920)
        self.frameWidthValue_label = QtWidgets.QLabel("00")
        self.frameWidthValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.frameWidthValue_label.setMaximumWidth(60)
        self.frameWidth_slider.valueChanged.connect(self.width_value_by_step)
        
        splitterFrameWidth = QtWidgets.QSplitter()
        splitterFrameWidth.setOrientation(QtCore.Qt.Horizontal)
        splitterFrameWidth.setMaximumWidth(150)
        splitterFrameWidth.addWidget(self.frameWidthValue_label)
        splitterFrameWidth.addWidget(self.frameWidth_slider)
        
        self.frameHeight_label = QtWidgets.QLabel("frame height")
        self.frameHeight_label.setMinimumWidth(30)
        self.frameHeight_label.setMaximumWidth(200)
        self.frameHeight_slider = StepSlider(2,1232,2)
        self.frameHeight_slider.setOrientation(QtCore.Qt.Horizontal)
        self.frameHeight_slider.setMinimumWidth(30)
        self.frameHeight_slider.setMaximumWidth(200)
        # self.frameHeight_slider.setMaximum(1232)
        # self.frameHeight_slider.setMinimum(2)
        # self.frameHeight_slider.setInterval(2)
        # self.frameHeight_slider.setValue(1200)
        self.frameHeightValue_label = QtWidgets.QLabel("00")
        self.frameHeightValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.frameHeightValue_label.setMaximumWidth(60)
        self.frameHeight_slider.valueChanged.connect(self.height_value_by_step)
        
        splitterFrameHeight = QtWidgets.QSplitter()
        splitterFrameHeight.setOrientation(QtCore.Qt.Horizontal)
        splitterFrameHeight.setMaximumWidth(150)
        splitterFrameHeight.addWidget(self.frameHeightValue_label)
        splitterFrameHeight.addWidget(self.frameHeight_slider)
        
        self.resultFPS_label= QtWidgets.QLabel("resulting frame rate")
        self.resultFPS_label.setMaximumWidth(100)
        self.resultFPSValue_label= QtWidgets.QLabel("00")
        self.resultFPSValue_label.setMinimumWidth(30)
        self.resultFPSValue_label.setMaximumWidth(100)
        self.frameRate_label= QtWidgets.QLabel("frame rate :")
        self.frameRate_text = QtWidgets.QLineEdit()
        self.frameRate_text.setMinimumWidth(30)
        self.frameRate_text.setMaximumWidth(60)
        self.frameRate_text.returnPressed.connect(self.cam_change_framerate)
        
        splitterFrameRate = QtWidgets.QSplitter()
        splitterFrameRate.setOrientation(QtCore.Qt.Horizontal)
        splitterFrameRate.setMaximumWidth(150)
        splitterFrameRate.addWidget(self.frameRate_text)
        splitterFrameRate.addWidget(self.resultFPS_label)
        splitterFrameRate.addWidget(self.resultFPSValue_label)
        
        self.exposureTime_label= QtWidgets.QLabel("exposure time :")
        self.exposureTime_text = QtWidgets.QLineEdit()
        self.exposureTime_text.setMinimumWidth(30)
        self.exposureTime_text.setMaximumWidth(60)
        self.exposureTime_text.returnPressed.connect(self.cam_change_exposureTime)

        self.offsetX_label = QtWidgets.QLabel("offset x")
        self.offsetX_label.setMinimumWidth(30)
        self.offsetX_label.setMaximumWidth(200)
        self.offsetX_slider = StepSlider(0,96,32)
        self.offsetX_slider.setOrientation(QtCore.Qt.Horizontal)
        self.offsetX_slider.setMinimumWidth(30)
        self.offsetX_slider.setMaximumWidth(200)
        # self.offsetX_slider.setMaximum(96)
        # self.offsetX_slider.setMinimum(0)
        # self.offsetX_slider.setInterval(32)
        # self.offsetX_slider.setValue(0)
        self.offsetXValue_label = QtWidgets.QLabel("00")
        self.offsetXValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.offsetXValue_label.setMaximumWidth(60)
        self.offsetX_slider.valueChanged.connect(self.offsetX_value_by_step)
        
        splitterOffsetX = QtWidgets.QSplitter()
        splitterOffsetX.setOrientation(QtCore.Qt.Horizontal)
        splitterOffsetX.setMaximumWidth(150)
        splitterOffsetX.addWidget(self.offsetXValue_label)
        splitterOffsetX.addWidget(self.offsetX_slider)
       
        self.offsetY_label = QtWidgets.QLabel("offset y")
        self.offsetY_label.setMinimumWidth(30)
        self.offsetY_label.setMaximumWidth(200)
        self.offsetY_slider = StepSlider(0,64,2)
        self.offsetY_slider.setOrientation(QtCore.Qt.Horizontal)
        self.offsetY_slider.setMinimumWidth(30)
        self.offsetY_slider.setMaximumWidth(200)
        # self.offsetY_slider.setMaximum(96)
        # self.offsetY_slider.setMinimum(0)
        # self.offsetY_slider.setInterval(2)
        # self.offsetY_slider.setValue(0)
        self.offsetYValue_label = QtWidgets.QLabel("00")
        self.offsetYValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.offsetYValue_label.setMaximumWidth(60)
        self.offsetY_slider.valueChanged.connect(self.offsetY_value_by_step)

        splitterOffsetY = QtWidgets.QSplitter()
        splitterOffsetY.setOrientation(QtCore.Qt.Horizontal)
        splitterOffsetY.setMaximumWidth(150)
        splitterOffsetY.addWidget(self.offsetYValue_label)
        splitterOffsetY.addWidget(self.offsetY_slider)
                   
        
        #--------------------------------------------------------------------------
        self.addWidget(splitter_connect,row=0,col=0)
        # self.addWidget(self.connectCam_btn,row=0,col=0)
        # self.addWidget(self.videoName_label,row=0,col=1)
        self.addWidget(loadConfig_btn, row=0, col=6)
        self.addWidget(saveConfig_btn, row=0, col=7)
        self.addWidget(splitter_live,row=2,col=0)
        self.addWidget(self.trigger_checkBox, row=3,col=0)
        # self.addWidget(self.lenBuffer_label,row=4,col=0)
        self.addWidget(splitterMemory,row=4,col=0)
        self.addWidget(splitterBuffer,row=5,col=0)
        self.addWidget(splitterBufProgress,row=6,col=0)
        self.addWidget(self.line1, row=1,col=1,rowspan=5)
        
        self.addWidget(self.pixFormat_label,row=1,col=2)
        self.addWidget(self.pixFormat_comboBox,row=1,col=3)
        self.addWidget(self.sensorReadMode_label,row=2,col=2)
        self.addWidget(self.sensorReadMode_comboBox,row=2,col=3)
       
       
        self.addWidget(self.frameRate_label,row=4,col=2)
        self.addWidget(splitterFrameRate,row=4,col=3)
        # self.addWidget(self.frameRate_text,row=4,col=3)
        # self.addWidget(self.resultFPS_label,row=4,col=4)
        # self.addWidget(self.resultFPSValue_label,row=4,col=5)
        self.addWidget(self.exposureTime_label,row=5,col=2)
        self.addWidget(self.exposureTime_text,row=5,col=3)
        
        self.addWidget(self.line2, row=1,col=4,rowspan=5)
        
        self.addWidget(self.frameWidth_label,row=2,col=6)
        self.addWidget(splitterFrameWidth,row=2,col=7)
        self.addWidget(self.frameHeight_label,row=3,col=6)
        self.addWidget(splitterFrameHeight,row=3,col=7)
        self.addWidget(self.offsetX_label,row=4,col=6)
        self.addWidget(splitterOffsetX,row=4,col=7)
        self.addWidget(self.offsetY_label,row=5,col=6)
        self.addWidget(splitterOffsetY,row=5,col=7)
        
          
    
    def width_value_by_step(self):
        self.cam_change()
        self.frameWidthValue_label.setNum(self.frameWidth_slider.value())
        self.video.device.Width.SetValue(self.frameWidth_slider.value())
        #update result fps value
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
        #update offsetX min and max
        self.offsetX_slider.setMaximum(self.video.device.OffsetX.Max)
        self.offsetX_slider.setMinimum(self.video.device.OffsetX.Min)
        
    def height_value_by_step(self):
        self.cam_change()
        self.frameHeightValue_label.setNum(self.frameHeight_slider.value())
        self.video.device.Height.SetValue(self.frameHeight_slider.value())
        #update result fps value
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
        #update offsetX min and max
        self.offsetY_slider.setMaximum(self.video.device.OffsetY.Max)
        self.offsetY_slider.setMinimum(self.video.device.OffsetY.Min)
        
    def offsetX_value_by_step(self):
        # if genicam.IsWritable(camera.OffsetX):
        self.offsetXValue_label.setNum(self.offsetX_slider.value())
        self.video.device.OffsetX.SetValue(self.offsetX_slider.value())
        
    def offsetY_value_by_step(self):
        # if genicam.IsWritable(camera.OffsetY):
        self.offsetYValue_label.setNum(self.offsetY_slider.value())
        self.video.device.OffsetY.SetValue(self.offsetY_slider.value())
    
    def cam_change_sensorReadMode(self,value):
        self.cam_change()
        self.video.device.SensorReadoutMode.SetValue(value)
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
    
    def cam_change_pixelFormat(self,value):
        self.cam_change()
        self.video.device.PixelFormat.SetValue(value)
        #update result fps value
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
        
    def cam_change_framerate(self):
        # print(self.frameRate_text.text())
        framerate=int(self.frameRate_text.text())
        self.video.device.AcquisitionFrameRate.SetValue(framerate)
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
        
    def cam_change_exposureTime(self):
        exposureTime=int(self.exposureTime_text.text())
        try:
            self.video.device.ExposureTime.SetValue(exposureTime)
            self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
        except genicam.GenericException as e:
            # Error handling.
            print("An exception occurred.", str(e))#.GetDescription())
            QtWidgets.QMessageBox.warning(None,"Error",str(e),QtWidgets.QMessageBox.Ok)

    def cam_change(self):
        # if self.video.device.IsGrabbing():
        if self.video.device.IsGrabbing():
            self.stop_grabbing()

    def update(self,cam):
        print("update cam parameters")
        self.pixFormat_comboBox.setCurrentText(cam.PixelFormat.GetValue())
        self.sensorReadMode_comboBox.setCurrentText(cam.SensorReadoutMode.GetValue())
        self.frameWidth_slider.setMaximum(cam.Width.Max)
        self.frameWidth_slider.setMinimum(cam.Width.Min)
        self.frameWidth_slider.setInterval(cam.Width.Inc)
        self.frameWidth_slider.setValue(self.video.width)
        self.frameWidthValue_label.setNum(self.video.width) 
        
        self.frameHeight_slider.setMaximum(cam.Height.Max)
        self.frameHeight_slider.setMinimum(cam.Height.Min)
        self.frameHeight_slider.setInterval(cam.Height.Inc)
        self.frameHeight_slider.setValue(self.video.height)
        self.frameHeightValue_label.setNum(self.video.height) 
        
        self.exposureTime_text.setText(str(int(cam.ExposureTime.GetValue())))
        
        self.offsetX_slider.setMaximum(cam.OffsetX.Max)
        self.offsetX_slider.setMinimum(cam.OffsetX.Min)
        self.offsetX_slider.setInterval(cam.OffsetX.Inc)
        self.offsetX_slider.setValue(cam.OffsetX.GetValue())
        self.offsetXValue_label.setNum(cam.OffsetX.GetValue())
        
        self.offsetY_slider.setMaximum(cam.OffsetY.Max)
        self.offsetY_slider.setMinimum(cam.OffsetY.Min)
        self.offsetY_slider.setInterval(cam.OffsetY.Inc)
        self.offsetY_slider.setValue(cam.OffsetY.GetValue())
        self.offsetYValue_label.setNum(cam.OffsetY.GetValue())
        
        self.resultFPSValue_label.setNum(cam.ResultingFrameRate.GetValue())
        self.frameRate_text.setText(str(int(cam.AcquisitionFrameRate.GetValue())))
        
    #-----------------------------------------------------------------------
        
   
        
    def connect_disconnect(self):
        if self.connectCam_btn.isChecked()==True:
            self.init_acquisitionDevice()
            self.connectCam_btn.setText("disconnect camera")
        else:
            self.disconnect_cam()
            self.connectCam_btn.setText("connect camera")
   
    # def init_video(self,videoPath):
    def init_acquisitionDevice(self):
        print("connecting device...")
        try :
            # #>>>available_cameras : [<DeviceInfo Basler acA2040-90um (xxxxxxx)>]
            # #video.devices = pypylon.factory.find_devices()
            # # Get all attached devices and exit application if no device is found.
            # # Get the transport layer factory.
           
           self.video.devices = pylon.TlFactory.GetInstance().EnumerateDevices()
           print(f"devices: {self.video.devices}")
           # Only look for cameras supported by Camera_t
           info = pylon.DeviceInfo()
           info.SetDeviceClass("BaslerUsb")
          
           self.video.device = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
           print("Connected device : ", self.video.device.GetDeviceInfo().GetModelName())
           self.videoName_label.setText(self.video.device.GetDeviceInfo().GetModelName())
            
           #connexion avec la camera
           self.video.device.Open()
           #paramètrage par défaut de certaines valeurs de la camera
           self.set_default_cam_parameters(self.video.device)
           #récupération des propriétés de la vidéo/camera
           self.get_cam_parameters(self.video.device)
           #chargement des valeurs dans l'interface de réglage
           self.update(self.video.device)
           
           self.activate_interface("live video")
           
        except genicam.GenericException as e:
            # Error handling.
            print("An exception occurred.", str(e))#.GetDescription())
            QtWidgets.QMessageBox.warning(None,"Error",str(e),QtWidgets.QMessageBox.Ok)      
   
    def get_cam_parameters(self,cam):
        """
        récupérer les valeurs des paramètres d'acquisition stockés dans la caméra
        """
        #TODO : if cam == basler Ace
        #// Get the parameters for setting the image area of interest (Image AOI).
        #const CIntegerPtr width = control.GetNode("Width");
        print("Saving camera's node map to file...")
        self.video.featuresFile = "NodeMap.pfs"
        print(self.video.featuresFile)
        # Save the content of the camera's node map into the file.
        pylon.FeaturePersistence.Save(self.video.featuresFile, cam.GetNodeMap())
        # # read the content of the file back to the camera's node map with enabled validation.
        # print("Reading file back to camera's node map...")
        # pylon.FeaturePersistence.Load(nodeFile, camera.GetNodeMap(), True)
        # width = cam.GetNode("Width")
        
        self.video.pixFormat=cam.PixelFormat.GetValue()
        self.video.width=cam.Width.GetValue()
        self.video.height=cam.Height.GetValue()
        self.video.grabFrameRate=cam.AcquisitionFrameRate.GetValue()
        self.video.offsetX=cam.OffsetX.GetValue()
        self.video.offsetY=cam.OffsetY.GetValue()
        self.video.centerX=cam.CenterX.GetValue()
        self.video.centerY=cam.CenterY.GetValue()
        self.video.gainAuto=cam.GainAuto.GetValue()
        self.video.gain=cam.Gain.GetValue()
        self.video.gamma=cam.Gamma.GetValue()
        self.video.LUT=image_processor.gamma_LUT(1)
        self.video.exposure=cam.ExposureTime.GetValue()
        self.video.sensorReadoutMode=cam.SensorReadoutMode.GetValue()
    
    
    def set_default_cam_parameters(self,cam):
        #certains paramètres sont réglés par défaut dans l'application
        print("set_cam_parameters")
        if cam.IsOpen():
            if cam.IsGrabbing():
                self.stop_grabbing()
            cam.AcquisitionFrameRateEnable.SetValue(True)
            cam.ExposureAuto.SetValue("Off")
            # cam.GainAuto.SetValue("Off")
            cam.SensorReadoutMode.SetValue("Fast")                
        else : 
            print("No camera found !")
                                            
    

    
    def start_stop_acquisition_toggle(self):
        if self.liveVideo_btn.isChecked()==True:
            self.start_acquisition()
        else:
            self.stop_acquisition()
            # self.pause_acquisition()
    
      
    
    def start_acquisition(self):
        """
        récupération et affichage du flux video de la camera connectée
        #>>>pypylon.pylon_version.version : '5.0.1.build_6388'
        """
        # lastTime_play=time.time()
        # measuredPlayfps=0
        nbFramesBufferMax=1000
        # Création et démarrage des threads
        self.acquisition_thread = FileVideoStreamLive(self.video,nbFramesBufferMax)
        # self.analysis_thread = TadpoleAnalysis(self.acquisition_thread)
        
        # Démarrage des threads
        self.acquisition_thread.start()

        self.videoDisplayer_updater=Display(self,self.acquisition_thread,self.videoDisplayer,self.video)
        self.videoDisplayer_updater.setup_display_timer()

                  
    def stop_acquisition(self):
       
        if self.acquisition_thread:
            self.acquisition_thread.stop()
        self.videoDisplayer_updater.display_timer.stop()
             
        print("stop grabbing")
    
  
            
    
    def set_framerateFactor(self,i):
        framerateFactor = i
        #print(self.framerateFactor)
        
    def bufferSizeMB_update(self) :
        if self.video.frameWeight>0:
            self.bufferSizeValMB_label.setText(str(round(self.bufferSizeFrames_spinbox.value()*self.video.frameWeight,2))+"MB")       

     
    def activate_interface(self,context):
         #rend accessible les boutons pour la lecture de la video
         if context=="live video":
             self.liveVideo_btn.setEnabled(True)
    
    def disconnect_cam(self):
        self.video.device.Close()
        print("camera connected : ",self.video.device.IsOpen())


    def load_config(self) :
        nodeFile,_=QtWidgets.QFileDialog.getOpenFileName(
                        None,
                        "Open settings file",
                        QtCore.QDir.homePath(),
                        "*.pfs" # opencv ne prend en charge que le format avi
                    )
        pylon.FeaturePersistence.Load(nodeFile, self.video.device.GetNodeMap(), True)
        self.get_cam_parameters(self.video.device)
        #chargement des valeurs dans l'interface de réglage
        self.update(self.video.device)
    
        
    def save_config(self) :
    # featurePersistence = pylon.FeaturePersistence()

        print("Saving camera's node map to file...")
        nodeFile,_ = QtWidgets.QFileDialog.getSaveFileName(None,"Save file", "", "all")
        print(nodeFile)
        # Save the content of the camera's node map into the file.
        pylon.FeaturePersistence.Save(nodeFile, self.video.device.GetNodeMap())      


class Display(QtWidgets.QWidget):
    def __init__(self,capture,acquisition_thread,videoDisplay_Widget,video):
        super().__init__()
        self.capture=capture
        self.acquisition = acquisition_thread
        self.video=video
        # Contrôles d'affichage
        self.show_overlay = False#True désactivé par défaut
        self.display_fps = 0
        self.last_fps_time = time.time()
        self.fps_counter = 0
        self.videoDisplayer=videoDisplay_Widget
        self.current_frame_to_display=None
        self.display_lock=Lock()

        
    def setup_display_timer(self):
        """Configuration du timer d'affichage à 25 fps"""
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_display)
        self.display_timer.start(40)  # 40ms = 25 fps
        
    def update_display(self):
        """update live display"""
        try:
            # Récupération de l'image à afficher
            # print(self.acquisition.display_queue.qsize())
    
            with self.display_lock :
                display_data = self.acquisition.display_queue.get_nowait()
                
                frame = display_data['image']
                timestamp = display_data['timestamp']
                
                # Affichage direct sans overlay
                self.current_frame_to_display = frame.copy()
                tLive = image_processor.convert_imageToPyqtgraph(frame, self.video)
                self.videoDisplayer.img.setImage(tLive, autoLevels=False)
                
                # Mise à jour des compteurs
                self.fps_counter += 1
                if time.time() - self.last_fps_time >= 1.0:
                    if hasattr(self.capture, 'measuredLivefps_Label'):
                        self.capture.measuredLivefps_Label.setText(f'{self.video.measuredLivefps:.2f} fps')
                    self.fps_counter = 0
                    self.last_fps_time = time.time()
                
                # Mise à jour barre de progression
                if hasattr(self.capture, 'buffer_progress'):
                    buffer_usage = self.acquisition.get_buffer_usage()
                    self.capture.buffer_progress.setValue(int(buffer_usage))
                        
           
        except queue.Empty:
            pass  # Pas de nouvelle image
            # print("queue is empty")
        except Exception as e:
            print(f"Erreur affichage: {e}")

class StepSlider(QtWidgets.QSlider):
    #slider pour lequel il est possible de définir un step entre les valeurs
    def __init__(self, minValue,maxValue,interval):
        super(StepSlider, self).__init__()
        self._min = minValue
        self._max = maxValue
        self.interval = interval
        self._range_adjusted()

    def setValue(self, value):
        index = round((value - self._min) / self.interval)
        return super(StepSlider, self).setValue(index)

    def value(self):
        return self.index * self.interval + self._min

    @property
    def index(self):
        return super(StepSlider, self).value()

    def setIndex(self, index):
        return super(StepSlider, self).setValue(index)

    def setMinimum(self, value):
        self._min = value
        self._range_adjusted()

    def setMaximum(self, value):
        self._max = value
        self._range_adjusted()

    def setInterval(self, value):
        # To avoid division by zero
        if not value:
            raise ValueError('Interval of zero specified')
        self.interval = value
        self._range_adjusted()

    def _range_adjusted(self):
        # number_of_steps = int((self._max - self._min) / self.interval)
        number_of_steps = int(math.ceil((self._max - self._min) / self.interval))
        super(StepSlider, self).setMaximum(number_of_steps)
        


    
### classe et méthodes nécessaires pour faire fonctionner le code en application principale

class PyqtgWindow(QtWidgets.QMainWindow):
    
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent=None)
        
        # tracking=analysis.Tracking()
        self.video=vp.Video()
        # image=analysis.ImageContainer()
        self.display_updater=None     
        self.videoDisplay_Widget=vp.UIVideoDisplay(self.video)
        self.videoCapture_Widget=UIVideoCapture(self.videoDisplay_Widget,self.video,self.display_updater)

        
        self.setWindowIcon(QtgGui.QIcon('Imagys_blue\logoAnimotion-square-112.png'))
        ## Switch to using white background and black foreground
        #pg.setConfigOption('background', 'w')
        #pg.setConfigOption('foreground', 'k')
        
#        self.win = QtgGui.QMainWindow()
        self.area = DockArea()
        self.setCentralWidget(self.area)
        self.resize(1500,800)
        self.setWindowTitle('Video Capture - beta')

        self.d1 = Dock("Image", size=(1000,500))
        self.d11 = Dock("Video capture", size=(500,200))
        # self.d13 = Dock("Cam settings", size=(500,200))
        
        self.area.addDock(self.d1, 'left')
        self.area.addDock(self.d11, 'bottom', self.d1)
        # self.area.addDock(self.d13, 'bottom', self.d11)
        
        ## Add widgets into each dock----------------------------------------------------------------------------VIDEO DISPLAY
        ## image/video widget
        # self.w1=pg.GraphicsLayoutWidget()
        # self.d1.addWidget(self.w1)
        
        self.d1.addWidget(self.videoDisplay_Widget)
        self.d11.addWidget(self.videoCapture_Widget)
        
 #-----------------------------------------------------------------------------------------------------------dock13 - CAM SETTINGS
      
        
        # loads and sets the Qt stylesheet
        qss = QSSHelper.open_qss(os.path.join('Imagys_blue', 'Imagys-blue.qss'))
        self.setStyleSheet(qss)                   
        self.show()
        
    def close_camera(self):
        self.video.device.Close()
        print(self.video.device.IsOpen())
        # print(video.device.IsCameraDeviceRemoved())
        # print(video.device.IsPylonDeviceAttached())
    
    def closeEvent(self, event):
        reply = QtWidgets.QMessageBox.question(
            self,
            'User confirm',
            'Do you want to quit ?',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.close_camera()
            event.accept()  # Accepte la fermeture
        else:
            event.ignore()  # Annule la fermeture
        





if __name__ == '__main__':
    
    app = QtWidgets.QApplication([])
    
    # Configuration pour optimiser les performances
    app.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)
    
    #interface utilisateur
    ui=PyqtgWindow()
  
    app.exec_()
    # clean_app()
    print("the end")
    
    app.quit()