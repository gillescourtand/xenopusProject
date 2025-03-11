# -*- coding: utf-8 -*-
"""
Created on Mon Mar  7 08:52:52 2022

@author: courtand
"""


import os
import time
import psutil

from threading import Thread
# import the Queue class from Python 3
import sys
if sys.version_info >= (3, 0):
	from queue import Queue
# otherwise, import the Queue class for Python 2.7
else:
	from Queue import Queue

#basler
from pypylon import pylon
from pypylon import genicam

from PyQt5 import QtWidgets,QtCore,QtGui
from PyQt5.QtCore import pyqtSignal

import pyqtgraph as pg
# import pyqtgraph.ptime as ptime
from pyqtgraph.Qt import QtGui as QtgGui
from pyqtgraph.dockarea import *

from Imagys_blue.qsshelper import QSSHelper

import numpy as np
# import cv2
import math

import video_player_4 as vp
import image_processor
import analysis_2 as analysis




class FileVideoStreamLive:
    def __init__(self, path,video,queueSize=2000):
        # initialize the file video stream along with the boolean
        # used to indicate if the thread should be stopped or not
        print("stream",path)
        
        self.stream = video.device
        self.video=video
        self.stopped = False
        self.lastTime_live=0
         
        # initialize the queue used to store frames read from the video file
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
    
            grabResult = self.stream.RetrieveResult(100, pylon.TimeoutHandling_ThrowException)
            # retrieve the result using a timeout of 100 milliseconds : if 2 seconds pass and there is no result, it would simply return
            # Camera.StopGrabbing() is called automatically by the RetrieveResult() method
    
            # When the cameras in the array are created 
            # the camera context value is set to the index of the camera in the array.
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
                i+=1
                # add the frame to the queue
                self.Q.put(frame)
                
                #calcul du framerate
                now = time.time()
                dt = now - self.lastTime_live + 0.0000001
                self.lastTime_live = now
                if self.video.measuredLivefps is None:
                    self.video.measuredLivefps = 1.0/dt
                else:
                    s = np.clip(dt*3., 0, 1)
                    self.video.measuredLivefps = self.video.measuredLivefps * (1-s) + (1.0/dt) * s
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



class UIVideoCapture(pg.LayoutWidget):
    
    analysisSignal=pyqtSignal(int)
    pauseSignal=pyqtSignal()
    updatePlotSignal=pyqtSignal()
    updateTrackMapSignal=pyqtSignal()
    
    testSignal=pyqtSignal(int)
    
    def __init__(self,videoDisplay,video,image,parent=None):
        super(UIVideoCapture,self).__init__()
        
        self.video=video
        self.vidDisplay=videoDisplay
        self.image=image
        
        # self.cam=video.device
       
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
        self.liveVideo_btn.clicked.connect(self.grab_video)
        
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
        # self.sensorReadMode_comboBox.setCurrentText(cam.SensorReadoutMode.GetValue())
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
        
        # self.video.centerX=cam.CenterX.GetValue()
        # self.video.centerY=cam.CenterY.GetValue()
        # self.video.gain=cam.Gain.GetValue()
        # self.video.gamma=cam.Gamma.GetValue()
        # self.video.LUT=image_processor.gamma_LUT(1)
    
    
    #----------------------------------------------------------------------
        
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
        # self.video.sensorReadoutMode=cam.SensorReadoutMode.GetValue()
    
    
    def set_default_cam_parameters(self,cam):
        #certains paramètres sont réglés par défaut dans l'application
        print("set_cam_parameters")
        if cam.IsOpen():
            if cam.IsGrabbing():
                self.stop_grabbing()
            cam.AcquisitionFrameRateEnable.SetValue(True)
            cam.ExposureAuto.SetValue("Off")
            # cam.GainAuto.SetValue("Off")
            # cam.SensorReadoutMode.SetValue("Fast")                
        else : 
            print("No camera found !")
                                            
    
    # def init_player(self) :
    def init_grabber(self) :
        #calcul de la mémoire disponible
        svmem=psutil.virtual_memory().available/1024.0**2
        print("Available memory : ", svmem,"MB")
        self.availableMemValue_label.setText(str(round(svmem,2)))
        #calculer la taille du buffer en fonction du poids de chaque frame
        fshape=self.video.currentFrame.shape
        #taille d'un frame en MB
        self.video.frameWeight=fshape[0]*fshape[1]*fshape[2]/1024/1024
        print("frame size (MB) : ",self.video.frameWeight)
        
        bufferSizeMax=2/3*svmem
        #en nombre de frame
        nbFramesBufferMax=int(bufferSizeMax/self.video.frameWeight)
        self.bufferSizeFrames_spinbox.setMaximum(nbFramesBufferMax)
        self.bufferSizeFrames_spinbox.setValue(nbFramesBufferMax)   
        #intancier le FileVideoStream_ToPlay pour la mise en buffer
        # video.grabber = FileVideoStream_ToPlay(video.capture,ui.bufferSizeFrames_spinbox.value()) 
        self.video.grabber = FileVideoStreamLive(self.video.capture,self.video,nbFramesBufferMax)
        
    def grab_video(self):
        """
        récupération et affichage du flux video de la camera connectée
        #>>>pypylon.pylon_version.version : '5.0.1.build_6388'
        #tester si une caméra est disponible
        """
        # timeToUpdate=0
        lastTime_play=time.time()
        measuredPlayfps=0
        
        #print("grab_video")
        if self.liveVideo_btn.isChecked()==True:
            self.video.isAcquiring=True #=video.device.IsGrabbing(), self.assertTrue(cam.IsGrabbing())
            
            try :
                #>>>available_cameras : [<DeviceInfo Basler acA2040-90um (xxxxxxx)>]
                #video.devices = pypylon.factory.find_devices()
                # Get all attached devices and exit application if no device is found.
                # Get the transport layer factory.
                
                #tester si un ou plusieurs périphériques sont connectés
                # tlFactory = pylon.TlFactory.GetInstance()
                # video.devices = tlFactory.EnumerateDevices()
                # print("video.devices : ",str(video.devices))
                if self.video.device.IsPylonDeviceAttached():            
                
                    self.video.grabber = FileVideoStreamLive("live", self.video).start()
                    self.video.path="video live"
                    time.sleep(0.1)
                                    
                    # timeToUpdate=ptime.time()
                    QtCore.QTimer.singleShot(50, lambda: self.refresh_live_display())
                    ## show the frame 
                    # self.vidDisplay.show_frame_in_pyqtgraph(self.video.currentFrame)
                                   
                    while self.liveVideo_btn.isChecked()==True:
                        #tant que le buffer de streaming n'est pas vide
                        if self.video.grabber.more():
                            # read_and_analyze()
                            # grab the frame from the threaded video file stream    
                            frame = self.video.grabber.read()
                            # print("read")
                            self.image.forAnalysis= frame
                            self.analysisSignal.emit(frame)
                            #calcul du framerate
                            now = time.time()
                            dt = now - lastTime_play + 0.0000001
                            # print (dt)
                            lastTime_play = now
                            if measuredPlayfps is 0:
                                measuredPlayfps = 1.0/dt
                            else:
                                s = np.clip(dt*3., 0, 1)
                                measuredPlayfps = measuredPlayfps * (1-s) + (1.0/dt) * s
                            #print("dt read:",dt)
                            #avec un framerate fps pour le maintenir on ajoute un timer pour ralentir l'affichage si besoin
                            #fpsTime=1/video.fps
                            #time.sleep(fpsTime)
                            #timeToSleep=fpsTime-dt
                            #if timeToSleep>0:
                                #time.sleep(timeToSleep)
                        else:
                            #print("wait...")
                            # app.processEvents()
                            QtWidgets.QApplication.instance().processEvents()
                        #    #calcul du temps pour rafraichir l'affichage à 30 images par seconde
                        #if now-varM.timeToUpdate>0.03 : 
                        #    app.processEvents()
            except genicam.GenericException as e:
                # Error handling.
                print("An exception occurred.", str(e))#.GetDescription())
                
                print("stooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooop")    
               
                #-------------------------------------------------------------------------------------------------------
              
           
        else :#bouton "live" désactivé
            self.video.isAcquiring=False
            print("video.isAcquiring=",self.video.isAcquiring)
            # wait a while to allow a processor to finish
            time.sleep(0.5)
            #while video.stream.Q.size>0:
            #    read_and_analyze()
                 
            self.video.grabber.stop()
            time.sleep(0.5)
            try :
                self.video.grabber.stream.StopGrabbing()
            except AttributeError as e :
                print(e)
            print("StopGrabbing")
            #video.grabber.stream.Close() #la camera ne doit pas être fermée pour modifier les paramètres d'acquisition
            #print("camera closed")
            #TODO : insérer un appel à Close() pour déconnecter la caméra avant de fermer l'application
                    
    
    def refresh_live_display(self):
        #l'affichage ne peut être fait à 200 images/s le refraichissement est donc géré indépendamment 
        self.buffer_progress.setValue(int(self.video.grabber.Q.qsize()/self.video.grabber.Q.maxsize*100))
        # print(self.buffer_progress.value())
        if self.liveVideo_btn.isChecked()==True :
            tLive=image_processor.convert_imageToPyqtgraph(self.image.forAnalysis,self.video)
            #ui.img_Live.setImage(tLive,autoLevels=False)
            self.vidDisplay.img.setImage(tLive,autoLevels=False)
            self.measuredLivefps_Label.setText('%0.2f fps' % self.video.measuredLivefps)
            self.lenBuffer_label.setText(str(self.video.grabber.Q.qsize()))
            
            #app.processEvents()  ## force complete redraw for every plot
            QtCore.QTimer.singleShot(50, lambda: self.refresh_live_display())
                  
        else : print("stop refresh live display")
            
    
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



    def stop_grabbing(self):
        print("stop grabbing...")
        # video.device.StopGrabbing()
        self.liveVideo_btn.setChecked(False)
        self.grab_video()
        time.sleep(0.5)
        
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
        
 #-----------------------------------------------------------------------------------------------------------dock13 - CAM SETTINGS
      
        
        # loads and sets the Qt stylesheet
        qss = QSSHelper.open_qss(os.path.join('Imagys_blue', 'Imagys-blue.qss'))
        self.setStyleSheet(qss)                   
        self.show()
        

def close_camera():
    video.device.Close()
    print(video.device.IsOpen())
    # print(video.device.IsCameraDeviceRemoved())
    # print(video.device.IsPylonDeviceAttached())

def test_Live():
    if ui.track_checkBox.isChecked()==True:
        print("test_Live")
        # pen=pg.mkPen((0,255,255), width=2)
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
        
               
        

if __name__ == '__main__':
    
    app = QtWidgets.QApplication([])
    tracking=analysis.Tracking()
    video=vp.Video()
    image=analysis.ImageContainer()
    
    videoDisplay_Widget=vp.UIVideoDisplay(video)
    videoCapture_Widget=UIVideoCapture(videoDisplay_Widget,video,image,tracking)
    # camParam_Widget=UICamParamWidget(video)
    #interface utilisateur
    ui=PyqtgWindow()
    ui.d1.addWidget(videoDisplay_Widget)
    ui.d11.addWidget(videoCapture_Widget)
    # ui.d13.addWidget(camParam_Widget)
    
    app.exec_()
    # clean_app()
    print("the end")
    close_camera()
    
    app.quit()