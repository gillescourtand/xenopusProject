# -*- coding: utf-8 -*-
"""
Created on Wed Nov 10 14:22:52 2021

@author: Courtand, Kadri 

Legacy video-state and video-display widgets.

This module keeps the historical Video, UIVideoPlayer and UIVideoDisplay API
used by the rest of the application. The standalone test window and commented
prototype blocks were removed to keep the module focused on reusable code.
"""

import os
import sys
import time
from threading import Thread

import cv2
import numpy as np
import psutil
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QIcon

try:
    from queue import Queue
except ImportError:  # pragma: no cover - Python 2 compatibility kept for legacy code.
    from Queue import Queue

from xenopus_app.video import image_conversion


class ImageProcessor:
    """Compatibility wrapper around the shared image-conversion helpers."""

    def gamma_LUT(self, gamma):
        """Build a gamma-correction lookup table."""
        return image_conversion.gamma_LUT(gamma)

    def convert_imageToPyqtgraph(self, img, vid):
        """Convert an OpenCV frame for pyqtgraph display."""
        return image_conversion.convert_imageToPyqtgraph(img, vid)


def track_display():
    """Compatibility hook called after manual frame navigation."""
    pass


def open_file(ui, fileType, fileFormat):
    """Open a Qt file dialog and return the selected path."""
    filePath, _ = QtWidgets.QFileDialog.getOpenFileName(
        ui,
        "Open " + fileType,
        QtCore.QDir.homePath(),
        fileType + fileFormat,
    )
    return filePath


def extract_fileName(filePath):
    """Return the basename of a file path."""
    return os.path.split(filePath)[-1]


def set_videoRotation(vid, frame):
    """Apply the rotation currently stored on the Video object."""
    if vid.rotation == "90_clockwise":
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if vid.rotation == "90_counterclockwise":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if vid.rotation == "180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


class Video:
    """Container for video, camera, display and acquisition parameters."""

    def __init__(self):
        """Initialize all video-related runtime fields."""
        self.path = None
        self.capture = None

        self.firstFrame = None
        self.LUT = None
        self.isAcquiring = False
        self.buffering = False
        self.playing = False
        self.grabber = None
        self.frameWeight = 0
        self.measuredLivefps = 0

        self.currentGrayFrame = None
        self.currentFrame = None
        self.currentTFrame = None

        self.devices = []
        self.device = None
        self.featuresFile = None

        self.pixFormat = None
        self.ToRGB = None
        self.width = None
        self.height = None
        self.fps = None
        self.nbFrames = None

        self.grabFrameRate = None
        self.offsetX = None
        self.offsetY = None
        self.centerX = None
        self.centerY = None
        self.gainAuto = None
        self.gain = None
        self.gamma = None
        self.exposure = None
        self.sensorReadoutMode = None

        self.rescale = 1
        self.rotation = "No rotation"
        self.pos = None


class FileVideoStream_ToPlay:
    """Read video frames in a background thread before UI playback."""

    def __init__(self, videoStream, video, queueSize=200):
        """Create the playback buffer around an OpenCV VideoCapture."""
        self.stream = videoStream
        self.vid = video
        self.stopped = False
        self.frameStep = 1
        self.Q = Queue(maxsize=queueSize)

    def start(self, nframe, framerateFactor):
        """Start buffered reading from the requested frame index."""
        self.frameStep = framerateFactor
        thread = Thread(target=self.update, args=())
        thread.daemon = True
        self.stream.set(1, nframe)
        thread.start()
        return self

    def update(self):
        """Read frames until the end of the video or until stop() is called."""
        endFrame = self.stream.get(7)
        print("last frame : ", endFrame)
        print("frame factor : ", self.frameStep)

        try:
            while self.stream.get(1) < endFrame:
                if self.stopped:
                    print("videostreamToPlay stopped")
                    return

                if not self.Q.full():
                    if self.frameStep != 1:
                        self.stream.set(
                            cv2.CAP_PROP_POS_FRAMES,
                            self.stream.get(1) - 1 + self.frameStep,
                        )

                    grabbed, frame = self.stream.read()
                    currentFrame = self.stream.get(1)

                    if not grabbed:
                        print("error : frame ", currentFrame, " not grabbed")
                        return

                    if self.vid.rescale != 1:
                        frame = cv2.resize(
                            frame,
                            None,
                            fx=self.vid.rescale,
                            fy=self.vid.rescale,
                            interpolation=cv2.INTER_CUBIC,
                        )

                    if self.vid.rotation != "No rotation":
                        frame = set_videoRotation(self.vid, frame)

                    self.Q.put(frame)
                else:
                    time.sleep(2.0)
                    print("fulllll")
        finally:
            print("read frame ended")
            self.stop()

    def read(self):
        """Return the next buffered frame."""
        return self.Q.get()

    def more(self):
        """Return True when at least one frame is ready in the buffer."""
        return self.Q.qsize() > 0

    def stop(self):
        """Request the background reader to stop."""
        self.stopped = True


class UIVideoPlayer(pg.LayoutWidget):
    """Legacy video-player widget used for file playback."""

    analysisSignal = pyqtSignal(int)
    loadVideoSignal = pyqtSignal()
    playSignal = pyqtSignal()
    pauseSignal = pyqtSignal()
    updatePlotSignal = pyqtSignal()
    updateTrackMapSignal = pyqtSignal()
    playFramerateSignal = pyqtSignal()
    timeLineMoveSignal = pyqtSignal(int)

    def __init__(self, videoDisplay, video, icon_path, parent=None):
        """Build the playback controls and bind them to a Video object."""
        super(UIVideoPlayer, self).__init__()

        self.vid = video
        self.vidDisplay = videoDisplay
        self.image_processor = ImageProcessor()

        self.framerateFactor = 1
        self.idxResultArray = 0

        self.loadVideo_btn = QtWidgets.QPushButton("Load video")
        self.loadVideo_btn.clicked.connect(self.load_video)
        self.lb_videoName = QtWidgets.QLabel(""" No video """)
        self.lb_videoName.setMaximumWidth(200)
        self.lb_videoName.setMinimumWidth(100)

        self.playVideo_btn = QtWidgets.QPushButton()
        self.playVideo_btn.setCheckable(True)
        self.playVideo_btn.setChecked(False)
        self.playVideo_btn.clicked.connect(self.playpause_video)
        self.playVideo_btn.setEnabled(False)
        play_icon_path = os.path.join("Imagys_blue", "play_button_off.png")
        play_icon = resource_path(play_icon_path)
        self.playVideo_btn.setIcon(QIcon(play_icon))
        self.playVideo_btn.setMaximumWidth(80)
        self.playVideo_btn.setMinimumWidth(40)
        self.playVideo_btn.setMaximumHeight(20)

        self.stepFwdVideo_btn = QtWidgets.QPushButton(">")
        self.stepFwdVideo_btn.clicked.connect(self.forward_video)
        self.stepFwdVideo_btn.setEnabled(False)
        self.stepFwdVideo_btn.setMaximumWidth(40)
        self.stepFwdVideo_btn.setMinimumWidth(40)

        self.stepBwdVideo_btn = QtWidgets.QPushButton("<")
        self.stepBwdVideo_btn.clicked.connect(self.backward_video)
        self.stepBwdVideo_btn.setEnabled(False)
        self.stepBwdVideo_btn.setMaximumWidth(40)
        self.stepBwdVideo_btn.setMinimumWidth(40)

        self.numFrame_label = QtWidgets.QLabel("00")
        self.numFrame_label.setAlignment(QtCore.Qt.AlignCenter)
        self.numFrame_label.setMaximumWidth(60)
        numFrameSeparator_label = QtWidgets.QLabel("/")
        numFrameSeparator_label.setAlignment(QtCore.Qt.AlignCenter)
        numFrameSeparator_label.setMaximumWidth(3)
        self.numTotalFrame_label = QtWidgets.QLabel("00 frame")
        self.numTotalFrame_label.setAlignment(QtCore.Qt.AlignCenter)
        self.numTotalFrame_label.setMaximumWidth(60)
        splitter_numFrame = QtWidgets.QSplitter()
        splitter_numFrame.setOrientation(QtCore.Qt.Horizontal)
        splitter_numFrame.addWidget(self.numFrame_label)
        splitter_numFrame.addWidget(numFrameSeparator_label)
        splitter_numFrame.addWidget(self.numTotalFrame_label)

        self.currentTime_label = QtWidgets.QLabel("00")
        self.currentTime_label.setAlignment(QtCore.Qt.AlignCenter)
        self.currentTime_label.setMaximumWidth(60)
        timeSeparator_label = QtWidgets.QLabel("/")
        timeSeparator_label.setAlignment(QtCore.Qt.AlignCenter)
        timeSeparator_label.setMaximumWidth(3)
        self.totalTime_label = QtWidgets.QLabel("00 sec.")
        self.totalTime_label.setAlignment(QtCore.Qt.AlignCenter)
        self.totalTime_label.setMaximumWidth(60)
        splitter_timeCode = QtWidgets.QSplitter()
        splitter_timeCode.setOrientation(QtCore.Qt.Horizontal)
        splitter_timeCode.addWidget(self.currentTime_label)
        splitter_timeCode.addWidget(timeSeparator_label)
        splitter_timeCode.addWidget(self.totalTime_label)

        splitter_frame_time = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter_frame_time.addWidget(splitter_numFrame)
        splitter_frame_time.addWidget(splitter_timeCode)

        self.timeLine_slider = QtWidgets.QSlider()
        self.timeLine_slider.setPageStep(100)
        self.timeLine_slider.setProperty("value", 1)
        self.timeLine_slider.setOrientation(QtCore.Qt.Horizontal)
        self.timeLine_slider.valueChanged.connect(self.slider_move)
        self.timeLine_slider.valueChanged["int"].connect(self.numFrame_label.setNum)
        self.timeLine_slider.setEnabled(False)

        splitter_frameplayBtn = QtWidgets.QSplitter()
        splitter_frameplayBtn.setOrientation(QtCore.Qt.Horizontal)
        splitter_frameplayBtn.addWidget(self.timeLine_slider)
        splitter_frameplayBtn.addWidget(self.playVideo_btn)
        splitter_frameplayBtn.addWidget(self.stepBwdVideo_btn)
        splitter_frameplayBtn.addWidget(self.stepFwdVideo_btn)

        self.playSpeed_label = QtWidgets.QLabel("each n frame")
        self.playSpeed_label.setMaximumWidth(80)
        self.playSpeed_spinbox = QtWidgets.QSpinBox()
        self.playSpeed_spinbox.setMaximumWidth(50)
        self.playSpeed_spinbox.setMinimum(1)
        self.playSpeed_spinbox.setMaximum(200)
        self.playSpeed_spinbox.setValue(1)
        self.playSpeed_spinbox.valueChanged.connect(self.set_framerateFactor)
        self.splitter_playSpeed = QtWidgets.QSplitter()
        self.splitter_playSpeed.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_playSpeed.addWidget(self.playSpeed_label)
        self.splitter_playSpeed.addWidget(self.playSpeed_spinbox)

        self.videoScale_label = QtWidgets.QLabel("video scale 1/")
        self.videoScale_label.setMaximumWidth(80)
        self.videoScale_label.setMinimumWidth(80)
        self.videoScale_spinbox = QtWidgets.QSpinBox()
        self.videoScale_spinbox.setMaximumWidth(50)
        self.videoScale_spinbox.setMinimum(1)
        self.videoScale_spinbox.setMaximum(6)
        self.videoScale_spinbox.setValue(1)
        self.videoScale_spinbox.valueChanged.connect(self.set_videoScale)
        splitter_videoScale = QtWidgets.QSplitter()
        splitter_videoScale.setOrientation(QtCore.Qt.Horizontal)
        splitter_videoScale.addWidget(self.videoScale_label)
        splitter_videoScale.addWidget(self.videoScale_spinbox)
        splitter_videoScale.setEnabled(True)

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

        availableMemory_label = QtWidgets.QLabel("Available memory (MB) : ")
        availableMemory_label.setMinimumWidth(120)
        availableMemory_label.setMaximumWidth(120)
        self.availableMemValue_label = QtWidgets.QLabel("0000")
        self.availableMemValue_label.setMinimumWidth(40)
        self.availableMemValue_label.setMaximumWidth(60)
        self.bufferSize_label = QtWidgets.QLabel("Buffer size (frames) ")
        self.bufferSize_label.setMinimumWidth(100)
        self.bufferSize_label.setMaximumWidth(100)
        self.bufferSizeValue_label = QtWidgets.QLabel("00000")
        self.bufferSizeValue_label.setMinimumWidth(40)
        self.bufferSizeValue_label.setMaximumWidth(60)
        self.bufferSizeValue_label.setAlignment(QtCore.Qt.AlignCenter)

        self.bufferSizeValMB_label = QtWidgets.QLabel("000MB")
        self.bufferSizeValMB_label.setMinimumWidth(60)
        self.bufferSizeValMB_label.setMaximumWidth(60)
        self.bufferSizeValMB_label.setAlignment(QtCore.Qt.AlignCenter)
        self.bufferSizeValue_slider = QtWidgets.QSlider()
        self.bufferSizeValue_slider.setProperty("value", 0)
        self.bufferSizeValue_slider.setOrientation(QtCore.Qt.Horizontal)
        self.bufferSizeValue_slider.setMinimumWidth(100)
        self.bufferSizeValue_slider.valueChanged.connect(self.bufferSizeMB_update)
        self.bufferSizeValue_slider.valueChanged["int"].connect(self.bufferSizeValue_label.setNum)

        self.buffer_progress = QtWidgets.QProgressBar()
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
        splitterBuffer.addWidget(self.buffer_progress)
        splitterBuffer.setEnabled(False)

        self.addWidget(splitter_frameplayBtn, row=0, col=0, colspan=6)
        self.addWidget(self.loadVideo_btn, row=1, col=0)
        self.addWidget(self.lb_videoName, row=1, col=1, colspan=3)
        self.addWidget(splitter_playerOptions, row=1, col=4, colspan=2)
        self.addWidget(splitter_frame_time, row=0, col=6, rowspan=2)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.video_play_framerate)

    def load_video(self):
        """Open a video file and initialize playback if a path was selected."""
        print("load...")
        self.vid.path = open_file(None, "Video file", " *.mpg;*.mp4;*.mov;*.mkv;*.avi")
        print("open :", self.vid.path)

        if len(self.vid.path) > 3:
            self.init_video(self.vid.path)
        else:
            print("no image ")

    def init_video(self, videoPath):
        """Load video metadata, display the first frame and initialize buffering."""
        videoName = extract_fileName(videoPath)
        self.lb_videoName.setText(videoName)

        try:
            self.vid.capture = cv2.VideoCapture(videoPath)
            ret = self.vid.capture.open(videoPath)
            if ret is False:
                print("False")
                QtWidgets.QMessageBox.warning(None, "Tracking", str("video error"), QtWidgets.QMessageBox.Ok)
        except cv2.error as exc:
            print("error : ", exc)
            QtWidgets.QMessageBox.warning(None, "Tracking", str(exc), QtWidgets.QMessageBox.Ok)

        self.vid.codec = self.vid.capture.get(cv2.CAP_PROP_FOURCC)
        self.vid.ToRGB = self.vid.capture.get(16)
        self.vid.width = self.vid.capture.get(3)
        self.vid.height = self.vid.capture.get(4)
        self.vid.fps = self.vid.capture.get(5)
        self.vid.nbFrames = self.vid.capture.get(7)
        self.numTotalFrame_label.setText(str(int(self.vid.nbFrames)))
        self.vid.LUT = self.image_processor.gamma_LUT(1.8)
        self.vid.firstFrame = 1

        print("codec : ", str(self.vid.codec))
        print("convert : " + str(self.vid.ToRGB))
        print("Frames= " + str(self.vid.nbFrames) + " at " + str(self.vid.fps) + "fps")
        print("frame W = " + str(self.vid.width) + " - H = " + str(self.vid.height))

        self.update_toolTip(self.lb_videoName)
        self.loadVideoSignal.emit()
        self.activate_interface("load video")
        print("player activated")
        self.play_frame(self.vid.firstFrame)
        self.init_player()

    def init_player(self):
        """Configure the timeline and frame-buffer size from video metadata."""
        self.timeLine_slider.setMaximum(int(self.vid.nbFrames))
        self.timeLine_slider.setValue(int(self.vid.firstFrame))

        svmem = psutil.virtual_memory().available / 1024.0 ** 2
        print("Available memory : ", svmem, "MB")
        fshape = self.vid.currentFrame.shape
        print("frame shape : ", fshape)

        self.vid.frameWeight = fshape[0] * fshape[1] * fshape[2] / 1024 / 1024
        print("frame size (MB) : ", self.vid.frameWeight)

        bufferSizeMax = 2 / 3 * svmem
        nbFramesBufferMax = int(bufferSizeMax / self.vid.frameWeight)
        self.vid.grabber = FileVideoStream_ToPlay(self.vid.capture, self.vid, nbFramesBufferMax)

    def update_toolTip(self, item):
        """Update the video-name tooltip with path and metadata."""
        videoInfo = "{0} \n size : {1}, {2} \n frames : {3} at {4} fps".format(
            self.vid.path,
            int(self.vid.width),
            int(self.vid.height),
            int(self.vid.nbFrames),
            int(self.vid.fps),
        )
        item.setToolTip(videoInfo)

    def playpause_video(self):
        """Start or pause playback depending on the play button state."""
        if self.vid.path is None:
            message = "You have to open a video first"
            QtWidgets.QMessageBox.warning(None, "no video", str(message), QtWidgets.QMessageBox.Ok)
        else:
            if self.playVideo_btn.isChecked() is True:
                self.vid.playing = True
                self.playSignal.emit()
                self.splitter_playSpeed.setEnabled(False)
                self.vid.grabber.stopped = False
                self.vid.grabber.start(self.timeLine_slider.value(), self.framerateFactor)
                print("grabber started")
                time.sleep(1.0)
                timer_interval = int(1000 / self.vid.fps)
                self.timer.start(timer_interval)
            else:
                self.pause_video()

    def pause_video(self):
        """Pause playback and clear buffered frames."""
        print("pause_video")
        self.vid.playing = False
        self.vid.grabber.stop()
        self.vid.grabber.Q.queue.clear()

        self.playVideo_btn.setChecked(False)
        self.splitter_playSpeed.setEnabled(True)
        self.pauseSignal.emit()

    def forward_video(self):
        """Move the timeline forward by the current frame step."""
        self.timeLine_slider.setValue(self.timeLine_slider.value() + self.framerateFactor)
        track_display()

    def backward_video(self):
        """Move the timeline backward by the current frame step."""
        self.timeLine_slider.setValue(self.timeLine_slider.value() - self.framerateFactor)
        track_display()

    def slider_move(self):
        """Display the selected frame when the timeline moves while paused."""
        if self.vid.capture is not None:
            if self.playVideo_btn.isChecked() is False:
                self.play_frame(self.timeLine_slider.value())
                self.timeLineMoveSignal.emit(self.timeLine_slider.value())

    def apply_frameRotation(self):
        """Apply the selected rotation to the current displayed frame."""
        self.vid.rotation = self.videoRotation_comboBox.currentText()
        self.play_frame(self.timeLine_slider.value())

    def play_frame(self, iframe):
        """Read and display a single frame by index."""
        retVal_set = False
        if self.vid.capture is not None:
            retVal_set = self.vid.capture.set(1, iframe)

        if retVal_set is True:
            retVal_read, self.vid.currentFrame = self.vid.capture.read()
            currentTime = self.vid.capture.get(cv2.CAP_PROP_POS_MSEC)
            seconds, milliseconds = divmod(currentTime, 1000)
            minutes, seconds = divmod(seconds, 60)
            label_time = f"{int(minutes):02d}:{int(seconds):02d}.{int(milliseconds):03d}"
        else:
            print("wrong frame index")
            retVal_read = False
            label_time = "00:00.000"

        if retVal_read is True:
            if self.vid.rescale != 1:
                self.vid.currentFrame = cv2.resize(
                    self.vid.currentFrame,
                    None,
                    fx=self.vid.rescale,
                    fy=self.vid.rescale,
                    interpolation=cv2.INTER_CUBIC,
                )

            if self.videoRotation_comboBox.currentText() != "No rotation":
                self.vid.currentFrame = set_videoRotation(self.vid, self.vid.currentFrame)

            tFrame = self.image_processor.convert_imageToPyqtgraph(self.vid.currentFrame, self.vid)
            self.vidDisplay.img.setImage(tFrame, autoLevels=False)
            self.numFrame_label.setNum(iframe)
            self.currentTime_label.setText(label_time)
            self.analysisSignal.emit(iframe)
        else:
            print("can't read this frame")

    def video_play_framerate(self):
        """Display the next buffered frame at the current playback rate."""
        if self.vid.grabber.more():
            self.vid.currentFrame = self.vid.grabber.read()
        else:
            for _ in range(2):
                print("waiting for new frame...")
                time.sleep(1)
                if self.vid.grabber.more():
                    self.vid.currentFrame = self.vid.grabber.read()
                    break
            if self.vid.grabber.more() is False:
                print("no more frame")
                self.timer.stop()

        self.vidDisplay.show_frame_in_pyqtgraph(self.vid.currentFrame)
        iframe = self.timeLine_slider.value()
        self.analysisSignal.emit(iframe)

        self.timeLine_slider.setValue(self.timeLine_slider.value() + self.framerateFactor)
        currentTime = (self.timeLine_slider.value() / self.vid.fps) * 1000
        seconds, milliseconds = divmod(currentTime, 1000)
        minutes, seconds = divmod(seconds, 60)
        label_time = f"{int(minutes):02d}:{int(seconds):02d}.{int(milliseconds):03d}"
        self.currentTime_label.setText(label_time)

        QtWidgets.QApplication.instance().processEvents()

    def video_play_stream(self, fvs):
        """Play frames directly from a FileVideoStream_ToPlay instance."""
        print("video position : ", self.vid.firstFrame)
        print("video nb frames : ", self.vid.nbFrames)

        for _ in range(int(self.vid.firstFrame), int(self.vid.nbFrames)):
            if self.vid.playing is False:
                break

            if fvs.more():
                self.vid.currentFrame = fvs.read()
            else:
                for _ in range(2):
                    print("waiting for new frame...")
                    time.sleep(1)
                    if fvs.more():
                        self.vid.currentFrame = fvs.read()
                        break
                if fvs.more() is False:
                    print("no more frame")
                    break

            self.vidDisplay.show_frame_in_pyqtgraph(self.vid.currentFrame)
            iframe = self.timeLine_slider.value()
            self.analysisSignal.emit(iframe)
            self.timeLine_slider.setValue(self.timeLine_slider.value() + self.framerateFactor)
            QtWidgets.QApplication.instance().processEvents()

        print("fvs.more=", fvs.more())

    def set_framerateFactor(self, i):
        """Set the number of frames skipped between playback updates."""
        self.framerateFactor = i

    def set_videoScale(self, i):
        """Set the display scaling factor and refresh the current frame."""
        if self.vid.capture is not None:
            self.vid.rescale = 1 / i
            iframe = self.timeLine_slider.value()
            self.play_frame(iframe)
        else:
            print("no video")

    def bufferSizeMB_update(self):
        """Update the buffer-size label in megabytes."""
        if self.vid.frameWeight > 0:
            self.bufferSizeValMB_label.setText(
                str(round(self.bufferSizeValue_slider.value() * self.vid.frameWeight, 2)) + "MB"
            )

    def activate_interface(self, context):
        """Enable video-player controls after a video has been loaded."""
        if context == "load video":
            self.playVideo_btn.setEnabled(True)
            self.playVideo_btn.setChecked(False)
            self.timeLine_slider.setEnabled(True)
            self.stepFwdVideo_btn.setEnabled(True)
            self.stepBwdVideo_btn.setEnabled(True)


class UIVideoDisplay(pg.GraphicsLayoutWidget):
    """Image display widget used by live and file-based video views."""

    def __init__(self, video, parent=None):
        """Create a pyqtgraph ImageItem inside an aspect-locked ViewBox."""
        super(UIVideoDisplay, self).__init__()

        self.vid = video
        self.image_processor = ImageProcessor()

        self.gviewBox = pg.ViewBox()
        self.plotView = self.addPlot(viewBox=self.gviewBox)
        self.plotView.setAspectLocked(True)

        self.img = pg.ImageItem()
        self.gviewBox.addItem(self.img)

    def show_frame_in_pyqtgraph(self, cv2frame):
        """Convert and display an OpenCV frame."""
        tFrame = self.image_processor.convert_imageToPyqtgraph(cv2frame, self.vid)
        self.img.setImage(tFrame, autoLevels=False)

    def contextMenuEvent(self, event):
        """Show a small context menu on right click."""
        menu = QtWidgets.QMenu(self)
        load_action = QtWidgets.QAction("Load video", self)
        load_action.triggered.connect(self.load_video)
        menu.addAction(load_action)
        menu.exec_(event.globalPos())

    def load_video(self):
        """Compatibility placeholder for the context-menu action."""
        print("load video...")


def resource_path(relative_path):
    """Return an absolute resource path for source or frozen execution."""
    if getattr(sys, "frozen", False):
        base_path = sys._MEIPASS
    else:
        base_path = os.path.dirname(os.path.abspath(__file__))

    return os.path.join(base_path, relative_path)
