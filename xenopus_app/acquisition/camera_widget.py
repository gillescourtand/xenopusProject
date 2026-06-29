# -*- coding: utf-8 -*-
"""
camera_widget.py

Basler camera acquisition panel used by the Xenopus real-time tracking UI.

This module contains only the live-camera side of the application:
- camera connection and Basler parameter controls;
- the acquisition thread feeding the analysis buffer;
- the light live display timer;
- a step-based slider used for Basler integer parameters.

The old standalone demo window and commented-out legacy code were removed so
this module can stay focused on the production UI.

@author: Courtand, Kadri 
"""

import math
import queue
import time
from collections import deque
from threading import Lock, Thread

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QTimer
from pypylon import genicam, pylon

from xenopus_app.video import image_conversion as image_processor


class SimpleFrameBuffer:
    """
    Thread-safe FIFO buffer used by the live acquisition thread.

    The analysis buffer keeps all frames that must be processed by the tracking
    pipeline. The display path is handled by a separate lightweight queue in
    ``FileVideoStreamLive`` so the UI can be refreshed at a lower rate.
    """

    def __init__(self, analysis_maxsize, display_maxsize=5):
        """
        Create the frame buffers.

        Parameters
        ----------
        analysis_maxsize : int
            Maximum number of frames kept for tracking.
        display_maxsize : int
            Kept for compatibility with older statistics code.
        """
        self.analysis_frames = deque(maxlen=analysis_maxsize)
        self.analysis_lock = Lock()

        self.display_frames = deque(maxlen=display_maxsize)
        self.display_lock = Lock()

        self.total_frames = 0
        self.dropped_analysis = 0
        self.dropped_display = 0

    def add_frame(self, frame_data, for_analysis=True, for_display=False):
        """
        Add a frame to the analysis buffer.

        Parameters
        ----------
        frame_data : dict
            Dictionary containing ``image``, ``timestamp`` and ``frame_id``.
        for_analysis : bool
            Whether the frame must be kept for tracking.
        for_display : bool
            Reserved for compatibility. Display frames are currently sent through
            ``FileVideoStreamLive.display_queue``.
        """
        self.total_frames += 1

        if not for_analysis:
            return

        with self.analysis_lock:
            if len(self.analysis_frames) >= self.analysis_frames.maxlen:
                self.dropped_analysis += 1

            self.analysis_frames.append({
                "image": frame_data["image"].copy(),
                "timestamp": frame_data["timestamp"],
                "frame_id": frame_data["frame_id"],
            })

    def get_frame_for_analysis(self):
        """
        Return the oldest frame waiting for tracking.

        Frames are popped in FIFO order so the pipeline does not intentionally
        skip analysis frames.
        """
        with self.analysis_lock:
            if self.analysis_frames:
                return self.analysis_frames.popleft()
            return None

    def get_analysis_buffer_size(self):
        """
        Return the number of frames currently waiting for tracking.
        """
        with self.analysis_lock:
            return len(self.analysis_frames)

    def get_display_buffer_size(self):
        """
        Return the legacy display-buffer size.
        """
        with self.display_lock:
            return len(self.display_frames)

    def get_stats(self):
        """
        Return frame-buffer statistics.
        """
        return {
            "analysis_size": self.get_analysis_buffer_size(),
            "display_size": self.get_display_buffer_size(),
            "total_frames": self.total_frames,
            "dropped_analysis": self.dropped_analysis,
            "dropped_display": self.dropped_display,
        }


class FileVideoStreamLive(Thread):
    """
    Basler acquisition thread.

    The thread grabs camera frames continuously, copies them into the analysis
    FIFO buffer, and sends a reduced-rate copy to the display queue.
    """

    def __init__(self, video, queueSize=500):
        """
        Create the acquisition thread.

        Parameters
        ----------
        video : object
            Shared video state containing the opened Basler camera.
        queueSize : int
            Maximum number of frames stored for analysis.
        """
        super().__init__()
        self.daemon = True

        self.camera = video.device
        self.video = video
        self.framerate = self.video.device.ResultingFrameRate.GetValue()
        self.lastTime_live = 0

        self.running = False
        self.frame_count = 0
        self.acquisition_fps = 0
        self.last_fps_time = time.perf_counter()
        self.fps_counter = 0

        self.camera.Open()
        frame_shape = self._read_frame_shape()

        self.frame_buffer = SimpleFrameBuffer(
            analysis_maxsize=queueSize,
            display_maxsize=2,
        )

        self.display_queue = queue.Queue(maxsize=5)
        self._temp_frame = np.zeros(frame_shape, dtype=np.uint8)

    def _read_frame_shape(self):
        """
        Read one frame to pre-allocate the acquisition buffer shape.
        """
        self.camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
        grab_result = self.camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)

        if grab_result.GrabSucceeded():
            frame_shape = grab_result.Array.shape
            grab_result.Release()
        else:
            frame_shape = (1024, 1280)

        self.camera.StopGrabbing()
        return frame_shape

    def run(self):
        """
        Main Basler acquisition loop.
        """
        display_freq = max(1, int(self.framerate / 25))

        self.camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
        self.running = True

        while self.running:
            try:
                grab_result = self.camera.RetrieveResult(50, pylon.TimeoutHandling_ThrowException)

                if grab_result.GrabSucceeded():
                    self._handle_grabbed_frame(grab_result.Array, display_freq)

                grab_result.Release()

            except Exception as exc:
                print(f"Acquisition error: {exc}")
                time.sleep(0.01)

        self.camera.StopGrabbing()
        print("Acquisition stopped")

    def _handle_grabbed_frame(self, frame_array, display_freq):
        """
        Copy a grabbed frame to the analysis buffer and display queue.
        """
        np.copyto(self._temp_frame, frame_array)
        timestamp = time.perf_counter()

        frame_data = {
            "image": self._temp_frame,
            "timestamp": timestamp,
            "frame_id": self.frame_count,
        }

        self.frame_buffer.add_frame(frame_data, for_analysis=True)

        if self.frame_count % display_freq == 0:
            self._push_display_frame(timestamp)

        self.frame_count += 1
        self.fps_counter += 1
        self._update_live_fps()

    def _push_display_frame(self, timestamp):
        """
        Keep only the latest frame for the live display path.
        """
        try:
            with self.frame_buffer.display_lock:
                while self.display_queue.qsize() >= 1:
                    try:
                        self.display_queue.get_nowait()
                    except queue.Empty:
                        break

                self.display_queue.put_nowait({
                    "image": self._temp_frame.copy(),
                    "timestamp": timestamp,
                    "frame_id": self.frame_count,
                })
        except queue.Full:
            pass

    def _update_live_fps(self):
        """
        Update the smoothed measured live frame rate stored in the video state.
        """
        now = time.time()
        dt = now - self.lastTime_live + 0.0000001
        self.lastTime_live = now

        if self.video.measuredLivefps is None:
            self.video.measuredLivefps = 1.0 / dt
            return

        smoothing = np.clip(dt * 3.0, 0, 1)
        self.video.measuredLivefps = (
            self.video.measuredLivefps * (1 - smoothing)
            + (1.0 / dt) * smoothing
        )

    def get_frame_for_analysis(self):
        """
        Return the next frame waiting for analysis.
        """
        return self.frame_buffer.get_frame_for_analysis()

    def get_buffer_usage(self):
        """
        Return the analysis-buffer fill ratio in percent.
        """
        analysis_size = self.frame_buffer.get_analysis_buffer_size()
        max_size = self.frame_buffer.analysis_frames.maxlen

        if max_size is None or max_size == 0:
            return 0

        return (analysis_size / max_size) * 100

    def get_buffer_stats(self):
        """
        Return detailed buffer statistics.
        """
        return self.frame_buffer.get_stats()

    def stop(self):
        """
        Request the acquisition loop to stop.
        """
        self.running = False


class UIVideoCapture(pg.LayoutWidget):
    """
    Camera-control widget shown in the ``video Capture`` dock.
    """

    def __init__(self, videoDisplay_Widget, video, display_updater, parent=None):
        """
        Build the live camera controls.

        Parameters
        ----------
        videoDisplay_Widget : object
            Image display widget used to show live frames.
        video : object
            Shared video state.
        display_updater : object
            Optional display updater currently attached to the live view.
        parent : QWidget, optional
            Qt parent widget.
        """
        super().__init__()

        self.acquisition_thread = None
        self.video = video
        self.videoDisplayer = videoDisplay_Widget
        self.videoDisplayer_updater = display_updater

        self.frame_count = 0
        self.fps_counter = 0

        self.setup_ui()

    def setup_ui(self):
        """
        Create and place all camera-control widgets.
        """
        self.connectCam_btn = QtWidgets.QPushButton("Connect camera")
        self.connectCam_btn.setMaximumWidth(150)
        self.connectCam_btn.setMinimumWidth(100)
        self.connectCam_btn.setCheckable(True)
        self.connectCam_btn.setChecked(False)
        self.connectCam_btn.clicked.connect(self.connect_disconnect)

        self.videoName_label = QtWidgets.QLabel("No camera connected")
        self.videoName_label.setMaximumWidth(150)

        splitter_connect = QtWidgets.QSplitter()
        splitter_connect.setOrientation(QtCore.Qt.Horizontal)
        splitter_connect.setMaximumWidth(150)
        splitter_connect.addWidget(self.connectCam_btn)
        splitter_connect.addWidget(self.videoName_label)

        self.liveVideo_btn = QtWidgets.QPushButton("Live")
        self.liveVideo_btn.setCheckable(True)
        self.liveVideo_btn.setChecked(False)
        self.liveVideo_btn.setMaximumWidth(150)
        self.liveVideo_btn.setMinimumWidth(100)
        self.liveVideo_btn.setEnabled(False)
        self.liveVideo_btn.clicked.connect(self.start_stop_acquisition_toggle)

        self.measuredLivefps_Label = QtWidgets.QLabel(" 00 fps ")
        self.measuredLivefps_Label.setMaximumWidth(40)
        self.measuredLivefps_Label.setMinimumWidth(40)

        splitter_live = QtWidgets.QSplitter()
        splitter_live.setOrientation(QtCore.Qt.Horizontal)
        splitter_live.setMaximumWidth(150)
        splitter_live.addWidget(self.liveVideo_btn)
        splitter_live.addWidget(self.measuredLivefps_Label)

        self.trigger_checkBox = QtWidgets.QCheckBox("Trigger")
        self.trigger_checkBox.setMaximumWidth(150)

        availableMemory_label = QtWidgets.QLabel("Available memory (MB) : ")
        availableMemory_label.setMinimumWidth(100)
        availableMemory_label.setMaximumWidth(120)

        self.availableMemValue_label = QtWidgets.QLabel("00")
        self.availableMemValue_label.setMaximumWidth(100)
        self.availableMemValue_label.setMinimumWidth(40)

        bufferSize_label = QtWidgets.QLabel("Camera buffer")
        bufferSize_label.setMinimumWidth(100)
        bufferSize_label.setMaximumWidth(120)
        bufferSize_label.setToolTip(
            "Camera buffer: maximum number of frames stored on the camera acquisition side."
        )

        self.bufferSizeValue_label = QtWidgets.QLabel("000")
        self.bufferSizeValue_label.setMinimumWidth(20)
        self.bufferSizeValue_label.setMaximumWidth(40)
        self.bufferSizeValue_label.setAlignment(QtCore.Qt.AlignCenter)

        self.bufferSizeValMB_label = QtWidgets.QLabel("00MB")
        self.bufferSizeValMB_label.setMinimumWidth(20)
        self.bufferSizeValMB_label.setMaximumWidth(40)
        self.bufferSizeValMB_label.setAlignment(QtCore.Qt.AlignCenter)

        self.bufferSizeFrames_spinbox = QtWidgets.QSpinBox()
        self.bufferSizeFrames_spinbox.setMaximumWidth(80)
        self.bufferSizeFrames_spinbox.setMinimum(1)
        self.bufferSizeFrames_spinbox.setMaximum(2000)
        self.bufferSizeFrames_spinbox.setValue(10)
        self.bufferSizeFrames_spinbox.valueChanged.connect(self.bufferSizeMB_update)
        self.bufferSizeFrames_spinbox.setToolTip(
            "Maximum number of frames stored in the camera buffer before analysis."
        )

        pipelineBuffer_label = QtWidgets.QLabel("Pipeline buffers")
        pipelineBuffer_label.setMinimumWidth(90)
        pipelineBuffer_label.setMaximumWidth(95)

        pipelineBuffer_help = QtWidgets.QLabel("?")
        pipelineBuffer_help.setMinimumWidth(16)
        pipelineBuffer_help.setMaximumWidth(16)
        pipelineBuffer_help.setAlignment(QtCore.Qt.AlignCenter)
        pipelineBuffer_help.setToolTip(
            "Pipeline buffers:\n"
            "T = Tracking buffer: frames awaiting analysis.\n"
            "R = Result buffer: results awaiting CSV writing.\n"
            "D = Display buffer: elements kept for display.\n\n"
            "These values are used when tracking starts."
        )

        trackingBufferShort_label = QtWidgets.QLabel("T:")
        trackingBufferShort_label.setToolTip("Tracking buffer: frames awaiting analysis.")

        self.trackingBufferFrames_spinbox = QtWidgets.QSpinBox()
        self.trackingBufferFrames_spinbox.setMaximumWidth(60)
        self.trackingBufferFrames_spinbox.setMinimum(1)
        self.trackingBufferFrames_spinbox.setMaximum(5000)
        self.trackingBufferFrames_spinbox.setValue(500)
        self.trackingBufferFrames_spinbox.setToolTip(
            "Maximum number of frames awaiting analysis."
        )

        resultBufferShort_label = QtWidgets.QLabel("R:")
        resultBufferShort_label.setToolTip("Result buffer: results awaiting CSV writing.")

        self.resultBufferFrames_spinbox = QtWidgets.QSpinBox()
        self.resultBufferFrames_spinbox.setMaximumWidth(60)
        self.resultBufferFrames_spinbox.setMinimum(1)
        self.resultBufferFrames_spinbox.setMaximum(5000)
        self.resultBufferFrames_spinbox.setValue(500)
        self.resultBufferFrames_spinbox.setToolTip(
            "Maximum number of results awaiting CSV writing."
        )

        displayBufferShort_label = QtWidgets.QLabel("D:")
        displayBufferShort_label.setToolTip("Display buffer: elements kept for display.")

        self.displayBufferFrames_spinbox = QtWidgets.QSpinBox()
        self.displayBufferFrames_spinbox.setMaximumWidth(50)
        self.displayBufferFrames_spinbox.setMinimum(1)
        self.displayBufferFrames_spinbox.setMaximum(100)
        self.displayBufferFrames_spinbox.setValue(5)
        self.displayBufferFrames_spinbox.setToolTip(
            "Maximum number of elements kept for display."
        )

        self.buffer_progress = QtWidgets.QProgressBar()
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

        splitterPipelineBuffer = QtWidgets.QSplitter()
        splitterPipelineBuffer.setOrientation(QtCore.Qt.Horizontal)
        splitterPipelineBuffer.setMaximumWidth(300)
        splitterPipelineBuffer.addWidget(pipelineBuffer_label)
        splitterPipelineBuffer.addWidget(pipelineBuffer_help)
        splitterPipelineBuffer.addWidget(trackingBufferShort_label)
        splitterPipelineBuffer.addWidget(self.trackingBufferFrames_spinbox)
        splitterPipelineBuffer.addWidget(resultBufferShort_label)
        splitterPipelineBuffer.addWidget(self.resultBufferFrames_spinbox)
        splitterPipelineBuffer.addWidget(displayBufferShort_label)
        splitterPipelineBuffer.addWidget(self.displayBufferFrames_spinbox)

        splitterBufProgress = QtWidgets.QSplitter()
        splitterBufProgress.setOrientation(QtCore.Qt.Horizontal)
        splitterBufProgress.setMaximumWidth(150)
        splitterBufProgress.addWidget(self.buffer_progress)
        splitterBufProgress.addWidget(self.lenBuffer_label)
        splitterBufProgress.setEnabled(False)

        self.line1 = QtWidgets.QFrame()
        self.line1.width = 20
        self.line1.midLineWidth = 10
        self.line1.minimumWidth = 10
        self.line1.setMaximumWidth(20)
        self.line1.setFrameShape(QtWidgets.QFrame.VLine)

        self.line2 = QtWidgets.QFrame()
        self.line2.width = 20
        self.line2.midLineWidth = 10
        self.line2.minimumWidth = 10
        self.line2.setMaximumWidth(20)
        self.line2.setFrameShape(QtWidgets.QFrame.VLine)

        self._create_camera_parameter_controls()
        self._place_widgets(
            splitter_connect,
            splitter_live,
            splitterMemory,
            splitterBuffer,
            splitterPipelineBuffer,
            splitterBufProgress,
        )

    def _create_camera_parameter_controls(self):
        """
        Create Basler camera parameter widgets.
        """
        self.pixFormat_label = QtWidgets.QLabel("pixel format")
        self.pixFormat_label.setMinimumWidth(30)
        self.pixFormat_label.setMaximumWidth(200)

        self.pixFormat_comboBox = QtWidgets.QComboBox()
        self.pixFormat_comboBox.setMaximumWidth(120)
        self.pixFormat_comboBox.addItem("Mono8")
        self.pixFormat_comboBox.addItem("BGR8")
        self.pixFormat_comboBox.currentTextChanged.connect(self.cam_change_pixelFormat)

        self.sensorReadMode_label = QtWidgets.QLabel("sensor read out mode")
        self.sensorReadMode_label.setMinimumWidth(30)
        self.sensorReadMode_label.setMaximumWidth(200)

        self.sensorReadMode_comboBox = QtWidgets.QComboBox()
        self.sensorReadMode_comboBox.setMinimumWidth(100)
        self.sensorReadMode_comboBox.setMaximumWidth(120)
        self.sensorReadMode_comboBox.addItem("Normal")
        self.sensorReadMode_comboBox.addItem("Fast")
        self.sensorReadMode_comboBox.currentTextChanged.connect(self.cam_change_sensorReadMode)

        self.frameWidth_label = QtWidgets.QLabel("frame width")
        self.frameWidth_label.setMinimumWidth(30)
        self.frameWidth_label.setMaximumWidth(200)
        self.frameWidth_slider = StepSlider(32, 1952, 32)
        self.frameWidth_slider.setOrientation(QtCore.Qt.Horizontal)
        self.frameWidth_slider.setMinimumWidth(30)
        self.frameWidth_slider.setMaximumWidth(200)

        self.frameWidthValue_label = QtWidgets.QLabel("00")
        self.frameWidthValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.frameWidthValue_label.setMaximumWidth(60)
        self.frameWidth_slider.valueChanged.connect(self.width_value_by_step)

        self.splitterFrameWidth = QtWidgets.QSplitter()
        self.splitterFrameWidth.setOrientation(QtCore.Qt.Horizontal)
        self.splitterFrameWidth.setMaximumWidth(150)
        self.splitterFrameWidth.addWidget(self.frameWidthValue_label)
        self.splitterFrameWidth.addWidget(self.frameWidth_slider)

        self.frameHeight_label = QtWidgets.QLabel("frame height")
        self.frameHeight_label.setMinimumWidth(30)
        self.frameHeight_label.setMaximumWidth(200)
        self.frameHeight_slider = StepSlider(2, 1232, 2)
        self.frameHeight_slider.setOrientation(QtCore.Qt.Horizontal)
        self.frameHeight_slider.setMinimumWidth(30)
        self.frameHeight_slider.setMaximumWidth(200)

        self.frameHeightValue_label = QtWidgets.QLabel("00")
        self.frameHeightValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.frameHeightValue_label.setMaximumWidth(60)
        self.frameHeight_slider.valueChanged.connect(self.height_value_by_step)

        self.splitterFrameHeight = QtWidgets.QSplitter()
        self.splitterFrameHeight.setOrientation(QtCore.Qt.Horizontal)
        self.splitterFrameHeight.setMaximumWidth(150)
        self.splitterFrameHeight.addWidget(self.frameHeightValue_label)
        self.splitterFrameHeight.addWidget(self.frameHeight_slider)

        self.resultFPS_label = QtWidgets.QLabel("resulting frame rate")
        self.resultFPS_label.setMaximumWidth(100)

        self.resultFPSValue_label = QtWidgets.QLabel("00")
        self.resultFPSValue_label.setMinimumWidth(30)
        self.resultFPSValue_label.setMaximumWidth(100)

        self.frameRate_label = QtWidgets.QLabel("frame rate :")
        self.frameRate_text = QtWidgets.QLineEdit()
        self.frameRate_text.setMinimumWidth(30)
        self.frameRate_text.setMaximumWidth(60)
        self.frameRate_text.returnPressed.connect(self.cam_change_framerate)

        self.splitterFrameRate = QtWidgets.QSplitter()
        self.splitterFrameRate.setOrientation(QtCore.Qt.Horizontal)
        self.splitterFrameRate.setMaximumWidth(150)
        self.splitterFrameRate.addWidget(self.frameRate_text)
        self.splitterFrameRate.addWidget(self.resultFPS_label)
        self.splitterFrameRate.addWidget(self.resultFPSValue_label)

        self.exposureTime_label = QtWidgets.QLabel("exposure time :")
        self.exposureTime_text = QtWidgets.QLineEdit()
        self.exposureTime_text.setMinimumWidth(30)
        self.exposureTime_text.setMaximumWidth(60)
        self.exposureTime_text.returnPressed.connect(self.cam_change_exposureTime)

        self.offsetX_label = QtWidgets.QLabel("offset x")
        self.offsetX_label.setMinimumWidth(30)
        self.offsetX_label.setMaximumWidth(200)
        self.offsetX_slider = StepSlider(0, 96, 32)
        self.offsetX_slider.setOrientation(QtCore.Qt.Horizontal)
        self.offsetX_slider.setMinimumWidth(30)
        self.offsetX_slider.setMaximumWidth(200)

        self.offsetXValue_label = QtWidgets.QLabel("00")
        self.offsetXValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.offsetXValue_label.setMaximumWidth(60)
        self.offsetX_slider.valueChanged.connect(self.offsetX_value_by_step)

        self.splitterOffsetX = QtWidgets.QSplitter()
        self.splitterOffsetX.setOrientation(QtCore.Qt.Horizontal)
        self.splitterOffsetX.setMaximumWidth(150)
        self.splitterOffsetX.addWidget(self.offsetXValue_label)
        self.splitterOffsetX.addWidget(self.offsetX_slider)

        self.offsetY_label = QtWidgets.QLabel("offset y")
        self.offsetY_label.setMinimumWidth(30)
        self.offsetY_label.setMaximumWidth(200)
        self.offsetY_slider = StepSlider(0, 64, 2)
        self.offsetY_slider.setOrientation(QtCore.Qt.Horizontal)
        self.offsetY_slider.setMinimumWidth(30)
        self.offsetY_slider.setMaximumWidth(200)

        self.offsetYValue_label = QtWidgets.QLabel("00")
        self.offsetYValue_label.setAlignment(QtCore.Qt.AlignCenter)
        self.offsetYValue_label.setMaximumWidth(60)
        self.offsetY_slider.valueChanged.connect(self.offsetY_value_by_step)

        self.splitterOffsetY = QtWidgets.QSplitter()
        self.splitterOffsetY.setOrientation(QtCore.Qt.Horizontal)
        self.splitterOffsetY.setMaximumWidth(150)
        self.splitterOffsetY.addWidget(self.offsetYValue_label)
        self.splitterOffsetY.addWidget(self.offsetY_slider)

    def _place_widgets(
        self,
        splitter_connect,
        splitter_live,
        splitterMemory,
        splitterBuffer,
        splitterPipelineBuffer,
        splitterBufProgress,
    ):
        """
        Place the controls in the pyqtgraph layout widget.
        """
        self.addWidget(splitter_connect, row=0, col=0)
        self.addWidget(splitter_live, row=2, col=0)
        self.addWidget(self.trigger_checkBox, row=3, col=0)
        self.addWidget(splitterMemory, row=4, col=0)
        self.addWidget(splitterBuffer, row=5, col=0)
        self.addWidget(splitterPipelineBuffer, row=6, col=0)
        self.addWidget(splitterBufProgress, row=7, col=0)

        self.addWidget(self.line1, row=1, col=1, rowspan=7)

        self.addWidget(self.pixFormat_label, row=1, col=2)
        self.addWidget(self.pixFormat_comboBox, row=1, col=3)
        self.addWidget(self.sensorReadMode_label, row=2, col=2)
        self.addWidget(self.sensorReadMode_comboBox, row=2, col=3)

        self.addWidget(self.frameRate_label, row=4, col=2)
        self.addWidget(self.splitterFrameRate, row=4, col=3)
        self.addWidget(self.exposureTime_label, row=5, col=2)
        self.addWidget(self.exposureTime_text, row=5, col=3)

        self.addWidget(self.line2, row=1, col=4, rowspan=5)

        self.addWidget(self.frameWidth_label, row=2, col=6)
        self.addWidget(self.splitterFrameWidth, row=2, col=7)
        self.addWidget(self.frameHeight_label, row=3, col=6)
        self.addWidget(self.splitterFrameHeight, row=3, col=7)
        self.addWidget(self.offsetX_label, row=4, col=6)
        self.addWidget(self.splitterOffsetX, row=4, col=7)
        self.addWidget(self.offsetY_label, row=5, col=6)
        self.addWidget(self.splitterOffsetY, row=5, col=7)

    def width_value_by_step(self):
        """
        Apply the selected camera width and refresh dependent values.
        """
        self.cam_change()

        value = self.frameWidth_slider.value()
        self.frameWidthValue_label.setNum(value)

        try:
            if genicam.IsWritable(self.video.device.Width):
                self.video.device.Width.SetValue(value)
            else:
                print("Width is not writable now: camera is active or parameter is locked")
                return

            self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
            self.offsetX_slider.setMaximum(self.video.device.OffsetX.Max)
            self.offsetX_slider.setMinimum(self.video.device.OffsetX.Min)

        except genicam.GenericException as exc:
            print("Width update error:", exc)
            QtWidgets.QMessageBox.warning(None, "Error", str(exc), QtWidgets.QMessageBox.Ok)

    def height_value_by_step(self):
        """
        Apply the selected camera height and refresh dependent values.
        """
        self.cam_change()
        self.frameHeightValue_label.setNum(self.frameHeight_slider.value())
        self.video.device.Height.SetValue(self.frameHeight_slider.value())
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
        self.offsetY_slider.setMaximum(self.video.device.OffsetY.Max)
        self.offsetY_slider.setMinimum(self.video.device.OffsetY.Min)

    def offsetX_value_by_step(self):
        """
        Apply the selected horizontal camera offset.
        """
        self.offsetXValue_label.setNum(self.offsetX_slider.value())
        self.video.device.OffsetX.SetValue(self.offsetX_slider.value())

    def offsetY_value_by_step(self):
        """
        Apply the selected vertical camera offset.
        """
        self.offsetYValue_label.setNum(self.offsetY_slider.value())
        self.video.device.OffsetY.SetValue(self.offsetY_slider.value())

    def cam_change_sensorReadMode(self, value):
        """
        Apply the selected Basler sensor readout mode.
        """
        self.cam_change()
        self.video.device.SensorReadoutMode.SetValue(value)
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())

    def cam_change_pixelFormat(self, value):
        """
        Apply the selected camera pixel format.
        """
        self.cam_change()
        self.video.device.PixelFormat.SetValue(value)
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())

    def cam_change_framerate(self):
        """
        Apply the target acquisition frame rate entered by the user.
        """
        framerate = int(self.frameRate_text.text())
        self.video.device.AcquisitionFrameRate.SetValue(framerate)
        self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())

    def cam_change_exposureTime(self):
        """
        Apply the target exposure time entered by the user.
        """
        exposure_time = int(self.exposureTime_text.text())

        try:
            self.video.device.ExposureTime.SetValue(exposure_time)
            self.resultFPSValue_label.setNum(self.video.device.ResultingFrameRate.GetValue())
        except genicam.GenericException as exc:
            print("Exposure update error:", str(exc))
            QtWidgets.QMessageBox.warning(None, "Error", str(exc), QtWidgets.QMessageBox.Ok)

    def cam_change(self):
        """
        Stop acquisition before changing camera parameters that lock while grabbing.
        """
        if self.video.device.IsGrabbing():
            self.stop_grabbing()

    def update(self, cam):
        """
        Load current Basler camera parameters into the UI widgets.
        """
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

    def stop_grabbing(self):
        """
        Stop camera acquisition synchronously before updating locked parameters.
        """
        try:
            if self.videoDisplayer_updater is not None:
                if hasattr(self.videoDisplayer_updater, "display_timer"):
                    self.videoDisplayer_updater.display_timer.stop()
        except Exception as exc:
            print("Display timer stop error:", exc)

        try:
            if self.acquisition_thread is not None:
                self.acquisition_thread.stop()

                if self.acquisition_thread.is_alive():
                    self.acquisition_thread.join(timeout=2.0)

                self.acquisition_thread = None
        except Exception as exc:
            print("Acquisition thread stop error:", exc)

        try:
            if self.video.device is not None:
                if self.video.device.IsGrabbing():
                    self.video.device.StopGrabbing()
        except Exception as exc:
            print("Camera StopGrabbing error:", exc)

        time.sleep(0.05)
        print("stop grabbing")

    def connect_disconnect(self):
        """
        Connect to or disconnect from the Basler camera.
        """
        if self.connectCam_btn.isChecked() is True:
            self.init_acquisitionDevice()
            self.connectCam_btn.setText("disconnect camera")
        else:
            self.disconnect_cam()
            self.connectCam_btn.setText("connect camera")

    def init_acquisitionDevice(self):
        """
        Detect and open the first available Basler camera.
        """
        print("connecting device...")

        try:
            self.video.devices = pylon.TlFactory.GetInstance().EnumerateDevices()
            print(f"devices: {self.video.devices}")

            info = pylon.DeviceInfo()
            info.SetDeviceClass("BaslerUsb")

            self.video.device = pylon.InstantCamera(
                pylon.TlFactory.GetInstance().CreateFirstDevice()
            )
            print("Connected device:", self.video.device.GetDeviceInfo().GetModelName())
            self.videoName_label.setText(self.video.device.GetDeviceInfo().GetModelName())

            self.video.device.Open()
            self.set_default_cam_parameters(self.video.device)
            self.get_cam_parameters(self.video.device)
            self.update(self.video.device)
            self.activate_interface("live video")

        except genicam.GenericException as exc:
            print("Camera connection error:", str(exc))
            QtWidgets.QMessageBox.warning(None, "Error", str(exc), QtWidgets.QMessageBox.Ok)

    def get_cam_parameters(self, cam):
        """
        Read the current acquisition parameters from the Basler camera.
        """
        print("Saving camera node map to file...")
        self.video.featuresFile = "NodeMap.pfs"
        print(self.video.featuresFile)
        pylon.FeaturePersistence.Save(self.video.featuresFile, cam.GetNodeMap())

        self.video.pixFormat = cam.PixelFormat.GetValue()
        self.video.width = cam.Width.GetValue()
        self.video.height = cam.Height.GetValue()
        self.video.grabFrameRate = cam.AcquisitionFrameRate.GetValue()
        self.video.offsetX = cam.OffsetX.GetValue()
        self.video.offsetY = cam.OffsetY.GetValue()
        self.video.centerX = cam.CenterX.GetValue()
        self.video.centerY = cam.CenterY.GetValue()
        self.video.gainAuto = cam.GainAuto.GetValue()
        self.video.gain = cam.Gain.GetValue()
        self.video.gamma = cam.Gamma.GetValue()
        self.video.LUT = image_processor.gamma_LUT(1)
        self.video.exposure = cam.ExposureTime.GetValue()
        self.video.sensorReadoutMode = cam.SensorReadoutMode.GetValue()

    def set_default_cam_parameters(self, cam):
        """
        Apply application defaults to a newly opened camera.
        """
        print("set_cam_parameters")

        if not cam.IsOpen():
            print("No camera found!")
            return

        if cam.IsGrabbing():
            self.stop_grabbing()

        cam.AcquisitionFrameRateEnable.SetValue(True)
        cam.ExposureAuto.SetValue("Off")
        cam.SensorReadoutMode.SetValue("Fast")

    def start_stop_acquisition_toggle(self):
        """
        Start or stop live acquisition according to the Live button state.
        """
        if self.liveVideo_btn.isChecked() is True:
            self.start_acquisition()
        else:
            self.stop_acquisition()

    def stop_acquisition(self):
        """
        Stop the live acquisition thread and the display timer.
        """
        try:
            if self.acquisition_thread:
                self.acquisition_thread.stop()
        except Exception as exc:
            print("Acquisition thread stop error:", exc)

        try:
            if self.videoDisplayer_updater is not None:
                if hasattr(self.videoDisplayer_updater, "display_timer"):
                    self.videoDisplayer_updater.display_timer.stop()
        except Exception as exc:
            print("Display timer stop error:", exc)

        try:
            if self.video.device is not None and self.video.device.IsGrabbing():
                self.video.device.StopGrabbing()
        except Exception as exc:
            print("StopGrabbing error:", exc)

        print("stop grabbing")

    def start_acquisition(self):
        """
        Start live acquisition and low-rate live display.
        """
        camera_buffer_size = self.bufferSizeFrames_spinbox.value()
        self.acquisition_thread = FileVideoStreamLive(self.video, camera_buffer_size)
        self.acquisition_thread.start()

        self.videoDisplayer_updater = Display(
            self,
            self.acquisition_thread,
            self.videoDisplayer,
            self.video,
        )
        self.videoDisplayer_updater.setup_display_timer()

    def bufferSizeMB_update(self):
        """
        Refresh the estimated camera-buffer memory size.
        """
        if self.video.frameWeight > 0:
            size_mb = round(self.bufferSizeFrames_spinbox.value() * self.video.frameWeight, 2)
            self.bufferSizeValMB_label.setText(str(size_mb) + "MB")

    def activate_interface(self, context):
        """
        Enable UI controls after a valid camera connection.
        """
        if context == "live video":
            self.liveVideo_btn.setEnabled(True)

    def disconnect_cam(self):
        """
        Close the current Basler camera connection.
        """
        self.video.device.Close()
        print("camera connected:", self.video.device.IsOpen())


class Display(QtWidgets.QWidget):
    """
    Low-rate live display updater.

    Acquisition and tracking can run at high frame rate while this timer updates
    the image view at about 25 fps.
    """

    def __init__(self, capture, acquisition_thread, videoDisplay_Widget, video):
        """
        Create the display updater.

        Parameters
        ----------
        capture : UIVideoCapture
            Camera control widget.
        acquisition_thread : FileVideoStreamLive
            Active acquisition thread.
        videoDisplay_Widget : object
            Image display widget.
        video : object
            Shared video state.
        """
        super().__init__()
        self.capture = capture
        self.acquisition = acquisition_thread
        self.video = video
        self.show_overlay = False
        self.display_fps = 0
        self.last_fps_time = time.time()
        self.fps_counter = 0
        self.videoDisplayer = videoDisplay_Widget
        self.current_frame_to_display = None
        self.display_lock = Lock()

    def setup_display_timer(self):
        """
        Configure the Qt timer used for 25 fps live display.
        """
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_display)
        self.display_timer.start(40)

    def update_display(self):
        """
        Show the latest display frame and update live FPS/buffer indicators.
        """
        try:
            with self.display_lock:
                display_data = self.acquisition.display_queue.get_nowait()
                frame = display_data["image"]

                self.current_frame_to_display = frame.copy()
                tLive = image_processor.convert_imageToPyqtgraph(frame, self.video)
                self.videoDisplayer.img.setImage(tLive, autoLevels=False)

                self.fps_counter += 1
                if time.time() - self.last_fps_time >= 1.0:
                    if hasattr(self.capture, "measuredLivefps_Label"):
                        self.capture.measuredLivefps_Label.setText(
                            f"{self.video.measuredLivefps:.2f} fps"
                        )
                    self.fps_counter = 0
                    self.last_fps_time = time.time()

                if hasattr(self.capture, "buffer_progress"):
                    buffer_usage = self.acquisition.get_buffer_usage()
                    self.capture.buffer_progress.setValue(int(buffer_usage))

        except queue.Empty:
            pass
        except Exception as exc:
            print(f"Display error: {exc}")


class StepSlider(QtWidgets.QSlider):
    """
    Slider that maps each internal integer index to a fixed external step value.
    """

    def __init__(self, minValue, maxValue, interval):
        """
        Create a stepped slider.

        Parameters
        ----------
        minValue : int
            Minimum real value.
        maxValue : int
            Maximum real value.
        interval : int
            Step size between two adjacent real values.
        """
        super(StepSlider, self).__init__()
        self._min = minValue
        self._max = maxValue
        self.interval = interval
        self._range_adjusted()

    def setValue(self, value):
        """
        Set the slider from a real value instead of an internal index.
        """
        index = round((value - self._min) / self.interval)
        return super(StepSlider, self).setValue(index)

    def value(self):
        """
        Return the real value represented by the current internal index.
        """
        return self.index * self.interval + self._min

    @property
    def index(self):
        """
        Return the internal Qt slider index.
        """
        return super(StepSlider, self).value()

    def setIndex(self, index):
        """
        Set the internal Qt slider index directly.
        """
        return super(StepSlider, self).setValue(index)

    def setMinimum(self, value):
        """
        Set the minimum real value.
        """
        self._min = value
        self._range_adjusted()

    def setMaximum(self, value):
        """
        Set the maximum real value.
        """
        self._max = value
        self._range_adjusted()

    def setInterval(self, value):
        """
        Set the real-value step interval.
        """
        if not value:
            raise ValueError("Interval of zero specified")

        self.interval = value
        self._range_adjusted()

    def _range_adjusted(self):
        """
        Recompute the internal Qt slider range from the real-value range.
        """
        number_of_steps = int(math.ceil((self._max - self._min) / self.interval))
        super(StepSlider, self).setMaximum(number_of_steps)
