# -*- coding: utf-8 -*-
"""
Main Qt window for the Xenopus tracking application.

This file now only keeps the window construction and application entry point.
The former 4000+ line monolithic file has been split into ui/, rois/,
tracking/, acquisition/, pipeline/, video/, stimulation/, io/ and core/ modules.

@author: Courtand, Kadri 
"""

import os

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import QtWidgets
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui as QtgGui
from pyqtgraph.dockarea import DockArea, Dock

from xenopus_app.constants import APP_TITLE
from xenopus_app.controller.app_controller import AppController
from xenopus_app.video import video_state as video_player
from xenopus_app.acquisition import camera_widget as video_capture
from xenopus_app.core import image_container as analysis
from xenopus_app.rois import define_rois
from xenopus_app.stimulation import optostim as optok

from xenopus_app.core.app_state import Analysis_Settings, Measure_Var, Target, update_settings
from xenopus_app.rois.roi_items import graphMark
from xenopus_app.rois.tail_arc_roi import TailArcROI
from xenopus_app.ui.overlays import RoiOverlayMixin
from xenopus_app.ui.docks import DockModeMixin
from xenopus_app.ui.imported_video_panel import ImportedVideoPanelMixin
from xenopus_app.io.settings_manager import SettingsManagerMixin
from xenopus_app.ui.tracking_controls_panel import TrackingControlsMixin
from xenopus_app.ui.plot_panel import PlotPanelMixin

# Modules that still contain legacy callbacks using shared state.
from xenopus_app.core import app_state as app_state_module
from xenopus_app.tracking import legacy_tracking as legacy_tracking_module
from xenopus_app.rois import tail_arc_roi as tail_arc_roi_module
from xenopus_app.ui import overlays as overlays_module
from xenopus_app.ui import docks as docks_module
from xenopus_app.ui import imported_video_panel as imported_video_panel_module
from xenopus_app.io import settings_manager as settings_manager_module
from xenopus_app.ui import tracking_controls_panel as tracking_controls_panel_module
from xenopus_app.ui import plot_panel as plot_panel_module

# Legacy shared state. It is kept during the refactor to avoid changing behavior.
timestampList = []
roisLimb = []
stimList = []
framesBuffer = []

app = None
analysisSet = None
tracking = None
video = None
image = None
target = None
varM = None
ui = None


class UIXenopus(
    QtWidgets.QMainWindow,
    RoiOverlayMixin,
    DockModeMixin,
    ImportedVideoPanelMixin,
    SettingsManagerMixin,
    TrackingControlsMixin,
    PlotPanelMixin,
):
    """Main Qt window that assembles all Xenopus UI panels and mixins.

    The class keeps only high-level construction code. Most behavior lives in
    extracted mixins and worker modules so that acquisition, tracking, display,
    persistence, and stimulation can evolve independently.
    """

    def __init__(self, parent=None):
        """Create the main window, shared runtime objects, and timers."""
        QtWidgets.QMainWindow.__init__(self, parent=None)

        self.video=video_player.Video()
        self.display_updater=None
        self.videoDisplay_Widget=define_rois.UIVideoDisplayRoi(self.video)
        self.videoDisplay_Widget.proxy1 = pg.SignalProxy(self.videoDisplay_Widget.plotView.scene().sigMouseClicked, rateLimit=60, slot=self.mouse_clicked)
        # Legacy video player removed from the interface.
        # Imported video analysis uses its own frame-by-frame review reader.
        self.videoPlayer_Widget = None

        self.video_capture_widget=video_capture.UIVideoCapture(self.videoDisplay_Widget,self.video,self.display_updater)

        self.tracking = False
        self.analysis_thread = None
        self.result_save_dir = None
        self.controller = None

        # Imported-video interface.
        self.imported_video_path = None
        self.imported_video_frame = None
        self.imported_video_total_frames = 0
        self.imported_video_raw_width = 0
        self.imported_video_raw_height = 0
        self.imported_review_last_auto_frame = -1
        self.imported_review_ignore_signals = False

        # Special layout used by the Imported video mode:
        # camera/player docks are compressed and the image uses the freed space.
        self._live_dock_height_cache = {}
        self._video_mode_image_height = 430

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(300)
        self.plot_timer.timeout.connect(self.update_plot)

        self.roisEye = []
        self.roisEllipseEye=[]
        self.roisEyeLabels=[]
        self.tailAngleList=[]
        self.tailPosList=[]
        self.tailAngleListR=[]
        self.tailAngleListM=[]
        self.tailAngleListC=[]
        self.tailPosListR=[]
        self.tailPosListM=[]
        self.tailPosListC=[]

        self.mark = graphMark()
        self.listTailSegments=[]

        self.eyeAxeLine1 = pg.InfiniteLine(movable=False)
        self.eyeAxeLine2 = pg.InfiniteLine(movable=False)
        self.eyeAxeLines=[self.eyeAxeLine1,self.eyeAxeLine2]

        self._bind_runtime_context()

        self.initUI()

        self.video_file_status_timer = QTimer()
        self.video_file_status_timer.setInterval(250)
        self.video_file_status_timer.timeout.connect(self.update_imported_video_status)

    def initUI(self):
        """Build the dock layout, controls, plots, and imported-video panel."""

        self.setWindowIcon(QtgGui.QIcon(os.path.join('Imagys_blue', 'logoAnimotion-square-112.png')))

        self.area = DockArea()
        self.setCentralWidget(self.area)
        self.resize(1500,800)
        self.setWindowTitle(APP_TITLE)

        # Global analysis-mode selector.
        # It prevents real-time camera and imported-video workflows from being active together.
        self.analysis_mode = "live"
        self.analysisMode_toolbar = self.addToolBar("Analysis mode")
        self.analysisMode_toolbar.setMovable(False)

        self.analysisMode_label = QtWidgets.QLabel("Analysis mode: ")

        # Global tabs at the top of the window.
        # Clearer than a small drop-down: only one mode is active at a time.
        self.analysisMode_tabs = QtWidgets.QTabBar()
        self.analysisMode_tabs.addTab("Real-time camera")
        self.analysisMode_tabs.addTab("Imported video")
        self.analysisMode_tabs.setExpanding(False)
        self.analysisMode_tabs.setDrawBase(False)
        self.analysisMode_tabs.setToolTip(
            "Choose the analysis mode. Only one mode can be active at a time."
        )
        self.analysisMode_tabs.currentChanged.connect(self.on_analysis_mode_changed)

        self.analysisMode_toolbar.addWidget(self.analysisMode_label)
        self.analysisMode_toolbar.addWidget(self.analysisMode_tabs)

        # Save/Load settings buttons are not placed in the global toolbar.
        # They are placed in the panel for the active mode:
        # - Real-time camera: video Capture dock;
        # - Imported video: Imported video progress dock.

        # Toolbar reserved for imported-video mode.
        # It is hidden in real-time mode so both modes are not visible at once.
        self.importedVideo_toolbar = self.addToolBar("Imported video tools")
        self.importedVideo_toolbar.setMovable(False)
        self.importedVideo_toolbar.hide()

        self.penCyan=pg.mkPen((0,255,255), width=2)
        self.penOrange=pg.mkPen((255,128,0), width=2)
        self.penGreen=pg.mkPen((0,255,0), width=2)

        # HSV-like colors for the three tail arcs.
        self.penTailR=pg.mkPen((255,0,0), width=2)
        self.penTailM=pg.mkPen((0,255,0), width=2)
        self.penTailC=pg.mkPen((0,80,255), width=2)

        self.d1 = Dock("Image", size=(1000,500))

        self.d2 = Dock("Layout preferences", size=(500,200))

        self.d3 = Dock("Eye 1", size=(500,200))
        self.d4 = Dock("Eye 2", size=(500,200))
        self.d5 = Dock("Tails", size=(500,200))
        self.d6 = Dock("Eye 1-Y", size=(500,200))
        self.d7 = Dock("Eye 2-Y", size=(500,200))
        self.d8 = Dock("Regions of interest", size=(500,200))
        # Video player dock removed: it could sometimes merge with video Capture after Load settings.
        self.d9 = None
        self.d10 = Dock("Segmentation settings", size=(500,200))
        self.d11 = Dock("video Capture ",size=(500,200))
        self.d13 = Dock("Optokinetic", size=(500,200))
        self.d14 = Dock("Video file analysis", size=(500,200))
        self.d15 = Dock("Imported video progress", size=(1000,232))

        self.area.addDock(self.d1, 'left')
        self.area.addDock(self.d2, 'bottom', self.d1)
        self.area.addDock(self.d3, 'right')
        self.area.addDock(self.d4, 'bottom', self.d3)
        self.area.addDock(self.d5, 'bottom', self.d4)
        self.area.addDock(self.d6, 'bottom', self.d5)
        self.area.addDock(self.d7, 'bottom', self.d6)

        self.area.addDock(self.d15, 'bottom', self.d1)
        self.area.addDock(self.d11, 'bottom', self.d15)
        self.area.addDock(self.d10, 'above', self.d2)
        self.area.addDock(self.d8, 'above', self.d10)
        self.area.addDock(self.d13, 'above', self.d2)
        # Dock d14 is no longer added to the DockArea.
        # Imported-video controls now live in a dedicated panel,
        # visible only in Imported video mode.

        self.d1.addWidget(self.videoDisplay_Widget)

        self.videoDisplay_Widget.plotView.addItem(self.mark)

        self.regionlr = pg.LinearRegionItem([0, 0], bounds=[0,0], movable=True)

        self.videoDisplay_Widget.plotView.addItem(self.regionlr)
        self.regionlr.setVisible(False)

        self.tailArcROI_R = TailArcROI(
            self.videoDisplay_Widget.plotView,
            label="R",
            color=(255, 0, 0),
            band_index=0
        )
        self.tailArcROI_M = TailArcROI(
            self.videoDisplay_Widget.plotView,
            label="M",
            color=(0, 255, 0),
            band_index=1
        )
        self.tailArcROI_C = TailArcROI(
            self.videoDisplay_Widget.plotView,
            label="C",
            color=(0, 80, 255),
            band_index=2
        )
        self.tailArcROIs = {
            "R": self.tailArcROI_R,
            "M": self.tailArcROI_M,
            "C": self.tailArcROI_C,
        }

        # Independent tracking lines and points for each arc.
        # Each arc has its own detected point and root-to-point line.
        self.tailArcTrackingLines = {}
        self.tailArcTrackingPoints = {}
        self.tailArcColors = {
            "R": (255, 0, 0),
            "M": (0, 255, 0),
            "C": (0, 80, 255),
        }

        for label, color in self.tailArcColors.items():
            line_item = pg.PlotDataItem(
                x=[],
                y=[],
                pen=pg.mkPen(color=color, width=2)
            )
            point_item = pg.ScatterPlotItem(
                x=[],
                y=[],
                size=9,
                brush=pg.mkBrush(color),
                pen=pg.mkPen(color='w', width=1),
                pxMode=True
            )

            line_item.setZValue(2300)
            point_item.setZValue(2400)

            self.videoDisplay_Widget.plotView.addItem(line_item)
            self.videoDisplay_Widget.plotView.addItem(point_item)

            self.tailArcTrackingLines[label] = line_item
            self.tailArcTrackingPoints[label] = point_item

        # Compatibility with older code that expects self.tailArcROI.
        self.tailArcROI = self.tailArcROI_R

        self.curveTail =pg.PlotDataItem(x=[], y=[], pen=pg.mkPen(color='#3c02fc'))
        self.videoDisplay_Widget.plotView.addItem(self.curveTail)

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

        self.w3 = pg.PlotWidget(title="Eye 1")

        self.d3.addWidget(self.w3)

        self.w4 = pg.PlotWidget(title="Eye 2")

        self.d4.addWidget(self.w4)

        self.w5 = pg.PlotWidget(title="tails")

        self.d5.addWidget(self.w5)

        self.w6 = pg.PlotWidget(title="Dock 6 plot")

        self.d6.addWidget(self.w6)

        self.w7 = pg.PlotWidget(title="Dock 4 plot")

        self.d7.addWidget(self.w7)

        self.w8 = pg.LayoutWidget()

        self.manipType_comboBox = QtWidgets.QComboBox()
        self.manipType_comboBox.addItem("choose an analysis mode...")
        self.manipType_comboBox.addItem("eyes-tails track")
        self.manipType_comboBox.addItem("eyes track only")

        self.manipType_comboBox.setEnabled(True)

        self.label8 = QtWidgets.QLabel(""" Selection type  """)
        splitter_Selection = QtWidgets.QSplitter()
        splitter_Selection.setOrientation(Qt.Vertical)
        self.selectEyes_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectEyes_radioButton.setToolTip("just click on the eye to trace the roi")
        self.selectEyes_radioButton.setChecked(True)
        self.selectEyes_radioButton.setText("Eye")
        self.selectTailRoot_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectTailRoot_radioButton.setText("Tails root")

        self.selectLimbs_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectLimbs_radioButton.setText("Limb")
        self.selectExclusion_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectExclusion_radioButton.setText("Circle (exclusion)")

        splitter_Background = QtWidgets.QSplitter()
        splitter_Background.setOrientation(Qt.Horizontal)
        self.whiteBgd_radioButton = QtWidgets.QRadioButton(splitter_Background)
        self.whiteBgd_radioButton.setToolTip("background color")
        self.whiteBgd_radioButton.setChecked(True)
        self.whiteBgd_radioButton.setText("White background")

        self.blackBgd_radioButton = QtWidgets.QRadioButton(splitter_Background)
        self.blackBgd_radioButton.setToolTip("background color")
        self.blackBgd_radioButton.setChecked(False)
        self.blackBgd_radioButton.setText("Black background")

        initLimbTrack_btn = QtWidgets.QPushButton('Init limbs')
        initLimbTrack_btn.clicked.connect(self.init_track)
        initLimbTrack_btn.setMinimumHeight(28)

        self.track_checkBox = QtWidgets.QPushButton('Start tracking')
        self.track_checkBox.setCheckable(True)
        self.track_checkBox.setChecked(False)
        self.track_checkBox.clicked.connect(self.toggle_tracking)
        self.track_checkBox.setMinimumHeight(60)
        self.track_checkBox.setStyleSheet("""
            QPushButton {
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:checked {
                background-color: #2f8f46;
                color: white;
            }
        """)

        splitter_ThreshTail = QtWidgets.QWidget()

        # Tail arcs to display and analyze.
        # Default state: R enabled, M and C disabled.
        self.tailArcR_checkBox = QtWidgets.QCheckBox("R", splitter_ThreshTail)
        self.tailArcM_checkBox = QtWidgets.QCheckBox("M", splitter_ThreshTail)
        self.tailArcC_checkBox = QtWidgets.QCheckBox("C", splitter_ThreshTail)
        self.tailArcR_checkBox.setChecked(True)
        self.tailArcM_checkBox.setChecked(False)
        self.tailArcC_checkBox.setChecked(False)

        for checkbox in [self.tailArcR_checkBox, self.tailArcM_checkBox, self.tailArcC_checkBox]:
            checkbox.stateChanged.connect(self.update_tail_arc_enabled_states)

        self.threshTailSlider_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.threshTailSlider_label.setText("R thresh")
        self.threshTailValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.threshTailValue_label.setText("00")
        self.threshTailValue_label.setAlignment(Qt.AlignCenter)
        self.threshTail_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.threshTail_slider.setMaximum(255)
        self.threshTail_slider.setPageStep(10)
        self.threshTail_slider.setOrientation(Qt.Horizontal)
        self.threshTail_slider.valueChanged.connect(self.threshTailValue_label.setNum)
        self.threshTail_slider.valueChanged.connect(self.update_tail_segment_thresh)

        self.threshTailMSlider_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.threshTailMSlider_label.setText("M thresh")
        self.threshTailMValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.threshTailMValue_label.setText("00")
        self.threshTailMValue_label.setAlignment(Qt.AlignCenter)
        self.threshTailM_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.threshTailM_slider.setMaximum(255)
        self.threshTailM_slider.setPageStep(10)
        self.threshTailM_slider.setOrientation(Qt.Horizontal)
        self.threshTailM_slider.valueChanged.connect(self.threshTailMValue_label.setNum)
        self.threshTailM_slider.valueChanged.connect(self.update_tail_segment_thresh)

        self.threshTailCSlider_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.threshTailCSlider_label.setText("C thresh")
        self.threshTailCValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.threshTailCValue_label.setText("00")
        self.threshTailCValue_label.setAlignment(Qt.AlignCenter)
        self.threshTailC_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.threshTailC_slider.setMaximum(255)
        self.threshTailC_slider.setPageStep(10)
        self.threshTailC_slider.setOrientation(Qt.Horizontal)
        self.threshTailC_slider.valueChanged.connect(self.threshTailCValue_label.setNum)
        self.threshTailC_slider.valueChanged.connect(self.update_tail_segment_thresh)

        self.tailArcCurveR_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.tailArcCurveR_label.setText("R curve")
        self.tailArcCurveR_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.tailArcCurveR_slider.setMinimum(0)
        self.tailArcCurveR_slider.setMaximum(100)
        self.tailArcCurveR_slider.setPageStep(5)
        self.tailArcCurveR_slider.setOrientation(Qt.Horizontal)
        self.tailArcCurveR_slider.setValue(40)
        self.tailArcCurveRValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.tailArcCurveRValue_label.setText("40")
        self.tailArcCurveRValue_label.setAlignment(Qt.AlignCenter)
        self.tailArcCurveR_slider.valueChanged.connect(self.tailArcCurveRValue_label.setNum)
        self.tailArcCurveR_slider.valueChanged.connect(self.update_tail_arc_curve_R)

        self.tailArcCurveM_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.tailArcCurveM_label.setText("M curve")
        self.tailArcCurveM_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.tailArcCurveM_slider.setMinimum(0)
        self.tailArcCurveM_slider.setMaximum(100)
        self.tailArcCurveM_slider.setPageStep(5)
        self.tailArcCurveM_slider.setOrientation(Qt.Horizontal)
        self.tailArcCurveM_slider.setValue(40)
        self.tailArcCurveMValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.tailArcCurveMValue_label.setText("40")
        self.tailArcCurveMValue_label.setAlignment(Qt.AlignCenter)
        self.tailArcCurveM_slider.valueChanged.connect(self.tailArcCurveMValue_label.setNum)
        self.tailArcCurveM_slider.valueChanged.connect(self.update_tail_arc_curve_M)

        self.tailArcCurveC_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.tailArcCurveC_label.setText("C curve")
        self.tailArcCurveC_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.tailArcCurveC_slider.setMinimum(0)
        self.tailArcCurveC_slider.setMaximum(100)
        self.tailArcCurveC_slider.setPageStep(5)
        self.tailArcCurveC_slider.setOrientation(Qt.Horizontal)
        self.tailArcCurveC_slider.setValue(40)
        self.tailArcCurveCValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        self.tailArcCurveCValue_label.setText("40")
        self.tailArcCurveCValue_label.setAlignment(Qt.AlignCenter)
        self.tailArcCurveC_slider.valueChanged.connect(self.tailArcCurveCValue_label.setNum)
        self.tailArcCurveC_slider.valueChanged.connect(self.update_tail_arc_curve_C)

        # Compatibility with older code: tailArcCurve_slider points to R.
        self.tailArcCurve_slider = self.tailArcCurveR_slider

        self.tailArcControlWidgets = {
            "R": [
                self.threshTailSlider_label,
                self.threshTailValue_label,
                self.threshTail_slider,
                self.tailArcCurveR_label,
                self.tailArcCurveRValue_label,
                self.tailArcCurveR_slider,
            ],
            "M": [
                self.threshTailMSlider_label,
                self.threshTailMValue_label,
                self.threshTailM_slider,
                self.tailArcCurveM_label,
                self.tailArcCurveMValue_label,
                self.tailArcCurveM_slider,
            ],
            "C": [
                self.threshTailCSlider_label,
                self.threshTailCValue_label,
                self.threshTailC_slider,
                self.tailArcCurveC_label,
                self.tailArcCurveCValue_label,
                self.tailArcCurveC_slider,
            ],
        }

        # Compact layout: row 0 = R/M/C activation,
        # row 1 = thresholds, row 2 = curvature controls.
        tailGrid = QtWidgets.QGridLayout(splitter_ThreshTail)
        tailGrid.setContentsMargins(0, 0, 0, 0)
        tailGrid.setHorizontalSpacing(6)
        tailGrid.setVerticalSpacing(4)

        tailGrid.addWidget(QtWidgets.QLabel("Track arcs"), 0, 0)
        tailGrid.addWidget(self.tailArcR_checkBox, 0, 1)
        tailGrid.addWidget(self.tailArcM_checkBox, 0, 3)
        tailGrid.addWidget(self.tailArcC_checkBox, 0, 6)
        tailGrid.setColumnStretch(8, 1)

        tailGrid.addWidget(self.threshTailSlider_label, 1, 0)
        tailGrid.addWidget(self.threshTailValue_label, 1, 1)
        tailGrid.addWidget(self.threshTail_slider, 1, 2)
        tailGrid.addWidget(self.threshTailMSlider_label, 1, 3)
        tailGrid.addWidget(self.threshTailMValue_label, 1, 4)
        tailGrid.addWidget(self.threshTailM_slider, 1, 5)
        tailGrid.addWidget(self.threshTailCSlider_label, 1, 6)
        tailGrid.addWidget(self.threshTailCValue_label, 1, 7)
        tailGrid.addWidget(self.threshTailC_slider, 1, 8)

        tailGrid.addWidget(self.tailArcCurveR_label, 2, 0)
        tailGrid.addWidget(self.tailArcCurveRValue_label, 2, 1)
        tailGrid.addWidget(self.tailArcCurveR_slider, 2, 2)
        tailGrid.addWidget(self.tailArcCurveM_label, 2, 3)
        tailGrid.addWidget(self.tailArcCurveMValue_label, 2, 4)
        tailGrid.addWidget(self.tailArcCurveM_slider, 2, 5)
        tailGrid.addWidget(self.tailArcCurveC_label, 2, 6)
        tailGrid.addWidget(self.tailArcCurveCValue_label, 2, 7)
        tailGrid.addWidget(self.tailArcCurveC_slider, 2, 8)

        for stretch_col in [2, 5, 8]:
            tailGrid.setColumnStretch(stretch_col, 1)

        splitter_ThreshEyes=QtWidgets.QSplitter()

        threshEye1Slider_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye1Slider_label.setText("Eye1")
        self.threshEye1_slider = QtWidgets.QSlider(splitter_ThreshEyes)
        self.threshEye1_slider.setMaximum(255)
        self.threshEye1_slider.setPageStep(10)
        self.threshEye1_slider.setOrientation(Qt.Horizontal)
        threshEye1Value_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye1Value_label.setText("00")
        threshEye1Value_label.setAlignment(Qt.AlignCenter)
        self.threshEye1_slider.valueChanged.connect(threshEye1Value_label.setNum)
        self.threshEye1_slider.valueChanged.connect(lambda value,idx=0 : self.threshEyeValue_change(value,idx))

        threshEye2Slider_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye2Slider_label.setText("Eye2")
        self.threshEye2_slider = QtWidgets.QSlider(splitter_ThreshEyes)
        self.threshEye2_slider.setMaximum(255)
        self.threshEye2_slider.setPageStep(10)
        self.threshEye2_slider.setOrientation(Qt.Horizontal)
        threshEye2Value_label = QtWidgets.QLabel(splitter_ThreshEyes)
        threshEye2Value_label.setText("00")
        threshEye2Value_label.setAlignment(Qt.AlignCenter)
        self.threshEye2_slider.valueChanged.connect(threshEye2Value_label.setNum)
        self.threshEye2_slider.valueChanged.connect((lambda value,idx=1 : self.threshEyeValue_change(value,idx)))

        self.threshTail_slider.setValue(90)
        self.threshTailM_slider.setValue(90)
        self.threshTailC_slider.setValue(90)
        self.tailArcCurve_slider.setValue(40)
        self.threshEye1_slider.setValue(60)
        self.threshEye2_slider.setValue(60)
        self.update_tail_arc_enabled_states(update_preview=False)

        resetPlots_btn = QtWidgets.QPushButton('Reset plots')
        resetPlots_btn.clicked.connect(self.reset_Plot)
        resetPlots_btn.setMinimumHeight(28)

        resetBuffer_btn = QtWidgets.QPushButton('Reset buffer')
        resetBuffer_btn.clicked.connect(self.reset_Buffer)
        resetBuffer_btn.setMinimumHeight(28)

        self.saveAnalysisSettings_btn = QtWidgets.QPushButton("Save settings")
        self.saveAnalysisSettings_btn.setToolTip(
            "Save all current settings for Real-time camera or Imported video."
        )
        self.saveAnalysisSettings_btn.clicked.connect(self.save_analysis_settings)
        self.saveAnalysisSettings_btn.setMinimumHeight(28)

        self.loadAnalysisSettings_btn = QtWidgets.QPushButton("Load settings")
        self.loadAnalysisSettings_btn.setToolTip(
            "Reload settings previously saved as JSON."
        )
        self.loadAnalysisSettings_btn.clicked.connect(self.load_analysis_settings)
        self.loadAnalysisSettings_btn.setMinimumHeight(28)

        self.outputFolder_btn = QtWidgets.QPushButton("Output folder")
        self.outputFolder_btn.clicked.connect(self.choose_result_folder)

        self.outputFolder_label = QtWidgets.QLabel("No folder selected")
        self.outputFolder_label.setMinimumWidth(180)
        self.outputFolder_label.setMaximumWidth(260)

        stage_label = QtWidgets.QLabel("Stage")
        self.stageLineEdit = QtWidgets.QLineEdit()
        self.stageLineEdit.setPlaceholderText("ex: 52")
        self.stageLineEdit.setMaximumWidth(60)
        self.stageLineEdit.textChanged.connect(self.update_next_csv_preview)

        self.nextCsv_label = QtWidgets.QLabel("Next CSV: -")
        self.nextCsv_label.setMinimumWidth(180)
        self.nextCsv_label.setMaximumWidth(260)
        
        fileNumber_label = QtWidgets.QLabel("File no.")

        self.fileNumber_spinBox = QtWidgets.QSpinBox()
        self.fileNumber_spinBox.setMinimum(-1)
        self.fileNumber_spinBox.setMaximum(999)
        self.fileNumber_spinBox.setValue(-1)
        self.fileNumber_spinBox.setSpecialValueText("auto")
        self.fileNumber_spinBox.setMaximumWidth(70)
        self.fileNumber_spinBox.setToolTip(
            "Optional file number.\n"
            "auto = next available number.\n"
            "Example: 3 creates file ..._003.csv"
        )
        self.fileNumber_spinBox.valueChanged.connect(self.update_next_csv_preview)

        splitter_Output = QtWidgets.QSplitter()
        splitter_Output.setOrientation(Qt.Horizontal)
        splitter_Output.addWidget(self.outputFolder_btn)
        splitter_Output.addWidget(self.outputFolder_label)
        splitter_Output.addWidget(stage_label)
        splitter_Output.addWidget(self.stageLineEdit)
        splitter_Output.addWidget(fileNumber_label)
        splitter_Output.addWidget(self.fileNumber_spinBox)
        splitter_Output.addWidget(self.nextCsv_label)
        
        actionPanel = QtWidgets.QWidget()
        actionLayout = QtWidgets.QHBoxLayout(actionPanel)
        actionLayout.setContentsMargins(0, 4, 0, 0)
        actionLayout.setSpacing(8)

        secondaryButtons = QtWidgets.QWidget()
        secondaryLayout = QtWidgets.QVBoxLayout(secondaryButtons)
        secondaryLayout.setContentsMargins(0, 0, 0, 0)
        secondaryLayout.setSpacing(5)

        secondaryLayout.addWidget(initLimbTrack_btn)
        secondaryLayout.addWidget(resetPlots_btn)
        secondaryLayout.addWidget(resetBuffer_btn)

        actionLayout.addWidget(self.track_checkBox, 2)
        actionLayout.addWidget(secondaryButtons, 1)

        # ------------------------------------------------------------------
        # ROI / segmentation panel: cleaner and more readable layout
        # ------------------------------------------------------------------
        self.w8.setContentsMargins(8, 6, 8, 6)

        modeGroup = QtWidgets.QGroupBox("Mode")
        modeLayout = QtWidgets.QVBoxLayout(modeGroup)
        modeLayout.setContentsMargins(8, 8, 8, 8)
        modeLayout.setSpacing(6)
        modeLayout.addWidget(self.manipType_comboBox)

        selectionGroup = QtWidgets.QGroupBox("Selection")
        selectionLayout = QtWidgets.QVBoxLayout(selectionGroup)
        selectionLayout.setContentsMargins(8, 8, 8, 8)
        selectionLayout.setSpacing(6)
        selectionLayout.addWidget(self.selectEyes_radioButton)
        selectionLayout.addWidget(self.selectTailRoot_radioButton)
        # Limb and Circle are kept in the code for compatibility,
        # but removed from the interface to free space.
        self.selectLimbs_radioButton.hide()
        self.selectExclusion_radioButton.hide()

        bgGroup = QtWidgets.QGroupBox("Background")
        bgLayout = QtWidgets.QVBoxLayout(bgGroup)
        bgLayout.setContentsMargins(8, 8, 8, 8)
        bgLayout.setSpacing(6)
        bgLayout.addWidget(self.whiteBgd_radioButton)
        bgLayout.addWidget(self.blackBgd_radioButton)
        bgLayout.addStretch(1)

        eyesGroup = QtWidgets.QGroupBox("Eyes")
        eyesLayout = QtWidgets.QVBoxLayout(eyesGroup)
        eyesLayout.setContentsMargins(8, 8, 8, 8)
        eyesLayout.setSpacing(6)
        eyesLayout.addWidget(splitter_ThreshEyes)

        tailGroup = QtWidgets.QGroupBox("Tails")
        tailLayout = QtWidgets.QVBoxLayout(tailGroup)
        tailLayout.setContentsMargins(8, 8, 8, 8)
        tailLayout.setSpacing(6)
        tailLayout.addWidget(splitter_ThreshTail)

        outputGroup = QtWidgets.QGroupBox("Output")
        outputLayout = QtWidgets.QVBoxLayout(outputGroup)
        outputLayout.setContentsMargins(8, 8, 8, 8)
        outputLayout.setSpacing(6)
        outputLayout.addWidget(splitter_Output)

        actionsGroup = QtWidgets.QGroupBox("Actions")
        actionsLayout = QtWidgets.QVBoxLayout(actionsGroup)
        actionsLayout.setContentsMargins(8, 8, 8, 8)
        actionsLayout.setSpacing(6)
        actionsLayout.addWidget(self.track_checkBox)
        actionsLayout.addWidget(initLimbTrack_btn)
        actionsLayout.addWidget(resetPlots_btn)
        actionsLayout.addWidget(resetBuffer_btn)

        # Small local style: more spacing without changing the global Imagys theme.
        panelStyle = """
        QGroupBox {
            font-weight: bold;
            border: 1px solid #b8b8d8;
            border-radius: 6px;
            margin-top: 8px;
            padding-top: 8px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 4px;
        }
        QPushButton {
            min-height: 26px;
        }
        """
        for group in [modeGroup, selectionGroup, bgGroup, eyesGroup, tailGroup, outputGroup, actionsGroup]:
            group.setStyleSheet(panelStyle)

        self.w8.addWidget(modeGroup, row=0, col=0)
        self.w8.addWidget(selectionGroup, row=1, col=0)
        self.w8.addWidget(bgGroup, row=2, col=0, rowspan=2)

        self.w8.addWidget(eyesGroup, row=0, col=1, colspan=2)
        self.w8.addWidget(tailGroup, row=1, col=1, rowspan=2, colspan=2)
        self.w8.addWidget(outputGroup, row=3, col=1, colspan=2)

        self.w8.addWidget(actionsGroup, row=0, col=3, rowspan=4)

        self.d8.addWidget(self.w8)

        self.w10 = pg.LayoutWidget()

        self.contourCorrection_label=QtWidgets.QLabel("Contour correction")

        self.medianFilter_checkBox=QtWidgets.QCheckBox(""" median filter matrix : """)
        self.medianFilter_checkBox.setChecked(True)

        self.medianFilter_spinbox =  QtWidgets.QSpinBox(value=3, singleStep=2,minimum=1,maximum=21)

        self.contourOpen_checkBox=QtWidgets.QCheckBox("""Open""")
        self.contourOpen_checkBox.setChecked(True)

        splitter_openKernel  = QtWidgets.QSplitter()
        openKernel_label = QtWidgets.QLabel("""kernel : """)
        self.openKernel_spinbox =  QtWidgets.QSpinBox(value=9, singleStep=2,minimum=1,maximum=9)
        self.openKernel_spinbox.valueChanged.connect(lambda: update_settings())
        splitter_openKernel.addWidget(openKernel_label)
        splitter_openKernel.addWidget(self.openKernel_spinbox)

        splitter_openIteration  = QtWidgets.QSplitter()
        openIteration_label = QtWidgets.QLabel("""itérations : """)
        self.openIteration_spinbox =  QtWidgets.QSpinBox(value=1, singleStep=1,minimum=1,maximum=5)
        splitter_openIteration.addWidget(openIteration_label)
        splitter_openIteration.addWidget(self.openIteration_spinbox)

        splitter_open  = QtWidgets.QSplitter()
        splitter_open.addWidget(self.contourOpen_checkBox)
        splitter_open.addWidget(openKernel_label)
        splitter_open.addWidget(self.openKernel_spinbox)
        splitter_open.addWidget(openIteration_label)
        splitter_open.addWidget(self.openIteration_spinbox)

        splitter_set1 = QtWidgets.QSplitter()
        splitter_label = QtWidgets.QLabel("""  """)
        splitter_label.setMinimumWidth(100)
        splitter_set1.addWidget(splitter_label)
        splitter_set2 = QtWidgets.QSplitter()

        imFilteredViewLayout = pg.GraphicsLayoutWidget()

        v1a = imFilteredViewLayout.addViewBox(lockAspect=True)
        arr = np.ones((50, 50), dtype=float)
        self.imFilteredTest =  pg.ImageItem(arr)
        v1a.addItem(self.imFilteredTest)
        v1a.disableAutoRange('xy')
        v1a.autoRange()

        splitter_set2.addWidget(imFilteredViewLayout)

        self.threshold_label = QtWidgets.QLabel(""" threshold  """)
        self.threshold_slider = QtWidgets.QSlider()
        self.threshold_slider.setMaximum(255)
        self.threshold_slider.setPageStep(10)
        self.threshold_slider.setOrientation(Qt.Horizontal)
        self.threshold_slider.setEnabled(False)
        self.threshValue_edit=QtWidgets.QLineEdit()
        self.threshValue_edit.setText("00")
        self.threshValue_edit.setMinimumWidth(30)
        self.threshValue_edit.setMaximumWidth(30)

        self.threshold_slider.setEnabled(False)
        self.autoThresh_checkBox=QtWidgets.QCheckBox("Auto thresh")
        self.autoThresh_checkBox.clicked.connect(self.activate_interface)

        self.w10.addWidget(splitter_set1,row=0,col=0,rowspan=5)

        self.w10.addWidget(self.contourCorrection_label,row=0,col=1)
        self.w10.addWidget(self.medianFilter_checkBox,row=1,col=2)
        self.w10.addWidget(self.medianFilter_spinbox,row=1,col=3)
        self.w10.addWidget(splitter_open,row=2,col=2,colspan=2)
        self.w10.addWidget(splitter_set2,row=0,col=5,rowspan=5)

        self.w10.addWidget(self.threshold_label, row=3, col=1)
        self.w10.addWidget(self.threshold_slider, row=3, col=2)
        self.w10.addWidget(self.threshValue_edit, row=3, col=3)
        self.w10.addWidget(self.autoThresh_checkBox,row=3,col=4)

        self.d10.addWidget(self.w10)

        self.d11.addWidget(self.video_capture_widget)

        # Settings buttons for Real-time camera mode.
        # They are placed in the video Capture dock, not in the global toolbar,
        # to keep the same organization as the Imported video panel.
        try:
            self.saveAnalysisSettings_btn.setMaximumWidth(140)
            self.loadAnalysisSettings_btn.setMaximumWidth(140)
            self.video_capture_widget.addWidget(self.saveAnalysisSettings_btn, row=1, col=6)
            self.video_capture_widget.addWidget(self.loadAnalysisSettings_btn, row=1, col=7)
        except Exception as exc:
            print("Could not add real-time settings buttons:", exc)

        self.optokinetic_Widget=optok.UIOptostim()
        self.d13.addWidget(self.optokinetic_Widget)

        # ------------------------------------------------------------------
        # Dedicated interface for imported-video analysis.
        # It is intentionally separated from the video player and camera capture.
        # ------------------------------------------------------------------
        self.w14 = pg.LayoutWidget()

        self.videoFileHelp_label = QtWidgets.QLabel(
            "Imported video analysis: open a video, place the same ROIs/arcs on the image, then analyze all frames."
        )
        self.videoFileHelp_label.setWordWrap(True)

        self.openImportedVideo_btn = QtWidgets.QPushButton("Open video")
        self.openImportedVideo_btn.clicked.connect(self.choose_imported_video_file)

        self.importedVideoPath_label = QtWidgets.QLabel("No video selected")
        self.importedVideoPath_label.setWordWrap(True)

        self.showDirectInterface_btn = QtWidgets.QPushButton("Back to real-time")
        self.showDirectInterface_btn.clicked.connect(self.show_direct_interface)

        self.startImportedVideo_btn = QtWidgets.QPushButton("Analyze")
        self.startImportedVideo_btn.clicked.connect(self.start_imported_video_analysis)
        self.startImportedVideo_btn.setMinimumHeight(28)

        self.stopImportedVideo_btn = QtWidgets.QPushButton("Stop")
        self.stopImportedVideo_btn.clicked.connect(self.stop_imported_video_analysis)
        self.stopImportedVideo_btn.setEnabled(False)
        self.stopImportedVideo_btn.setMinimumHeight(28)

        self.importedVideoProgress = QtWidgets.QProgressBar()
        self.importedVideoProgress.setRange(0, 100)
        self.importedVideoProgress.setValue(0)

        self.importedVideoStatus_label = QtWidgets.QLabel("Video mode: idle")
        self.importedVideoStatus_label.setWordWrap(True)

        self.previewDuringAnalysis_ckb = QtWidgets.QCheckBox("Preview while analyzing")
        self.previewDuringAnalysis_ckb.setChecked(True)
        self.previewDuringAnalysis_ckb.setToolTip(
            "Show the currently analyzed frame with tracking overlays during imported video analysis."
        )

        self.reviewFramePrev_btn = QtWidgets.QPushButton("◀")
        self.reviewFramePrev_btn.setMaximumWidth(38)
        self.reviewFramePrev_btn.clicked.connect(self.review_imported_video_previous_frame)

        self.reviewFrameNext_btn = QtWidgets.QPushButton("▶")
        self.reviewFrameNext_btn.setMaximumWidth(38)
        self.reviewFrameNext_btn.clicked.connect(self.review_imported_video_next_frame)

        self.reviewFrameSlider = QtWidgets.QSlider(Qt.Horizontal)
        self.reviewFrameSlider.setRange(0, 0)
        self.reviewFrameSlider.setEnabled(False)
        self.reviewFrameSlider.valueChanged.connect(self.on_review_frame_slider_changed)

        self.reviewFrameSpin = QtWidgets.QSpinBox()
        self.reviewFrameSpin.setRange(0, 0)
        self.reviewFrameSpin.setEnabled(False)
        self.reviewFrameSpin.valueChanged.connect(self.on_review_frame_spin_changed)

        self.reviewFrameLabel = QtWidgets.QLabel("Review frame")
        self.reviewFrameLabel.setMinimumWidth(85)

        self.videoCropEnable_ckb = QtWidgets.QCheckBox("Use video crop")
        self.videoCropEnable_ckb.setChecked(True)
        self.videoCropEnable_ckb.setToolTip(
            "Analyze only the selected region of the imported video."
        )

        # Crop vidéo importée : mêmes contrôles que la navigation caméra,
        # mais sans champ numérique éditable.
        # On garde des labels de valeur + sliders horizontaux.
        self.videoCropFrameWidth_value = QtWidgets.QLabel("00")
        self.videoCropFrameHeight_value = QtWidgets.QLabel("00")
        self.videoCropOffsetX_value = QtWidgets.QLabel("00")
        self.videoCropOffsetY_value = QtWidgets.QLabel("00")

        self.videoCropFrameWidth_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.videoCropFrameHeight_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.videoCropOffsetX_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.videoCropOffsetY_slider = QtWidgets.QSlider(Qt.Horizontal)

        for value_label in [
            self.videoCropFrameWidth_value,
            self.videoCropFrameHeight_value,
            self.videoCropOffsetX_value,
            self.videoCropOffsetY_value,
        ]:
            value_label.setMinimumWidth(42)
            value_label.setAlignment(Qt.AlignCenter)

        for slider in [
            self.videoCropFrameWidth_slider,
            self.videoCropFrameHeight_slider,
            self.videoCropOffsetX_slider,
            self.videoCropOffsetY_slider,
        ]:
            slider.setRange(0, 99999)
            slider.setPageStep(10)
            slider.valueChanged.connect(self.update_imported_video_crop_labels)

        self.videoCropApply_btn = QtWidgets.QPushButton("Apply crop")
        self.videoCropApply_btn.setMaximumWidth(95)
        self.videoCropApply_btn.clicked.connect(self.apply_imported_video_crop)

        self.videoCropReset_btn = QtWidgets.QPushButton("Full frame")
        self.videoCropReset_btn.setMaximumWidth(95)
        self.videoCropReset_btn.clicked.connect(self.reset_imported_video_crop)

        self.saveImportedSettings_btn = QtWidgets.QPushButton("Save settings")
        self.saveImportedSettings_btn.setToolTip(
            "Save the current imported-video/live settings in JSON."
        )
        self.saveImportedSettings_btn.clicked.connect(self.save_analysis_settings)

        self.loadImportedSettings_btn = QtWidgets.QPushButton("Load settings")
        self.loadImportedSettings_btn.setToolTip(
            "Load a JSON settings file and restore ROIs/arcs/thresholds/camera UI."
        )
        self.loadImportedSettings_btn.clicked.connect(self.load_analysis_settings)

        self.videoFileNote_label = QtWidgets.QLabel(
            "This tab does not use the camera capture dock or the video player dock. "
            "It reuses the same tracking code as live mode, including R/M/C arcs, thresholds, curves, and CSV output."
        )
        self.videoFileNote_label.setWordWrap(True)

        self.w14.addWidget(self.videoFileHelp_label, row=0, col=0, colspan=4)
        self.w14.addWidget(self.openImportedVideo_btn, row=1, col=0)
        self.w14.addWidget(self.importedVideoPath_label, row=1, col=1, colspan=3)
        self.w14.addWidget(self.startImportedVideo_btn, row=2, col=0)
        self.w14.addWidget(self.stopImportedVideo_btn, row=2, col=1)
        self.w14.addWidget(self.showDirectInterface_btn, row=2, col=2)
        self.w14.addWidget(self.importedVideoProgress, row=3, col=0, colspan=4)
        self.w14.addWidget(self.importedVideoStatus_label, row=4, col=0, colspan=4)
        self.w14.addWidget(self.videoFileNote_label, row=5, col=0, colspan=4)

        self.d14.addWidget(self.w14)

        # Dedicated progress strip placed under the image in Imported video mode.
        self.importedProgressWidget = QtWidgets.QWidget()
        self.importedProgressLayout = QtWidgets.QGridLayout(self.importedProgressWidget)
        self.importedProgressLayout.setContentsMargins(8, 4, 8, 4)
        self.importedProgressLayout.setHorizontalSpacing(8)
        self.importedProgressLayout.setVerticalSpacing(3)

        self.importedProgressTitle_label = QtWidgets.QLabel("Imported video")
        self.importedProgressTitle_label.setMinimumWidth(95)

        # Row 0: video actions
        self.importedProgressLayout.addWidget(self.importedProgressTitle_label, 0, 0)
        self.importedProgressLayout.addWidget(self.openImportedVideo_btn, 0, 1)
        self.importedProgressLayout.addWidget(self.importedVideoPath_label, 0, 2, 1, 2)
        self.importedProgressLayout.addWidget(self.startImportedVideo_btn, 0, 4)
        self.importedProgressLayout.addWidget(self.stopImportedVideo_btn, 0, 5)

        # Row 1: progress
        self.importedProgressLayout.addWidget(self.importedVideoProgress, 1, 0, 1, 4)
        self.importedProgressLayout.addWidget(self.importedVideoStatus_label, 1, 4, 1, 2)

        # Row 2: frame-by-frame navigation during or after analysis
        self.importedProgressLayout.addWidget(self.reviewFrameLabel, 2, 0)
        self.importedProgressLayout.addWidget(self.reviewFramePrev_btn, 2, 1)
        self.importedProgressLayout.addWidget(self.reviewFrameSlider, 2, 2, 1, 2)
        self.importedProgressLayout.addWidget(self.reviewFrameSpin, 2, 4)
        self.importedProgressLayout.addWidget(self.reviewFrameNext_btn, 2, 5)
        self.importedProgressLayout.addWidget(self.previewDuringAnalysis_ckb, 3, 0, 1, 6)

        # Rows 4 to 7: imported-video crop controls.
        # No visible QSpinBox controls: labels plus sliders.
        self.importedProgressLayout.addWidget(self.videoCropEnable_ckb, 4, 0)
        self.importedProgressLayout.addWidget(QtWidgets.QLabel("frame width"), 4, 1)
        self.importedProgressLayout.addWidget(self.videoCropFrameWidth_value, 4, 2)
        self.importedProgressLayout.addWidget(self.videoCropFrameWidth_slider, 4, 3, 1, 2)
        self.importedProgressLayout.addWidget(self.videoCropApply_btn, 4, 5)

        self.importedProgressLayout.addWidget(QtWidgets.QLabel("frame height"), 5, 1)
        self.importedProgressLayout.addWidget(self.videoCropFrameHeight_value, 5, 2)
        self.importedProgressLayout.addWidget(self.videoCropFrameHeight_slider, 5, 3, 1, 2)
        self.importedProgressLayout.addWidget(self.videoCropReset_btn, 5, 5)

        self.importedProgressLayout.addWidget(QtWidgets.QLabel("offset x"), 6, 1)
        self.importedProgressLayout.addWidget(self.videoCropOffsetX_value, 6, 2)
        self.importedProgressLayout.addWidget(self.videoCropOffsetX_slider, 6, 3, 1, 2)

        self.importedProgressLayout.addWidget(QtWidgets.QLabel("offset y"), 7, 1)
        self.importedProgressLayout.addWidget(self.videoCropOffsetY_value, 7, 2)
        self.importedProgressLayout.addWidget(self.videoCropOffsetY_slider, 7, 3, 1, 2)

        self.importedProgressLayout.addWidget(QtWidgets.QLabel("Settings"), 8, 1)
        self.importedProgressLayout.addWidget(self.saveImportedSettings_btn, 8, 3)
        self.importedProgressLayout.addWidget(self.loadImportedSettings_btn, 8, 4)

        self.importedProgressLayout.setColumnStretch(3, 1)
        self.importedProgressLayout.setColumnStretch(4, 1)

        self.d15.addWidget(self.importedProgressWidget)

        # Imported-video controls are no longer in the top toolbar.
        # They are placed in the strip under the image, together with progress.
        # The toolbar remains unused to avoid an overloaded top area.
        try:
            self.importedVideoPath_label.setMinimumWidth(260)
            self.importedVideoProgress.setMinimumHeight(18)
            self.importedVideoStatus_label.setMinimumWidth(320)
            self.openImportedVideo_btn.setMaximumWidth(120)
            self.startImportedVideo_btn.setMaximumWidth(95)
            self.stopImportedVideo_btn.setMaximumWidth(70)
        except Exception:
            pass

        # Default startup mode is live/direct.
        # The imported-video dock stays hidden until Imported video mode is selected.
        try:
            self._set_dock_clean_visible(self.d15, False)
        except Exception:
            pass

        self.set_analysis_mode("live", update_combo=True)

        # With the Video player dock removed, video Capture is no longer in a
        # tabbed container. raiseDock() may therefore raise an error depending on the
        # pyqtgraph version. Keep the call only when possible.
        try:
            self.d8.raiseDock()
        except Exception:
            pass

        try:
            self.d11.raiseDock()
        except Exception:
            pass

        try:
            from Imagys_blue.qsshelper import QSSHelper
            qss = QSSHelper.open_qss(os.path.join('Imagys_blue', 'Imagys-blue.qss'))
            self.setStyleSheet(qss)
        except Exception as e:
            print('QSS not loaded:', e)

        self.show()

    def _bind_runtime_context(self):
        """Expose legacy shared objects to extracted modules.

        Several old callbacks still use module-level variables such as ui,
        image, or varM. This bridge keeps behavior unchanged while the
        codebase is progressively cleaned.
        """
        ctx = {
            "app": globals().get("app"),
            "analysisSet": globals().get("analysisSet"),
            "tracking": globals().get("tracking"),
            "video": globals().get("video"),
            "image": globals().get("image"),
            "target": globals().get("target"),
            "varM": globals().get("varM"),
            "ui": self,
            "timestampList": timestampList,
            "roisLimb": roisLimb,
            "stimList": stimList,
            "framesBuffer": framesBuffer,
            "define_rois": define_rois,
            "video_player": video_player,
            "video_capture": video_capture,
            "analysis": analysis,
            "optok": optok,
        }

        for module in [
            app_state_module,
            legacy_tracking_module,
            tail_arc_roi_module,
            overlays_module,
            docks_module,
            imported_video_panel_module,
            settings_manager_module,
            tracking_controls_panel_module,
            plot_panel_module,
        ]:
            try:
                module.set_context(**ctx)
            except Exception:
                pass


def main():
    """Application entry point."""
    global app, analysisSet, tracking, video, image, target, varM, ui

    app = QtWidgets.QApplication([])

    analysisSet = Analysis_Settings()
    tracking = analysis.Tracking()
    video = video_player.Video()
    image = analysis.ImageContainer()
    target = Target()
    varM = Measure_Var()

    ui = UIXenopus()
    ui._bind_runtime_context()

    ui.controller = AppController(
        ui=ui,
        video=ui.video,
        varM=varM,
    )

    app.exec_()
    print("the end")
    app.quit()


if __name__ == "__main__":
    main()
