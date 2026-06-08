# -*- coding: utf-8 -*-
"""
Created on Tue May  2 13:49:31 2017

@author: courtand, kadri

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
v15 : utilisation du module video_player_4, video_capture_2b (load pylon config node)
v15a : mise à jour du code ftdi            
--nouveau nom Swim-X ?
v15c : reduction des frequence de rafraichissement des plot et overlay (100 et 50)
        pour analyse à 200fps
v15d : amélioration des réglages de seuillage de la queue (avant tracking)
v15e : analyse des frames stochés dans le buffer après arrêt du tracking
v16 : change the way to acquire frames, dissociate live and analyze/track
v17 : back to the threading process for read_and_analyze
v17d : class processingThread run : modification des temps d'attente pour le frame suivant : 0.001'
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
import sys
from PyQt5.QtCore import Qt, QTimer, pyqtSlot, QRectF
from PyQt5 import QtWidgets

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui as QtgGui

from pyqtgraph.dockarea import DockArea, Dock
from app_controller import AppController

import numpy as np
import math
import csv
import cv2

from pylibftdi import BitBangDevice, Driver

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import video_player_6 as video_player
import video_capture_5 as video_capture
import analysis_2 as analysis
import define_rois_4 as define_rois
import optostim_9 as optok

timestampList=[]

roisLimb = []
stimList=[]

framesBuffer=[]

class Analysis_Settings :
    def __init__(self):
        self.framerateFactor=1
        self.threshMode = cv2.THRESH_BINARY_INV
        self.kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))

        self.duration=60
        self.nbFramesToAnalyze=0

        self.scale=1
        self.scaleUnit="pixel"

    def set_framerateFactor(self, i):
        self.framerateFactor = i

    def set_thresholdMethod(self,whiteIsChecked):

        if whiteIsChecked==True :
            self.threshMode=cv2.THRESH_BINARY
        else :
            self.threshMode=cv2.THRESH_BINARY_INV

def update_settings():
    kernelWidth=ui.openKernel_spinbox.value()
    analysisSet.kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernelWidth,kernelWidth))

class Measure_Var :
    def __init__(self):

        self.var=[]
        self.measuredLivefps=00
        self.measuredPlayfps=00
        self.lastTime_live=time.time()
        self.lastTime_play=time.time()
        self.frameCount=0
        self.timeToUpdate=0
        self.ctailX=[]
        self.ctailY=[]
        self.bodyAxis_Y=0
        self.idxResultArray=0
        self.bodyAngle=0
        self.tailAngle=[]
        self.tailAngleCorr=[]

class Roi(pg.RectROI):
    def __init__(self, pos, size, centered, sideScalers=False, **args):
        pg.RectROI.__init__(self, pos, size, centered, sideScalers=False, **args)

        self.posList=[[0,0]]

class Target:
    def __init__(self):
        self.name="None"
        self.size=0
        self.minRadius=1
        self.alldist=0
        self.tracker=None
        self.region=None

def get_ftdi_device_list():
    dev_list = []

    ref_list = []

    for device in Driver().list_devices():

        vendor, product, serial = device
        dev_list.append("%s:%s:%s" % (vendor, product, serial))
        print("ftdi : ",dev_list)
        ref_list=[vendor, product, serial]
    return ref_list

def trigger():
    try :

        ftdiRef_list=get_ftdi_device_list()

        if len(ftdiRef_list)!=0:
            with BitBangDevice(ftdiRef_list[1]) as bb:
                bb.direction = 0x0F
                bb.port |= 2
                bb.port &= 0xFE

    except sys.exc_info()[0] as e:

        print("error sys : ",e)
        QtWidgets.QMessageBox.warning(None,"Error",str(e))

    except IOError as e :
        print(e)
        QtWidgets.QMessageBox.warning(None,"Error",str(e))

def eye_segmentation(roiIndex,roiEye,threshEye,analysimg):

    img_height,img_width=analysimg.shape

    xroi,yroi=roiEye.pos()
    wroi,hroi=roiEye.size()

    roiXc,roiYc,wroic,hroic=testRoiInImageview(xroi,yroi,wroi,hroi,analysimg)

    image.crop=[roiXc,roiYc,wroic,hroic]

    xroicv,yroicv,wroicv,hroicv=int(roiXc),int(img_height-roiYc-hroic),int(wroic),int(hroic)

    cropImgData=analysimg[yroicv:yroicv+hroicv,xroicv:xroicv+wroicv]

    blur = cv2.GaussianBlur(cropImgData, (5, 5), 0)

    threshed_Eye = cv2.threshold(blur, threshEye[roiIndex], 255, cv2.THRESH_BINARY_INV)[1]

    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
    threshed_Eye = cv2.morphologyEx(threshed_Eye, cv2.MORPH_OPEN, kernel)

    (cnts, _) = cv2.findContours(threshed_Eye.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]

    if len(cnts)>0:

        cnt = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]

        return cnt

    else :
        print("no eye contour")

def eye_Rotation(roiIndex,roiEye,cnt):

    for contour in cnt :

        hull = cv2.convexHull(contour)
        if len(hull)>4 :
            ellipse= cv2.fitEllipse(hull)
            (xe,ye),(ma,MA),angle = ellipse

        else :

            print("eye detection failed (rotate)")
            print(roiIndex)
            print(len(ui.roisEllipseEye))
            print(ui.roisEllipseEye[roiIndex].yList[-1])
            ellipse_pos=ui.roisEllipseEye[roiIndex].yList[-1]
            print(ellipse_pos)
            return(ellipse_pos,0)
            break

        anglep=angle-90

        xroi,yroi=roiEye.pos()
        wroi,hroi=roiEye.size()
        xroi,yroi,wroi,hroi=image.crop
        xp,yp=int(xroi+xe-MA/2),int(yroi+hroi-ye-ma/2)

        xc=MA/2
        yc=ma/2
        radangle=anglep/180*math.pi
        xrot=xc*math.cos(radangle) + yc*math.sin(radangle)
        yrot=xc*(-math.sin(radangle))+yc*math.cos(radangle)

        vx=xrot-xc
        vy=yrot-yc
        ui.roisEllipseEye[roiIndex].descriptor=[xp,yp,MA,ma,-anglep,vx,vy,xrot,yrot]
        ellipse_pos=[xp-vx+xrot,yp-vy+yrot]

        newroiX,newroiY=int(xroi+xe-wroi/2),int(yroi+hroi-ye-hroi/2)

        return(ellipse_pos,anglep)

def tail_Track(iframe,analysimg,thresh_tail,rgn_tail):

    im_height,im_width=analysimg.shape

    threshValue_Tail=thresh_tail

    xroicv=int(rgn_tail[0])

    yroicv=0
    wroi=int(rgn_tail[1]-rgn_tail[0])

    hroi=int(im_height)

    cropImgData=analysimg[yroicv:yroicv+hroi,xroicv:xroicv+wroi]

    threshTailZone = cv2.threshold(cropImgData, threshValue_Tail, 255, cv2.THRESH_BINARY_INV)[1]
    threshTailZone = cv2.dilate(threshTailZone, None, iterations=5)

    (tailcnts, _) = cv2.findContours(threshTailZone.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]

    if tailcnts :

        ctail = sorted(tailcnts, key = cv2.contourArea, reverse = True)[:1]

        M = cv2.moments(ctail[0])
        tailX = int(M["m10"] / M["m00"])
        tailY = int(M["m01"] / M["m00"])

        ctailX=tailX+xroicv
        ctailY=int(im_height)-tailY

        rootx,rooty=ui.mark.data['pos'][0]

        tailAngle=((math.atan2((rooty-ctailY),(rootx-ctailX))) * 180 / math.pi)*(-1)

        tailAngleCorr=tailAngle+varM.bodyAngle

    else :

        print("tail detection failed")
        tailAngleCorr=180

        ctailX=0
        ctailY=0

    tail_pos=[ctailX,ctailY]

    ui.tailAngleList.append([iframe,tailAngleCorr])
    ui.tailPosList.append([iframe,tail_pos])
    print("Tail anglelist ADD",iframe)

    return iframe,tail_pos,tailAngleCorr

def testRoiInImageview(x,y,w,h,img):

    img_height,img_width=img.shape

    if x<0 :
        w=w+x
        x=0
    elif x+w > img_width:
        diff=x+w-img_width
        w=w-diff
    if y<0 :
        h=h+y
        y=0
    elif y+h > img_height:
        diff=y+h-img_height
        h=h-diff
    return(x,y,w,h)

class graphMark(pg.GraphItem):
    def __init__(self):
        self.dragPoint = None
        self.dragOffset = None
        self.textItems = []
        pg.GraphItem.__init__(self)
        self.scatter.sigClicked.connect(self.clicked)

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

        if ev.button() != Qt.LeftButton:
            ev.ignore()
            return

        if ev.isStart():

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

class UIXenopus(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent=None)

        self.video=video_player.Video()
        self.display_updater=None
        self.videoDisplay_Widget=define_rois.UIVideoDisplayRoi(self.video)
        self.videoDisplay_Widget.proxy1 = pg.SignalProxy(self.videoDisplay_Widget.plotView.scene().sigMouseClicked, rateLimit=60, slot=self.mouse_clicked)
        self.videoPlayer_Widget=video_player.UIVideoPlayer(self.videoDisplay_Widget,self.video,tracking)

        self.video_capture_widget=video_capture.UIVideoCapture(self.videoDisplay_Widget,self.video,self.display_updater)

        self.tracking = False
        self.analysis_thread = None
        self.controller = None

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(300)
        self.plot_timer.timeout.connect(self.update_plot)

        self.roisEye = []
        self.roisEllipseEye=[]
        self.tailAngleList=[]
        self.tailPosList=[]

        self.mark = graphMark()
        self.listTailSegments=[]

        self.eyeAxeLine1 = pg.InfiniteLine(movable=False)
        self.eyeAxeLine2 = pg.InfiniteLine(movable=False)
        self.eyeAxeLines=[self.eyeAxeLine1,self.eyeAxeLine2]

        self.initUI()

    def initUI(self):

        self.setWindowIcon(QtgGui.QIcon('Imagys_blue\logoAnimotion-square-112.png'))

        self.area = DockArea()
        self.setCentralWidget(self.area)
        self.resize(1500,800)
        self.setWindowTitle('Xenopus project - beta')

        self.penCyan=pg.mkPen((0,255,255), width=2)
        self.penOrange=pg.mkPen((255,128,0), width=2)
        self.penGreen=pg.mkPen((255,128,0), width=2)

        self.d1 = Dock("Image", size=(1000,500))

        self.d2 = Dock("Layout preferences", size=(500,200))

        self.d3 = Dock("Eye 1", size=(500,200))
        self.d4 = Dock("Eye 2", size=(500,200))
        self.d5 = Dock("Tail", size=(500,200))
        self.d6 = Dock("Eye 1-Y", size=(500,200))
        self.d7 = Dock("Eye 2-Y", size=(500,200))
        self.d8 = Dock("Regions of interest", size=(500,200))
        self.d9 = Dock("Video player", size=(500,200))
        self.d10 = Dock("Segmentation settings", size=(500,200))
        self.d11 = Dock("video Capture ",size=(500,200))
        self.d13 = Dock("Optokinetic", size=(500,200))

        self.area.addDock(self.d1, 'left')
        self.area.addDock(self.d2, 'bottom', self.d1)
        self.area.addDock(self.d3, 'right')
        self.area.addDock(self.d4, 'bottom', self.d3)
        self.area.addDock(self.d5, 'bottom', self.d4)
        self.area.addDock(self.d6, 'bottom', self.d5)
        self.area.addDock(self.d7, 'bottom', self.d6)

        self.area.addDock(self.d11, 'bottom', self.d1)
        self.area.addDock(self.d9, 'above', self.d11)
        self.area.addDock(self.d10, 'above', self.d2)
        self.area.addDock(self.d8, 'above', self.d10)
        self.area.addDock(self.d13, 'above', self.d2)

        self.d1.addWidget(self.videoDisplay_Widget)

        self.videoDisplay_Widget.plotView.addItem(self.mark)

        self.regionlr = pg.LinearRegionItem([0, 0], bounds=[0,0], movable=True)

        self.videoDisplay_Widget.plotView.addItem(self.regionlr)

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

        self.w5 = pg.PlotWidget(title="tail")

        self.d5.addWidget(self.w5)

        self.w6 = pg.PlotWidget(title="Dock 6 plot")

        self.d6.addWidget(self.w6)

        self.w7 = pg.PlotWidget(title="Dock 4 plot")

        self.d7.addWidget(self.w7)

        self.w8 = pg.LayoutWidget()

        self.manipType_comboBox = QtWidgets.QComboBox()
        self.manipType_comboBox.addItem("choose an analysis mode...")
        self.manipType_comboBox.addItem("eyes-tail track")
        self.manipType_comboBox.addItem("eyes track only")
        self.manipType_comboBox.addItem("eyes-limbs track")
        self.manipType_comboBox.addItem("limbs track only")
        self.manipType_comboBox.addItem("test live")

        self.manipType_comboBox.setEnabled(True)

        self.label8 = QtWidgets.QLabel(""" Selection type  """)
        splitter_Selection = QtWidgets.QSplitter()
        splitter_Selection.setOrientation(Qt.Vertical)
        self.selectEyes_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectEyes_radioButton.setToolTip("just click on the eye to trace the roi")
        self.selectEyes_radioButton.setChecked(True)
        self.selectEyes_radioButton.setText("Eye")
        self.selectTailRoot_radioButton = QtWidgets.QRadioButton(splitter_Selection)
        self.selectTailRoot_radioButton.setText("Tail root")

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

        initLimbTrack_btn= QtWidgets.QPushButton('init limbs track')
        initLimbTrack_btn.clicked.connect(self.init_track)

        self.track_checkBox = QtWidgets.QPushButton('Track')
        self.track_checkBox.setCheckable(True)
        self.track_checkBox.setChecked(False)
        self.track_checkBox.clicked.connect(self.toggle_tracking)

        splitter_ThreshTail=QtWidgets.QSplitter()
        tailSplitter_label= QtWidgets.QLabel(splitter_ThreshTail)
        tailSplitter_label.setText("Tail segments")

        tailSegNumber_label= QtWidgets.QLabel(splitter_ThreshTail)
        tailSegNumber_label.setText("number")
        self.tailSegNumber_spinBox =  QtWidgets.QSpinBox(splitter_ThreshTail)
        self.tailSegNumber_spinBox.setValue(1)
        self.tailSegNumber_spinBox.setMinimum(1)
        self.tailSegNumber_spinBox.setMaximum(12)
        self.tailSegNumber_spinBox.valueChanged.connect(self.update_tail_segment_overlay)

        tailSegSize_label= QtWidgets.QLabel(splitter_ThreshTail)
        tailSegSize_label.setText("size")

        self.tailSegSize_spinBox =  QtWidgets.QSpinBox(splitter_ThreshTail)
        self.tailSegSize_spinBox.setValue(6)
        self.tailSegSize_spinBox.setMinimum(6)
        self.tailSegSize_spinBox.setMaximum
        self.tailSegSize_spinBox.valueChanged.connect(self.update_tail_segment_overlay)

        threshTailSlider_label= QtWidgets.QLabel(splitter_ThreshTail)
        threshTailSlider_label.setText("threshold")
        threshTailValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        threshTailValue_label.setText("00")
        threshTailValue_label.setAlignment(Qt.AlignCenter)
        self.threshTail_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.threshTail_slider.setMaximum(255)
        self.threshTail_slider.setPageStep(10)
        self.threshTail_slider.setOrientation(Qt.Horizontal)
        self.threshTail_slider.valueChanged.connect(threshTailValue_label.setNum)
        self.threshTail_slider.valueChanged.connect(self.update_tail_segment_thresh)

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
        self.threshEye1_slider.setValue(60)
        self.threshEye2_slider.setValue(60)

        saveResults_btn = QtWidgets.QPushButton('Save results')
        saveResults_btn.clicked.connect(self.save_ResultsNow)
        resetPlots_btn = QtWidgets.QPushButton('Reset plots')
        resetPlots_btn.clicked.connect(self.reset_Plot)
        resetBuffer_btn = QtWidgets.QPushButton('Reset buffer')
        resetBuffer_btn.clicked.connect(self.reset_Buffer)

        self.w8.addWidget(self.manipType_comboBox, row=0, col=0)
        self.w8.addWidget(self.label8, row=1, col=0)
        self.w8.addWidget(splitter_Background, row=1,col=1,colspan=2)
        self.w8.addWidget(splitter_Selection, row=2, col=0,rowspan=4)
        self.w8.addWidget(splitter_ThreshEyes, row=2, col=1,colspan=3)
        self.w8.addWidget(splitter_ThreshTail, row=3, col=1,colspan=3)

        self.w8.addWidget(initLimbTrack_btn, row=4, col=1)
        self.w8.addWidget(self.track_checkBox,row=5,col=1)
        self.w8.addWidget(saveResults_btn,row=5,col=2)
        self.w8.addWidget(resetPlots_btn,row=4,col=3)
        self.w8.addWidget(resetBuffer_btn,row=5,col=3)
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

        self.optokinetic_Widget=optok.UIOptostim()
        self.d9.addWidget(self.videoPlayer_Widget)
        self.d13.addWidget(self.optokinetic_Widget)

        self.d8.raiseDock()

        self.d11.raiseDock()

        try:
            from Imagys_blue.qsshelper import QSSHelper
            qss = QSSHelper.open_qss(os.path.join('Imagys_blue', 'Imagys-blue.qss'))
            self.setStyleSheet(qss)
        except Exception as e:
            print('QSS non chargé :', e)

        self.show()

    def mouse_clicked(self,evt):
        pos = evt[0].pos()

        if self.videoDisplay_Widget.plotView.sceneBoundingRect().contains(pos):

            mousePointFromScene=self.videoDisplay_Widget.gviewBox.mapToView(pos)

            x0,y0=mousePointFromScene.x(),mousePointFromScene.y()

            if evt[0].button() == 1 and self.selectTailRoot_radioButton.isChecked():

                self.init_markNoseRootTail(x0,y0)
                self.update_tail_segment_overlay()

            if evt[0].button() == 1 and self.selectEyes_radioButton.isChecked():

                w=100
                h=80

                roi_limits=QRectF(0,0,self.video.width,self.video.height)

                self.roisEye.append(Roi([x0-w/2, y0-h/2], [w, h],maxBounds=roi_limits,centered=True, pen=("b"),removable=True))
                self.roisEye[-1].sigRemoveRequested.connect(self.remove_ROI)
                self.videoDisplay_Widget.plotView.addItem(self.roisEye[-1])

                self.roisEllipseEye.append(define_rois.EllipseROI_Centered_NoHandle(pos=self.roisEye[-1].pos(),size=[1,1],pen=(3,5)))
                self.videoDisplay_Widget.plotView.addItem(self.roisEllipseEye[-1])

            if evt[0].button() == 1 and self.selectLimbs_radioButton.isChecked():

                w=100
                h=80

                roisLimb.append(Roi([x0-w/2, y0-h/2], [w, h],centered=True, pen=("m"),removable=True))
                roisLimb[-1].sigRemoveRequested.connect(self.remove_ROI)
                self.videoDisplay_Widget.plotView.addItem(roisLimb[-1])

        else:

            return

    def remove_ROI(self,evt):

         print("remove:",evt)
         self.videoDisplay_Widget.plotView.scene().removeItem(evt)

         index=self.roisEye.index(evt)

         self.roisEye.remove(evt)

         self.videoDisplay_Widget.plotView.scene().removeItem(self.roisEllipseEye[index])
         del self.roisEllipseEye[index]

    def init_markNoseRootTail(self,x0,y0):

        if len(self.mark.data) != 0 :

            tailRoot=self.mark.data['pos'][0]
            nose=self.mark.data['pos'][1]
            tailpos=self.mark.data['pos'][2]

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

        adj = np.array([
            [0,1],
            [0,2]
            ])

        symbols = ['o','o','o']

        symbolBrushes=[(255,0,255),
                       (255,0,255),
                       (0,255,0)
                       ]

        lines = np.array([
            (255,0,255,255,2),
            (0,255,0,255,2)
            ], dtype=[('red',np.ubyte),('green',np.ubyte),('blue',np.ubyte),('alpha',np.ubyte),('width',float)])

        texts=["root","nose","tail"]

        self.mark.size=8
        self.mark.setData(pos=pos, adj=adj, pen=lines, size=self.mark.size, symbolBrush=symbolBrushes,symbolPen='w',symbol=symbols, pxMode=False, text=texts)

    @pyqtSlot()
    def toggle_tracking(self):
        if self.controller is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Controller",
                "AppController n'est pas initialisé dans le main."
            )
            self.track_checkBox.setChecked(False)
            return

        if self.track_checkBox.isChecked():
            ok = self.check_analysis_parameters()

            if not ok:
                self.track_checkBox.setChecked(False)
                return

            self.reset_Plot()
            self.reset_Buffer()

            try:
                if hasattr(self.video_capture_widget, "stop_acquisition"):
                    self.video_capture_widget.stop_acquisition()
                    time.sleep(0.1)

                if hasattr(self.video_capture_widget, "start_acquisition"):
                    self.video_capture_widget.start_acquisition()
                    time.sleep(0.1)
            except Exception as exc:
                print("Erreur start acquisition:", exc)

            try:
                if self.video_capture_widget.trigger_checkBox.isChecked():
                    trigger()
            except Exception:
                pass

            self.controller.start_tracking()
            self.track_checkBox.setText("Stop")
            print("Tracking started with realtime pipeline")

        else:
            try:
                self.controller.stop_tracking()
            except Exception as exc:
                print("Erreur stop pipeline:", exc)

            try:
                if hasattr(self.video_capture_widget, "stop_acquisition"):
                    self.video_capture_widget.stop_acquisition()
            except Exception as exc:
                print("Erreur stop acquisition:", exc)

            self.track_checkBox.setText("Track")
            self.updatePlot_Full()
            print("Tracking stopped")

    def check_analysis_parameters(self):
        ok=True

        if self.manipType_comboBox.currentIndex()==0:
            message="You have to select an analysis mode"
            QtWidgets.QMessageBox.warning(ui,"analysis mode",str(message),QtWidgets.QMessageBox.Ok)
            self.track_checkBox.setChecked(False)
            ok=False

        return ok

    def threshEyeValue_change(self,value,idx):
        if len(self.roisEye)>0 :
            if self.track_checkBox.isChecked()==False :
                print(idx)
                self.update_eyes_overlay(value,idx)

    def update_eyes_overlay(self,value,roiIndex):

        frame=self.video_capture_widget.videoDisplayer_updater.current_frame_to_display

        if not self.track_checkBox.isChecked() and len(self.mark.data)>0:
            if len(self.roisEye)>0 :

                roiEye=self.roisEye[roiIndex]

                xroi,yroi=roiEye.pos()
                wroi,hroi=roiEye.size()

                threshEye=[self.threshEye1_slider.value(),self.threshEye2_slider.value()]

                cnt=eye_segmentation(roiIndex,roiEye,threshEye,frame)

                if cnt :
                    ellipse_pos,anglep=eye_Rotation(roiIndex,roiEye,cnt)

                    if roiIndex==0 :
                        linePen=pg.mkPen(color='c', width=2)
                    elif roiIndex==1 :
                        linePen=pg.mkPen(color=(255,128,0), width=2)

                    self.eyeAxeLines[roiIndex].setPos((ellipse_pos[0],ellipse_pos[1]))
                    self.eyeAxeLines[roiIndex].setAngle(-anglep+90)
                    self.eyeAxeLines[roiIndex].setPen(linePen)
                    self.videoDisplay_Widget.plotView.addItem(self.eyeAxeLines[roiIndex], ignoreBounds=True)

                    x,y,MA,ma,angle,vx,vy,xrot,yrot=self.roisEllipseEye[roiIndex].descriptor
                    self.roisEllipseEye[roiIndex].setPos((x,y),update=False)
                    self.roisEllipseEye[roiIndex].setSize((MA, ma),update=False)
                    self.roisEllipseEye[roiIndex].setAngle(angle,update=False)
                    self.roisEllipseEye[roiIndex].translate(-vx,-vy,update=False)
                    self.roisEllipseEye[roiIndex].stateChanged()

                    self.eyeAxeLines[roiIndex].setPos((x-vx+xrot,y-vy+yrot))
                    self.eyeAxeLines[roiIndex].setAngle(angle+90)

                    if yroi>varM.bodyAxis_Y:
                        angleCorr=-anglep+90-varM.bodyAngle
                    else:
                        angleCorr=anglep+90+varM.bodyAngle

                    print("init angle eye",roiIndex," : ",angleCorr)

                else :
                    print("eye detection failed, change threshold value")

        else :
            msg="no reference for body axe. You can add one with 'tail-root'"
            QtWidgets.QMessageBox.warning(ui,"warning",str(msg),QtWidgets.QMessageBox.Ok)

    def update_tail_segment_thresh(self,thresh_value):
        if not self.track_checkBox.isChecked() and self.selectTailRoot_radioButton.isChecked()==True:

            frame=self.video_capture_widget.videoDisplayer_updater.current_frame_to_display
            iframe,tail_pos,tail_angle=tail_Track(0,frame,thresh_value,self.regionlr.getRegion())
            self.mark.data['pos'][2] = [tail_pos[0],tail_pos[1]]
            self.mark.updateGraph()

    def update_tail_segment_overlay(self):
        if self.track_checkBox.isChecked()==False :

            if len(self.mark.data)>0:
                tailRoot=self.mark.data['pos'][0]
                nose=self.mark.data['pos'][1]
                tail=self.mark.data['pos'][2]

                noseX=nose[0]
                noseY=nose[1]
                rootX=tailRoot[0]
                rootY=tailRoot[1]
                tailX=tail[0]
                tailY=tail[1]

                varM.bodyAxis_Y=noseY

                xv= noseX-rootX
                yv= noseY-rootY

                varM.bodyAngle = math.atan2(yv, xv)* 180 / math.pi
                print("angle de l'axe du corps : ",varM.bodyAngle)

                linePen=pg.mkPen(color='y', width=2)

                self.regionlr.setBounds([0,rootX])
                self.regionlr.setRegion([tailX-10,tailX+10])

                tail_region=self.regionlr.getRegion()

                frame=self.video_capture_widget.videoDisplayer_updater.current_frame_to_display.copy()

                iframe,tail_pos,tail_angle=tail_Track(0,frame,self.threshTail_slider.value(),tail_region)

                self.mark.data['pos'][2] = [tail_pos[0],tail_pos[1]]
                self.mark.updateGraph()

            else :
                msg="no reference for body axe. You can add one with 'tail-root'"
                QtWidgets.QMessageBox.warning(ui,"warning",str(msg),QtWidgets.QMessageBox.Ok)

    def init_track(self):

        bboxes=[]
        if self.video.nbFrames!=None:
            currentFrame=self.video.capture.get(1)-1
            retVal=self.video.capture.set(1,currentFrame)

            (ret, frame) = self.video.capture.read()

            target.tracker =cv2.MultiTracker("MIL")
            if len(roisLimb)>0:

                for roi in roisLimb :
                    print(roi.size())
                    print(roi.pos())
                    x,y=roi.pos()
                    w,h=roi.size()

                    ycv2=self.video.height-y-h
                    bbox=(x, ycv2, w, h)
                    bboxes.append(bbox)

            ok = target.tracker.add(frame,(bboxes))

            print("tracker : ",ok)

    def activate_interface(self,module):

        if module=="load video":
            self.videoPlayer_Widget.playVideo_btn.setEnabled(True)
            self.videoPlayer_Widget.playVideo_btn.setChecked(False)
            self.videoPlayer_Widget.timeLine_slider.setEnabled(True)
            self.videoPlayer_Widget.stepFwdVideo_btn.setEnabled(True)
            self.videoPlayer_Widget.stepBwdVideo_btn.setEnabled(True)

        elif module=="live video":
            self.video_capture_widget.liveVideo_btn.setEnabled(True)

    def reset_Buffer(self):
        framesBuffer[:]=[]
        self.video_capture_widget.lenBuffer_label.setText(str(len(framesBuffer)))

    def reset_Plot(self):

        for eyeEllipse in self.roisEllipseEye :
            eyeEllipse.angleList[:]=[]

            eyeEllipse.yList[:]=[]

        self.tailAngleList[:]=[]

        self.tailPosList[:]=[]

        stimList[:]=[]

        try:

            dataArray = np.asarray([])
            self.w3.plot(dataArray,clear=True)
            self.w4.plot(dataArray,clear=True)
            self.w5.plot(dataArray,clear=True)
            self.w6.plot(dataArray,clear=True)
            self.w7.plot(dataArray,clear=True)
            print("plot reset ok ")
        except ValueError as e :
                QtWidgets.QMessageBox.warning(ui,"Error",str(ValueError),QtWidgets.QMessageBox.Ok)

    def update_plot(self) :
        currentIdx=self.analysis_thread.last_processed_id
        if self.track_checkBox.isChecked()==True:

            dataArray1 = np.asarray(self.roisEllipseEye[0].angleList[currentIdx-200: currentIdx])

            self.w3.plot(dataArray1,pen=self.penCyan,clear=True)

            dataArray2 = np.asarray(self.roisEllipseEye[1].angleList[currentIdx-200: currentIdx])

            self.w4.plot(dataArray2,pen=self.penOrange,clear=True)

            if len(self.tailAngleList)!=0:
                dataArray3 = np.asarray(self.tailAngleList[currentIdx-200: currentIdx])

                self.w5.plot(dataArray3,pen=self.penGreen,clear=True)

    def updatePlot_Full(self) :

        if self.track_checkBox.isChecked()==False:
            if len(self.roisEllipseEye)>0 :

                dataArray1 = np.asarray(self.roisEllipseEye[0].angleList)

                self.w3.plot(dataArray1,pen=self.penCyan,clear=True)

                dataArray2 = np.asarray(self.roisEllipseEye[1].angleList)

                self.w4.plot(dataArray2,pen=self.penOrange,clear=True)

                dataArray4 = np.asarray(self.roisEllipseEye[0].yList)

                self.w6.plot(dataArray4,pen=self.penCyan,clear=True)

                dataArray5 = np.asarray(self.roisEllipseEye[1].yList)

                self.w7.plot(dataArray5,pen=self.penOrange,clear=True)

                if len(self.tailAngleList)!=0:
                    dataArray3 = np.asarray(self.tailAngleList)

                    self.w5.plot(dataArray3,pen=self.penGreen,clear=True)

    def save_ResultsNow(self):
        filename,_ = QtWidgets.QFileDialog.getSaveFileName(ui,"Save track file", "", ".csv")
        print(filename)
        resultFile=self.create_Result(filename,0)
        print(resultFile)
        self.save_Results(resultFile)

    def create_Result(self,filePath,phaseDuration):

        framerate=self.video_capture_widget.resultFPSValue_label.text()

        csvResult=csv.writer(open(filePath+".csv","w",newline=''))

        csvResult.writerow([self.video.path])
        csvResult.writerow(["framerate : "+str(framerate)+' fps'])

        csvResult.writerow(["video size : "+str(self.video.width)+" ; "+str(self.video.height)])
        csvResult.writerow([" "])
        if self.manipType_comboBox.currentText()=="eyes-tail track":
            csvResult.writerow(['frame_idx',"timestamp","eye1 angle","eye2 angle","tail angle","eye1 Ymove","eye2 Ymove","optostim"])

        elif ui.manipType_comboBox.currentText()=="eyes track only":
            csvResult.writerow(['frame_idx',"timestamp","eye1 angle","eye2 angle","eye1 Ymove","eye2 Ymove","optostim_speed","optostim_direction"])

        return csvResult

    def save_Results(self,result_f):

        if self.manipType_comboBox.currentText()=="eyes-tail track":

            print(len(timestampList),len(self.roisEllipseEye[0].angleList),len(self.tailAngleList))
            for i in range(len(timestampList)):

                result_f.writerow([self.roisEllipseEye[0].angleList[i][0],
                                   timestampList[i][1],
                                   self.roisEllipseEye[0].angleList[i][1],
                                   self.roisEllipseEye[1].angleList[i][1],
                                   self.tailAngleList[i][1],
                                   self.roisEllipseEye[0].yList[i][1],
                                   self.roisEllipseEye[1].yList[i][1],

                                   ])

        elif self.manipType_comboBox.currentText()=="eyes track only":

            for i in range(len(self.roisEllipseEye[0].angleList)):

                result_f.writerow([self.roisEllipseEye[0].angleList[i][0],
                                   timestampList[i][1],
                                   self.roisEllipseEye[0].angleList[i][1],
                                   self.roisEllipseEye[1].angleList[i][1],
                                   self.roisEllipseEye[0].yList[i][1],
                                   self.roisEllipseEye[1].yList[i][1],

                                   ])

    def closeEvent(self, event):

        reply = QtWidgets.QMessageBox.question(
            self,
            'User confirm',
            'Have you saved track and optokinetic results ?',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.close_camera()
            event.accept()
        else:
            event.ignore()

        self.video_capture_widget.acquisition_thread.stop()
        if self.analysis_thread :
            self.analysis_thread.stop()

        try :
            print("close stream")
            self.video.grabber.stream.Close()
        except AttributeError as e :

            print("no stream was opened")
        try :
            print("close camera")
            self.video.device.Close()
        except AttributeError as e :

            print("no camera was opened")

        event.accept()

if __name__ == '__main__':

    app = QtWidgets.QApplication([])

    analysisSet = Analysis_Settings()

    tracking = analysis.Tracking()
    video = video_player.Video()
    image = analysis.ImageContainer()

    target = Target()
    varM = Measure_Var()

    ui = UIXenopus(video)

    ui.controller = AppController(
        ui=ui,
        video=ui.video,
        varM=varM
    )

    app.exec_()
    print("the end")
    app.quit()
