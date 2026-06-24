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
import json
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


def _angle_delta_signed(start_angle, end_angle):
    return (end_angle - start_angle + math.pi) % (2.0 * math.pi) - math.pi


def _tail_arc_crop_mask(image_shape, arc_roi):
    """
    Crée un masque seulement sur le petit carré autour de l'arc.
    Pas sur toute l'image : important pour garder le temps réel.
    """
    img_height, img_width = image_shape[:2]

    cx, cy = arc_roi["center"]
    cx = float(cx)
    cy = float(cy)

    inner_radius = max(1.0, float(arc_roi["inner_radius"]))
    outer_radius = max(inner_radius + 1.0, float(arc_roi["outer_radius"]))

    start_angle = float(arc_roi["start_angle"])
    end_angle = float(arc_roi["end_angle"])

    margin = 3

    x0 = max(0, int(cx - outer_radius - margin))
    x1 = min(img_width, int(cx + outer_radius + margin + 1))

    # Coordonnées pyqtgraph -> OpenCV : y_cv = img_height - y_plot.
    y_plot_min = cy - outer_radius - margin
    y_plot_max = cy + outer_radius + margin

    y0 = max(0, int(img_height - y_plot_max))
    y1 = min(img_height, int(img_height - y_plot_min + 1))

    if x1 <= x0 or y1 <= y0:
        return None, None

    yy, xx = np.indices((y1 - y0, x1 - x0), dtype=np.float32)
    xx += x0

    # y en repère pyqtgraph.
    y_plot = img_height - (yy + y0)

    dx = xx - cx
    dy = y_plot - cy

    radius = np.sqrt(dx * dx + dy * dy)
    angle = np.arctan2(dy, dx)

    delta_total = _angle_delta_signed(start_angle, end_angle)
    delta_pixel = (angle - start_angle + math.pi) % (2.0 * math.pi) - math.pi

    if delta_total >= 0:
        angle_ok = (delta_pixel >= 0) & (delta_pixel <= delta_total)
    else:
        angle_ok = (delta_pixel <= 0) & (delta_pixel >= delta_total)

    radius_ok = (radius >= inner_radius) & (radius <= outer_radius)

    mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    mask[radius_ok & angle_ok] = 255

    return (x0, y0, x1, y1), mask


def tail_Track_arc_fast(iframe, analysimg, thresh_tail, arc_roi, append_to_lists=True):
    """
    Même principe que tail_Track(), mais zone = arc.
    Optimisé : on ne traite que le crop autour de l'arc, pas toute l'image.
    """
    if analysimg is None or arc_roi is None:
        return iframe, [0, 0], 180

    if len(analysimg.shape) > 2:
        gray = cv2.cvtColor(analysimg, cv2.COLOR_BGR2GRAY)
    else:
        gray = analysimg

    im_height = gray.shape[0]

    crop_box, arc_mask = _tail_arc_crop_mask(gray.shape, arc_roi)

    if crop_box is None:
        return iframe, [0, 0], 180

    x0, y0, x1, y1 = crop_box
    crop = gray[y0:y1, x0:x1]

    threshTailZone = cv2.threshold(crop, thresh_tail, 255, cv2.THRESH_BINARY_INV)[1]
    threshTailZone = cv2.bitwise_and(threshTailZone, threshTailZone, mask=arc_mask)
    threshTailZone = cv2.dilate(threshTailZone, None, iterations=5)

    tailcnts = cv2.findContours(
        threshTailZone.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )[-2]

    if tailcnts:
        ctail = sorted(tailcnts, key=cv2.contourArea, reverse=True)[:1]
        M = cv2.moments(ctail[0])

        if M["m00"] == 0:
            return iframe, [0, 0], 180

        tailX_local = int(M["m10"] / M["m00"])
        tailY_local = int(M["m01"] / M["m00"])

        ctailX = tailX_local + x0
        ctailY = int(im_height) - (tailY_local + y0)

        rootx, rooty = ui.mark.data['pos'][0]

        tailAngle = ((math.atan2((rooty - ctailY), (rootx - ctailX))) * 180 / math.pi) * (-1)
        tailAngleCorr = tailAngle + varM.bodyAngle

    else:
        print("tail detection failed")
        tailAngleCorr = 180
        ctailX = 0
        ctailY = 0

    tail_pos = [ctailX, ctailY]

    if append_to_lists:
        ui.tailAngleList.append([iframe, tailAngleCorr])
        ui.tailPosList.append([iframe, tail_pos])

    return iframe, tail_pos, tailAngleCorr


class TailArcHandleGraph(pg.GraphItem):
    def __init__(self, on_change=None):
        pg.GraphItem.__init__(self)
        self.on_change = on_change
        self.dragPoint = None
        self.dragOffset = None
        self.textItems = []
        self.data = {
            "pos": np.empty((0, 2), dtype=float),
            "data": np.empty(0, dtype=[("index", int)])
        }

    def set_positions(self, positions, labels=None):
        # Labels masqués volontairement : on garde seulement les petits points.
        for item in self.textItems:
            try:
                item.scene().removeItem(item)
            except Exception:
                pass

        self.textItems = []

        pos = np.asarray(positions, dtype=float)
        data = np.empty(len(pos), dtype=[("index", int)])
        data["index"] = np.arange(len(pos))
        self.data = {"pos": pos.copy(), "data": data}

        self.updateGraph()

    def updateGraph(self):
        pg.GraphItem.setData(
            self,
            pos=self.data["pos"],
            data=self.data["data"],
            size=8,
            symbol="o",
            symbolBrush=(40, 140, 255, 230),
            symbolPen=pg.mkPen("w", width=1),
            pxMode=True
        )

        for i, item in enumerate(self.textItems):
            if i < len(self.data["pos"]):
                item.setPos(self.data["pos"][i][0], self.data["pos"][i][1])

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
            self.dragOffset = self.data["pos"][ind] - pos

        elif ev.isFinish():
            self.dragPoint = None
            return

        else:
            if self.dragPoint is None:
                ev.ignore()
                return

        ind = self.dragPoint.data()[0]
        new_pos = ev.pos() + self.dragOffset
        new_xy = np.array([new_pos.x(), new_pos.y()], dtype=float)

        self.data["pos"][ind] = new_xy
        self.updateGraph()

        if self.on_change is not None:
            self.on_change(ind, new_xy)

        ev.accept()



class TailArcFillItem(QtWidgets.QGraphicsPathItem):
    """
    Zone bleue cliquable : permet de déplacer tout l'arc par clic-glissé.
    """
    def __init__(self, roi):
        QtWidgets.QGraphicsPathItem.__init__(self)
        self.roi = roi
        self._last_pos = None
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setAcceptHoverEvents(True)
        # curseur normal, pour ne pas gêner le clic sur les points

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            # Si on clique près d'une poignée R1/R2/A/B, on laisse passer le clic au GraphItem.
            # Sinon le remplissage bleu peut passer par-dessus et empêcher de prendre les points.
            try:
                p = ev.pos()
                handle_positions = getattr(self.roi.handles, "data", {}).get("pos", [])
                for hp in handle_positions:
                    dx = float(hp[0]) - float(p.x())
                    dy = float(hp[1]) - float(p.y())
                    if (dx * dx + dy * dy) <= (14.0 * 14.0):
                        ev.ignore()
                        return
            except Exception:
                pass

            self._last_pos = ev.pos()
            # curseur normal
            ev.accept()
        else:
            QtWidgets.QGraphicsPathItem.mousePressEvent(self, ev)

    def mouseMoveEvent(self, ev):
        if self._last_pos is None:
            QtWidgets.QGraphicsPathItem.mouseMoveEvent(self, ev)
            return

        pos = ev.pos()
        delta = pos - self._last_pos
        self._last_pos = pos

        self.roi.move_by(delta.x(), delta.y())
        ev.accept()

    def mouseReleaseEvent(self, ev):
        self._last_pos = None
        # curseur normal, pour ne pas gêner le clic sur les points
        ev.accept()


class TailArcROI(object):
    """
    ROI arc pour la queue.
    Elle remplace seulement la zone de recherche rectangle.
    Le tracking reste identique : threshold -> contour -> centre -> angle.
    """
    def __init__(self, plot_view, label="R", color=(255, 0, 0), band_index=0):
        self.plot_view = plot_view
        self.label = str(label)
        self.color = tuple(color)
        self.band_index = int(band_index)

        self.center = np.array([50.0, 150.0], dtype=float)
        self.inner_radius = 35.0
        self.outer_radius = 95.0
        self.start_angle = -0.75
        self.end_angle = 0.75
        self.initialized = False

        self._dirty_version = 0
        self._curve_reference = None
        self._last_curve_value = 50.0

        self.fill_item = TailArcFillItem(self)
        self.fill_item.setPen(pg.mkPen(color=self.color, width=1))
        self.fill_item.setBrush(QtgGui.QBrush(QtgGui.QColor(
            int(self.color[0]), int(self.color[1]), int(self.color[2]), 45
        )))

        pen = pg.mkPen(color=self.color, width=2)
        side_pen = pg.mkPen(color=self.color, width=1)

        self.inner_curve = pg.PlotDataItem(x=[], y=[], pen=pen)
        self.outer_curve = pg.PlotDataItem(x=[], y=[], pen=pen)
        self.side_curve_1 = pg.PlotDataItem(x=[], y=[], pen=side_pen)
        self.side_curve_2 = pg.PlotDataItem(x=[], y=[], pen=side_pen)
        self.handles = TailArcHandleGraph(on_change=self._handle_moved)
        # Label R/M/C placé en haut à gauche de chaque arc.
        self.label_item = pg.TextItem(self.label, color=self.color, anchor=(0.0, 1.0))

        self.fill_item.setZValue(800)
        self.plot_view.addItem(self.fill_item)

        for item in [
            self.inner_curve,
            self.outer_curve,
            self.side_curve_1,
            self.side_curve_2,
        ]:
            item.setZValue(900)
            self.plot_view.addItem(item)

        self.handles.setZValue(2000)
        self.plot_view.addItem(self.handles)

        self.label_item.setZValue(2100)
        self.plot_view.addItem(self.label_item)

        self.set_visible(False)

    def set_visible(self, visible):
        for item in [
            self.fill_item,
            self.inner_curve,
            self.outer_curve,
            self.side_curve_1,
            self.side_curve_2,
            self.handles,
            self.label_item,
        ]:
            item.setVisible(visible)

    def move_by(self, dx, dy):
        self.center = self.center + np.array([float(dx), float(dy)], dtype=float)

        if self._curve_reference is not None:
            self._curve_reference["mid_point"] = self._curve_reference["mid_point"] + np.array([float(dx), float(dy)], dtype=float)

        self._dirty_version += 1
        self.update_graph()

    def initialize_from_points(self, root, nose, tail):
        root = np.asarray(root, dtype=float)
        nose = np.asarray(nose, dtype=float)
        tail = np.asarray(tail, dtype=float)

        direction = tail - root
        length = np.linalg.norm(direction)

        if length < 1:
            direction = root - nose
            length = np.linalg.norm(direction)

        if length < 1:
            return

        angle_mid = math.atan2(direction[1], direction[0])

        # ------------------------------------------------------------------
        # Taille par défaut de l'arc
        # ------------------------------------------------------------------
        # Ancien problème :
        # la taille dépendait surtout de la distance root -> tail.
        # Au démarrage, cette distance peut être très petite, donc l'arc était
        # collé au root et minuscule.
        #
        # Ici, on force une taille basée sur la hauteur de l'image.
        # Résultat : arc déjà grand, à gauche du root, avec une vraie distance.
        # ------------------------------------------------------------------
        try:
            img_h = float(ui.video.height)
        except Exception:
            img_h = max(300.0, length * 4.0)

        self.center = root.copy()

        # Trois bandes le long de la queue :
        # R = proche de la racine, M = milieu, C = plus caudal.
        base_length = max(float(length), img_h * 0.45)
        fractions = [0.28, 0.52, 0.76]
        index = max(0, min(2, int(self.band_index)))

        root_gap = max(25.0, base_length * fractions[index])
        arc_width = max(20.0, base_length * 0.11)

        self.inner_radius = root_gap
        self.outer_radius = root_gap + arc_width

        # Ouverture un peu plus grande pour couvrir les mouvements latéraux.
        self.start_angle = angle_mid - math.radians(60.0)
        self.end_angle = angle_mid + math.radians(60.0)

        self.initialized = True
        self._dirty_version += 1

        self._save_curve_reference()
        self._last_curve_value = 50.0
        self.set_visible(True)
        self.update_graph()

        try:
            slider = ui.get_tail_arc_curve_slider(self.label)
            slider.blockSignals(True)
            slider.setValue(40)
            slider.blockSignals(False)
            self.set_curve_value(40)
        except Exception:
            pass

    def _save_curve_reference(self):
        mid_angle = self._angle_mid()
        mid_radius = (self.inner_radius + self.outer_radius) / 2.0
        width = max(2.0, self.outer_radius - self.inner_radius)
        delta = abs(_angle_delta_signed(self.start_angle, self.end_angle))

        if delta < math.radians(3.0):
            delta = math.radians(3.0)

        direction = np.array([math.cos(mid_angle), math.sin(mid_angle)], dtype=float)
        mid_point = self.center + mid_radius * direction

        self._curve_reference = {
            "mid_point": mid_point.copy(),
            "mid_angle": float(mid_angle),
            "mid_radius": float(mid_radius),
            "width": float(width),
            "arc_length": float(mid_radius * delta),
            "sign": 1.0 if _angle_delta_signed(self.start_angle, self.end_angle) >= 0 else -1.0,
        }

    def _update_curve_reference_width_only(self):
        """
        Quand on bouge R1/R2, on change seulement l'épaisseur de l'arc.
        On ne ré-écrit pas mid_radius / arc_length, sinon le slider curve perd son amplitude.
        """
        if self._curve_reference is None:
            self._save_curve_reference()
            return

        width = max(2.0, self.outer_radius - self.inner_radius)
        self._curve_reference["width"] = float(width)

    def _curve_scale_from_value(self, value):
        value = max(0.0, min(100.0, float(value)))

        if value <= 50.0:
            # 50 -> x1, 0 -> x4 : plus droit
            return 1.0 + ((50.0 - value) / 50.0) * 3.0

        # 50 -> x1, 100 -> x0.5 : plus courbé
        return 1.0 - ((value - 50.0) / 50.0) * 0.5

    def set_curve_value(self, value):
        """
        Réglage de l'arrondi SANS repositionner au point de départ.
        On transforme la géométrie actuelle, pas une ancienne référence sauvegardée.
        """
        if not self.initialized:
            return

        value = max(0.0, min(100.0, float(value)))

        previous_value = getattr(self, "_last_curve_value", 50.0)

        previous_scale = self._curve_scale_from_value(previous_value)
        new_scale = self._curve_scale_from_value(value)

        if previous_scale <= 0:
            previous_scale = 1.0

        ratio = new_scale / previous_scale

        mid_angle = self._angle_mid()
        mid_radius = max(1.0, (self.inner_radius + self.outer_radius) / 2.0)
        width = max(2.0, self.outer_radius - self.inner_radius)

        delta = _angle_delta_signed(self.start_angle, self.end_angle)
        sign = 1.0 if delta >= 0 else -1.0
        delta_abs = max(math.radians(3.0), abs(delta))

        direction = np.array([math.cos(mid_angle), math.sin(mid_angle)], dtype=float)
        mid_point = self.center + mid_radius * direction

        # Longueur actuelle de l'arc au milieu de l'épaisseur.
        # On la conserve pour que l'arc se courbe/décourbe sans sauter.
        arc_length = mid_radius * delta_abs

        new_mid_radius = max(width / 2.0 + 2.0, mid_radius * ratio)

        self.center = mid_point - new_mid_radius * direction

        self.inner_radius = max(2.0, new_mid_radius - width / 2.0)
        self.outer_radius = self.inner_radius + width

        new_delta = arc_length / max(new_mid_radius, 1.0)
        new_delta = max(math.radians(3.0), min(math.radians(170.0), new_delta))
        new_delta *= sign

        self.start_angle = mid_angle - new_delta / 2.0
        self.end_angle = mid_angle + new_delta / 2.0

        self._last_curve_value = value
        self._dirty_version += 1
        self.update_graph()


    def get_parameters(self):
        return {
            "center": [float(self.center[0]), float(self.center[1])],
            "inner_radius": float(self.inner_radius),
            "outer_radius": float(self.outer_radius),
            "start_angle": float(self.start_angle),
            "end_angle": float(self.end_angle),
            "version": int(self._dirty_version),
        }

    def _angle_mid(self):
        delta = _angle_delta_signed(self.start_angle, self.end_angle)
        return self.start_angle + delta / 2.0

    def _arc_points(self, radius, n=50):
        delta = _angle_delta_signed(self.start_angle, self.end_angle)
        angles = self.start_angle + np.linspace(0.0, delta, n)

        x = self.center[0] + radius * np.cos(angles)
        y = self.center[1] + radius * np.sin(angles)

        return x, y, angles

    def update_graph(self):
        self.inner_radius = max(2.0, float(self.inner_radius))
        self.outer_radius = max(self.inner_radius + 2.0, float(self.outer_radius))

        inner_x, inner_y, _ = self._arc_points(self.inner_radius)
        outer_x, outer_y, _ = self._arc_points(self.outer_radius)

        self.inner_curve.setData(inner_x, inner_y)
        self.outer_curve.setData(outer_x, outer_y)

        self.side_curve_1.setData([inner_x[0], outer_x[0]], [inner_y[0], outer_y[0]])
        self.side_curve_2.setData([inner_x[-1], outer_x[-1]], [inner_y[-1], outer_y[-1]])

        path = QtgGui.QPainterPath()
        path.moveTo(float(outer_x[0]), float(outer_y[0]))

        for x_val, y_val in zip(outer_x[1:], outer_y[1:]):
            path.lineTo(float(x_val), float(y_val))

        for x_val, y_val in zip(inner_x[::-1], inner_y[::-1]):
            path.lineTo(float(x_val), float(y_val))

        path.closeSubpath()
        self.fill_item.setPath(path)

        angle_mid = self._angle_mid()
        inner_mid = self.center + self.inner_radius * np.array([math.cos(angle_mid), math.sin(angle_mid)])
        outer_mid = self.center + self.outer_radius * np.array([math.cos(angle_mid), math.sin(angle_mid)])
        start_handle = self.center + self.outer_radius * np.array([math.cos(self.start_angle), math.sin(self.start_angle)])
        end_handle = self.center + self.outer_radius * np.array([math.cos(self.end_angle), math.sin(self.end_angle)])

        self.handles.set_positions(
            [inner_mid, outer_mid, start_handle, end_handle],
            ["R1", "R2", "A", "B"]
        )

        try:
            # Position du label : coin haut gauche de l'arc.
            # On utilise tous les points de l'arc pour trouver x minimum et y maximum.
            all_x = np.concatenate([inner_x, outer_x])
            all_y = np.concatenate([inner_y, outer_y])
            label_x = float(np.min(all_x))
            label_y = float(np.max(all_y) + 6.0)
            self.label_item.setText(self.label)
            self.label_item.setPos(label_x, label_y)
        except Exception:
            pass

    def _handle_moved(self, index, pos):
        try:
            if index == 0:
                # R1 : rayon interne.
                self.inner_radius = max(2.0, np.linalg.norm(pos - self.center))
                if self.inner_radius >= self.outer_radius - 2.0:
                    self.inner_radius = self.outer_radius - 2.0

                self._update_curve_reference_width_only()

            elif index == 1:
                # R2 : rayon externe.
                self.outer_radius = max(self.inner_radius + 2.0, np.linalg.norm(pos - self.center))

                self._update_curve_reference_width_only()

            elif index == 2:
                # A : bord angulaire 1.
                # Agrandissement symétrique : B bouge en miroir autour de l'axe central.
                mid_angle = self._angle_mid()
                moved_angle = math.atan2(pos[1] - self.center[1], pos[0] - self.center[0])

                half_width = abs(_angle_delta_signed(mid_angle, moved_angle))
                half_width = max(math.radians(3.0), min(math.radians(85.0), half_width))

                sign = 1.0 if _angle_delta_signed(self.start_angle, self.end_angle) >= 0 else -1.0

                self.start_angle = mid_angle - sign * half_width
                self.end_angle = mid_angle + sign * half_width
                self._save_curve_reference()
                self._last_curve_value = 50.0

                try:
                    slider = ui.get_tail_arc_curve_slider(self.label)
                    slider.blockSignals(True)
                    slider.setValue(50)
                    slider.blockSignals(False)
                except Exception:
                    pass

            elif index == 3:
                # B : bord angulaire 2.
                # Agrandissement symétrique : A bouge en miroir autour de l'axe central.
                mid_angle = self._angle_mid()
                moved_angle = math.atan2(pos[1] - self.center[1], pos[0] - self.center[0])

                half_width = abs(_angle_delta_signed(mid_angle, moved_angle))
                half_width = max(math.radians(3.0), min(math.radians(85.0), half_width))

                sign = 1.0 if _angle_delta_signed(self.start_angle, self.end_angle) >= 0 else -1.0

                self.start_angle = mid_angle - sign * half_width
                self.end_angle = mid_angle + sign * half_width
                self._save_curve_reference()
                self._last_curve_value = 50.0

                try:
                    slider = ui.get_tail_arc_curve_slider(self.label)
                    slider.blockSignals(True)
                    slider.setValue(50)
                    slider.blockSignals(False)
                except Exception:
                    pass

            self._dirty_version += 1
            self.update_graph()

        except Exception as exc:
            print("TailArcROI drag error:", exc)



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
        # Ancien Video player supprimé de l'interface.
        # L'analyse Imported video utilise son propre lecteur/review frame par frame.
        self.videoPlayer_Widget = None

        self.video_capture_widget=video_capture.UIVideoCapture(self.videoDisplay_Widget,self.video,self.display_updater)

        self.tracking = False
        self.analysis_thread = None
        self.result_save_dir = None
        self.controller = None

        # Interface vidéo importée.
        self.imported_video_path = None
        self.imported_video_frame = None
        self.imported_video_total_frames = 0
        self.imported_video_raw_width = 0
        self.imported_video_raw_height = 0
        self.imported_review_last_auto_frame = -1
        self.imported_review_ignore_signals = False

        # Layout spécial du mode Imported video :
        # les docks camera/player sont compressés et l'image prend la place libérée.
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

        self.initUI()

        self.video_file_status_timer = QTimer()
        self.video_file_status_timer.setInterval(250)
        self.video_file_status_timer.timeout.connect(self.update_imported_video_status)

    def initUI(self):

        self.setWindowIcon(QtgGui.QIcon(os.path.join('Imagys_blue', 'logoAnimotion-square-112.png')))

        self.area = DockArea()
        self.setCentralWidget(self.area)
        self.resize(1500,800)
        self.setWindowTitle('Xenopus project - beta')

        # Sélecteur global du mode d'analyse.
        # Il évite d'avoir le live/caméra et la vidéo importée actifs en même temps.
        self.analysis_mode = "live"
        self.analysisMode_toolbar = self.addToolBar("Analysis mode")
        self.analysisMode_toolbar.setMovable(False)

        self.analysisMode_label = QtWidgets.QLabel("Analysis mode: ")

        # Onglets globaux en haut de la fenêtre.
        # Plus clair qu'un petit menu déroulant : un seul mode actif à la fois.
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

        # Les boutons Save/Load settings ne sont pas dans la toolbar globale.
        # Ils sont placés dans le panneau du mode actif :
        # - Real-time camera : dock video Capture ;
        # - Imported video : dock Imported video progress.

        # Barre dédiée au mode vidéo importée.
        # Elle est masquée en mode temps réel, donc l'interface ne montre pas les deux modes en même temps.
        self.importedVideo_toolbar = self.addToolBar("Imported video tools")
        self.importedVideo_toolbar.setMovable(False)
        self.importedVideo_toolbar.hide()

        self.penCyan=pg.mkPen((0,255,255), width=2)
        self.penOrange=pg.mkPen((255,128,0), width=2)
        self.penGreen=pg.mkPen((0,255,0), width=2)

        # Couleurs type HSV pour les trois arcs de queue.
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
        # Dock Video player supprimé : il fusionnait parfois avec video Capture après Load settings.
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
        # Le dock d14 n'est plus ajouté dans le DockArea.
        # Les contrôles vidéo importée sont maintenant dans une barre dédiée en haut,
        # visible uniquement en mode Imported video.

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

        # Lignes et points de tracking indépendants pour chaque arc.
        # Chaque arc a son propre point détecté et sa propre droite root -> point.
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

        # Compatibilité avec l'ancien code qui attend self.tailArcROI.
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

        # Choix des arcs à afficher et à analyser.
        # Par défaut : R actif, M et C inactifs.
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

        # Compatibilité avec l'ancien code : tailArcCurve_slider pointe vers R.
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

        # Layout compact : ligne 0 = activation R/M/C,
        # ligne 1 = seuils, ligne 2 = courbures.
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
        # Panel ROI / segmentation : layout plus propre et plus lisible
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
        # Limb et Circle sont gardés dans le code pour compatibilité,
        # mais retirés de l'interface pour libérer de la place.
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

        # Petit style local : plus d'air, sans toucher au thème global Imagys.
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

        # Boutons settings du mode Real-time camera.
        # On les place dans le dock video Capture, pas dans la toolbar globale,
        # pour garder la même logique que le panneau Imported video.
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
        # Interface dédiée à l'analyse d'une vidéo importée.
        # Elle est volontairement séparée du video player et de la capture caméra.
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

        # Barre de progression dédiée placée juste sous l'image en mode Imported video.
        self.importedProgressWidget = QtWidgets.QWidget()
        self.importedProgressLayout = QtWidgets.QGridLayout(self.importedProgressWidget)
        self.importedProgressLayout.setContentsMargins(8, 4, 8, 4)
        self.importedProgressLayout.setHorizontalSpacing(8)
        self.importedProgressLayout.setVerticalSpacing(3)

        self.importedProgressTitle_label = QtWidgets.QLabel("Imported video")
        self.importedProgressTitle_label.setMinimumWidth(95)

        # Ligne 0 : actions vidéo
        self.importedProgressLayout.addWidget(self.importedProgressTitle_label, 0, 0)
        self.importedProgressLayout.addWidget(self.openImportedVideo_btn, 0, 1)
        self.importedProgressLayout.addWidget(self.importedVideoPath_label, 0, 2, 1, 2)
        self.importedProgressLayout.addWidget(self.startImportedVideo_btn, 0, 4)
        self.importedProgressLayout.addWidget(self.stopImportedVideo_btn, 0, 5)

        # Ligne 1 : progression
        self.importedProgressLayout.addWidget(self.importedVideoProgress, 1, 0, 1, 4)
        self.importedProgressLayout.addWidget(self.importedVideoStatus_label, 1, 4, 1, 2)

        # Ligne 2 : navigation frame par frame après / pendant analyse
        self.importedProgressLayout.addWidget(self.reviewFrameLabel, 2, 0)
        self.importedProgressLayout.addWidget(self.reviewFramePrev_btn, 2, 1)
        self.importedProgressLayout.addWidget(self.reviewFrameSlider, 2, 2, 1, 2)
        self.importedProgressLayout.addWidget(self.reviewFrameSpin, 2, 4)
        self.importedProgressLayout.addWidget(self.reviewFrameNext_btn, 2, 5)
        self.importedProgressLayout.addWidget(self.previewDuringAnalysis_ckb, 3, 0, 1, 6)

        # Lignes 4 à 7 : crop de la vidéo importée.
        # Plus de QSpinBox visibles : labels + barres de navigation.
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

        # Les contrôles vidéo importée ne sont plus dans la toolbar du haut.
        # Ils sont placés dans le bandeau juste sous l'image, avec la progression.
        # La toolbar reste inutilisée pour éviter une interface haute trop chargée.
        try:
            self.importedVideoPath_label.setMinimumWidth(260)
            self.importedVideoProgress.setMinimumHeight(18)
            self.importedVideoStatus_label.setMinimumWidth(320)
            self.openImportedVideo_btn.setMaximumWidth(120)
            self.startImportedVideo_btn.setMaximumWidth(95)
            self.stopImportedVideo_btn.setMaximumWidth(70)
        except Exception:
            pass

        # Démarrage par défaut en mode live/direct.
        # Le dock vidéo importée est masqué tant que le mode Imported video n'est pas choisi.
        try:
            self._set_dock_clean_visible(self.d15, False)
        except Exception:
            pass

        self.set_analysis_mode("live", update_combo=True)

        # Avec le dock Video player supprimé, video Capture n'est plus dans un
        # conteneur tabulé. raiseDock() peut donc lever une erreur selon la
        # version de pyqtgraph. On garde l'appel seulement si possible.
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
            print('QSS non chargé :', e)

        self.show()

    def update_eye_roi_label_positions(self):
        for i, roi in enumerate(self.roisEye):
            if i >= len(self.roisEyeLabels):
                continue

            try:
                label = self.roisEyeLabels[i]
                label.setText("Eye{}".format(i + 1))

                w, h = roi.size()
                label.setAnchor((1, 0))
                label.setPos(float(w) - 4.0, 4.0)

            except Exception:
                pass

    def refresh_eye_roi_labels(self):
        for i, label in enumerate(self.roisEyeLabels):
            try:
                label.setText("Eye{}".format(i + 1))
            except Exception:
                pass

        self.update_eye_roi_label_positions()

    def add_eye_roi_label(self, roi):
        idx = len(self.roisEyeLabels) + 1

        label = pg.TextItem(
            "Eye{}".format(idx),
            color=(120, 170, 255, 120),
            anchor=(1, 0)
        )
        label.setZValue(1200)
        label.setParentItem(roi)

        self.roisEyeLabels.append(label)

        try:
            roi.sigRegionChanged.connect(self.update_eye_roi_label_positions)
        except Exception:
            pass

        self.update_eye_roi_label_positions()
        return label

    def mouse_clicked(self,evt):
        pos = evt[0].pos()

        if self.videoDisplay_Widget.plotView.sceneBoundingRect().contains(pos):

            mousePointFromScene=self.videoDisplay_Widget.gviewBox.mapToView(pos)

            x0,y0=mousePointFromScene.x(),mousePointFromScene.y()

            if evt[0].button() == 1 and self.selectTailRoot_radioButton.isChecked():

                self.init_markNoseRootTail(x0,y0)
                self.update_tail_segment_overlay()

            if evt[0].button() == 1 and self.selectEyes_radioButton.isChecked():

                if len(self.roisEye) >= 2:
                    print("Maximum 2 eye ROIs.")
                    return

                w=100
                h=80

                roi_limits=QRectF(0,0,self.video.width,self.video.height)

                self.roisEye.append(Roi([x0-w/2, y0-h/2], [w, h],maxBounds=roi_limits,centered=True, pen=("b"),removable=True))
                self.roisEye[-1].sigRemoveRequested.connect(self.remove_ROI)
                self.videoDisplay_Widget.plotView.addItem(self.roisEye[-1])
                self.add_eye_roi_label(self.roisEye[-1])

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

         index=self.roisEye.index(evt)

         if index < len(self.roisEyeLabels):
             try:
                 self.videoDisplay_Widget.plotView.scene().removeItem(self.roisEyeLabels[index])
             except Exception:
                 pass
             del self.roisEyeLabels[index]

         self.videoDisplay_Widget.plotView.scene().removeItem(evt)

         self.roisEye.remove(evt)

         self.videoDisplay_Widget.plotView.scene().removeItem(self.roisEllipseEye[index])
         del self.roisEllipseEye[index]

         self.refresh_eye_roi_labels()

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

    def is_tail_arc_enabled(self, label):
        """
        Indique si l'arc R/M/C est actif.
        Un arc décoché est masqué et n'est pas envoyé au tracking.
        """
        label = str(label).upper()
        checkbox = getattr(self, "tailArc{}_checkBox".format(label), None)

        if checkbox is None:
            return True

        return bool(checkbox.isChecked())

    def update_tail_arc_enabled_states(self, *args, update_preview=True):
        """
        Synchronise les checkbox R/M/C avec :
        - l'état enabled des sliders thresh / curve ;
        - la visibilité des arcs ;
        - les marqueurs de tracking.
        """
        try:
            for label in ["R", "M", "C"]:
                enabled = self.is_tail_arc_enabled(label)

                for widget in getattr(self, "tailArcControlWidgets", {}).get(label, []):
                    try:
                        widget.setEnabled(enabled)
                    except Exception:
                        pass

                roi = self.get_tail_arc_rois().get(label)
                if roi is not None:
                    try:
                        roi.set_visible(enabled and roi.initialized)
                    except Exception:
                        pass

                if not enabled:
                    self.set_tail_arc_tracking_marker(label, None)

            if update_preview and not self.track_checkBox.isChecked():
                if self.selectTailRoot_radioButton.isChecked() and len(self.mark.data) > 0:
                    self.update_tail_segment_thresh(self.threshTail_slider.value())

        except Exception as exc:
            print("update_tail_arc_enabled_states error:", exc)

    def get_tail_arc_roi_params(self):
        # Compatibilité avec l'ancien tracking : retourne R uniquement si R est actif.
        if self.is_tail_arc_enabled("R") and hasattr(self, "tailArcROI_R") and self.tailArcROI_R.initialized:
            return self.tailArcROI_R.get_parameters()
        return None

    def get_tail_arc_roi_params_all(self):
        params = {}

        # Important : on retourne un dictionnaire, même vide.
        # Ainsi, si aucun arc n'est coché, le tracking ne retombe pas sur
        # l'ancien tracking rectangulaire de queue.
        for label, roi in self.get_tail_arc_rois().items():
            if not self.is_tail_arc_enabled(label):
                continue

            if roi.initialized:
                params[label] = roi.get_parameters()

        return params

    def get_tail_arc_rois(self):
        if hasattr(self, "tailArcROIs"):
            return self.tailArcROIs

        if hasattr(self, "tailArcROI"):
            return {"R": self.tailArcROI}

        return {}

    def get_tail_arc_thresholds(self):
        return {
            "R": int(self.threshTail_slider.value()),
            "M": int(self.threshTailM_slider.value()) if hasattr(self, "threshTailM_slider") else int(self.threshTail_slider.value()),
            "C": int(self.threshTailC_slider.value()) if hasattr(self, "threshTailC_slider") else int(self.threshTail_slider.value()),
        }

    def get_tail_arc_curve_slider(self, label):
        label = str(label).upper()

        if label == "M" and hasattr(self, "tailArcCurveM_slider"):
            return self.tailArcCurveM_slider

        if label == "C" and hasattr(self, "tailArcCurveC_slider"):
            return self.tailArcCurveC_slider

        if hasattr(self, "tailArcCurveR_slider"):
            return self.tailArcCurveR_slider

        return self.tailArcCurve_slider

    def set_tail_arc_tracking_marker(self, label, tail_pos):
        """
        Affiche un point de queue et une droite root -> point pour un arc donné.
        Il y a donc une droite R, une droite M et une droite C.
        """
        try:
            label = str(label).upper()
            line_item = self.tailArcTrackingLines.get(label)
            point_item = self.tailArcTrackingPoints.get(label)

            if line_item is None or point_item is None:
                return

            if tail_pos is not None and not self.is_tail_arc_enabled(label):
                tail_pos = None

            if tail_pos is None or len(tail_pos) < 2:
                line_item.setData([], [])
                point_item.setData([], [])
                return

            x_pos = float(tail_pos[0])
            y_pos = float(tail_pos[1])

            if x_pos == 0 and y_pos == 0:
                line_item.setData([], [])
                point_item.setData([], [])
                return

            root = self.mark.data['pos'][0]
            root_x = float(root[0])
            root_y = float(root[1])

            line_item.setData([root_x, x_pos], [root_y, y_pos])
            point_item.setData([x_pos], [y_pos])

        except Exception as exc:
            print("set_tail_arc_tracking_marker error:", exc)

    def clear_tail_arc_tracking_markers(self):
        for label in ["R", "M", "C"]:
            self.set_tail_arc_tracking_marker(label, None)

    def _selected_tail_arc_labels(self):
        return [label for label in ["R", "M", "C"] if self.is_tail_arc_enabled(label)]

    def _initialize_tail_arcs_if_needed(self):
        if len(self.mark.data) == 0:
            return False

        tailRoot = self.mark.data['pos'][0]
        nose = self.mark.data['pos'][1]
        tail = self.mark.data['pos'][2]

        for label, roi in self.get_tail_arc_rois().items():
            if not roi.initialized:
                roi.initialize_from_points(tailRoot, nose, tail)

        self.update_tail_arc_enabled_states(update_preview=False)
        return True

    def get_current_analysis_frame(self):
        """
        Retourne la frame à utiliser pour les prévisualisations.

        En mode vidéo importée, on utilise la première frame de la vidéo.
        En mode live, on utilise la frame courante affichée par la caméra.
        """
        try:
            if getattr(self, "imported_video_frame", None) is not None:
                if getattr(self, "analysis_mode", "live") == "video":
                    return self.imported_video_frame
        except Exception:
            pass

        try:
            updater = getattr(self.video_capture_widget, "videoDisplayer_updater", None)

            if updater is not None:
                frame = getattr(updater, "current_frame_to_display", None)

                if frame is not None:
                    return frame
        except Exception:
            pass

        return getattr(self, "imported_video_frame", None)

    def show_gray_frame_in_image_dock(self, gray_frame):
        """
        Affiche une frame grayscale directement dans le plot image existant.

        On évite show_frame_in_pyqtgraph() ici pour ne pas dépendre du LUT live/caméra.
        """
        try:
            self.video.currentGrayFrame = gray_frame.copy()
            self.video.currentFrame = gray_frame.copy()
            self.video.currentTFrame = cv2.transpose(gray_frame[::-1, :])
            self.videoDisplay_Widget.img.setImage(
                self.video.currentTFrame,
                autoLevels=False
            )
        except Exception as exc:
            print("show_gray_frame_in_image_dock error:", exc)

    def _remember_live_dock_heights(self):
        """
        Sauvegarde les hauteurs actuelles des docks live pour pouvoir revenir proprement.
        """
        if getattr(self, "_live_dock_height_cache", None):
            return

        for name, dock in [("d1", self.d1), ("d11", self.d11)]:
            try:
                self._live_dock_height_cache[name] = {
                    "min": dock.minimumHeight(),
                    "max": dock.maximumHeight(),
                    "height": dock.height(),
                }
            except Exception:
                pass

    def _force_dock_height(self, dock, height):
        """
        Force une hauteur provisoire pour un dock.
        """
        try:
            dock.setMinimumHeight(int(height))
            dock.setMaximumHeight(int(height))
        except Exception:
            pass

    def _release_dock_height(self, dock):
        """
        Redonne au dock une hauteur libre.
        """
        try:
            dock.setMinimumHeight(0)
            dock.setMaximumHeight(16777215)
        except Exception:
            pass

    def _apply_imported_video_layout(self):
        """
        En mode Imported video, on remplit le vide en agrandissant la zone Image.
        """
        self._remember_live_dock_heights()

        # Camera compressée en mode Imported video : elle n'a pas besoin d'occuper de hauteur.
        try:
            self._force_dock_height(self.d11, 0)
            self._force_dock_height(self.d15, 232)
        except Exception:
            pass

        # Zone image plus haute, mais on garde une petite bande utile pour la progression.
        try:
            self._force_dock_height(self.d1, self._video_mode_image_height)
        except Exception:
            pass

        try:
            self.area.resizeDocks(
                [self.d1, self.d15, self.d11],
                [self._video_mode_image_height, 205, 1],
                "vertical"
            )
        except Exception:
            pass

        try:
            self.videoDisplay_Widget.plotView.setAspectLocked(True)
            self.videoDisplay_Widget.plotView.autoRange()
        except Exception:
            pass

    def _restore_live_layout(self):
        """
        Restaure un layout normal en mode Real-time camera.
        """
        try:
            self._release_dock_height(self.d1)
            self._release_dock_height(self.d11)
            self._release_dock_height(self.d15)
        except Exception:
            pass

        try:
            self.area.resizeDocks(
                [self.d1, self.d11, self.d15],
                [360, 210, 1],
                "vertical"
            )
        except Exception:
            pass


    def on_analysis_mode_changed(self, index):
        """
        Appelé par les onglets globaux du haut.
        0 = temps réel/caméra
        1 = vidéo importée
        """
        if index == 1:
            self.set_analysis_mode("video", update_combo=False)
        else:
            self.set_analysis_mode("live", update_combo=False)

    def _is_live_tracking_running(self):
        try:
            if self.track_checkBox.isChecked():
                return True
        except Exception:
            pass

        try:
            if self.controller is not None:
                if self.controller.pipeline is not None and self.controller.pipeline.running:
                    return True
        except Exception:
            pass

        return False

    def _is_video_file_analysis_running(self):
        try:
            if self.controller is None:
                return False

            status = self.controller.get_video_file_analysis_status()
            return bool(status.get("running", False))

        except Exception:
            return False

    def _set_dock_clean_visible(self, dock, visible):
        """
        Affiche ou masque un Dock pyqtgraph en limitant les espaces vides.

        hide() seul peut laisser un grand espace ou une barre de titre selon
        l'organisation des docks. On combine donc hide/show + hauteur max.
        """
        if dock is None:
            return

        try:
            if visible:
                dock.setMaximumHeight(16777215)
                dock.setMinimumHeight(0)
                dock.show()
            else:
                dock.hide()
                dock.setMinimumHeight(0)
                dock.setMaximumHeight(0)
        except Exception:
            pass

        try:
            if hasattr(dock, "label"):
                dock.label.setVisible(visible)
        except Exception:
            pass

    def set_analysis_mode(self, mode, update_combo=True):
        """
        Active un seul mode d'analyse à la fois.

        live  : caméra + video capture/player.
        video : vidéo importée, sans capture caméra ni video player.

        Les contrôles de l'autre mode sont vraiment masqués pour éviter
        l'interface vide et les conflits inutiles.
        """
        if mode not in ("live", "video"):
            mode = "live"

        # Protection : pas de bascule pendant une analyse en cours.
        if mode == "video" and self._is_live_tracking_running():
            QtWidgets.QMessageBox.warning(
                self,
                "Analysis mode",
                "Stop real-time tracking before switching to imported video mode."
            )
            self._sync_analysis_mode_combo("live")
            return

        if mode == "live" and self._is_video_file_analysis_running():
            QtWidgets.QMessageBox.warning(
                self,
                "Analysis mode",
                "Stop imported video analysis before switching to real-time mode."
            )
            self._sync_analysis_mode_combo("video")
            return

        self.analysis_mode = mode
        self._sync_analysis_mode_combo(mode)

        if mode == "video":
            # Mode vidéo importée :
            # - barre vidéo visible ;
            # - dock caméra masqué et compressé ;
            # - on garde Image + ROI + segmentation + optokinetic + plots.
            try:
                self.importedVideo_toolbar.hide()
            except Exception:
                pass

            self._set_dock_clean_visible(self.d11, False)
            self._set_dock_clean_visible(self.d15, True)

            try:
                self.d1.raiseDock()
            except Exception:
                pass

            try:
                self.d8.raiseDock()
            except Exception:
                pass

            try:
                self.track_checkBox.setEnabled(False)
            except Exception:
                pass

            # Remplir le vide laissé par le dock caméra.
            self._apply_imported_video_layout()

            try:
                self.startImportedVideo_btn.setEnabled(True)
            except Exception:
                pass

            try:
                if self.imported_video_path is None:
                    self.importedVideoStatus_label.setText(
                        "Open a video, place ROIs/arcs, then analyze."
                    )
            except Exception:
                pass

        else:
            # Mode temps réel :
            # - barre vidéo importée masquée ;
            # - dock caméra visible ;
            # - aucun accès visuel au mode importé.
            try:
                self.importedVideo_toolbar.hide()
            except Exception:
                pass

            self._restore_live_layout()

            self._set_dock_clean_visible(self.d15, False)
            self._set_dock_clean_visible(self.d11, True)

            try:
                self.d11.raiseDock()
            except Exception:
                pass

            try:
                self.d8.raiseDock()
            except Exception:
                pass

            try:
                self.track_checkBox.setEnabled(True)
            except Exception:
                pass

    def _sync_analysis_mode_combo(self, mode):
        try:
            self.analysisMode_tabs.blockSignals(True)
            self.analysisMode_tabs.setCurrentIndex(1 if mode == "video" else 0)
            self.analysisMode_tabs.blockSignals(False)
        except Exception:
            pass

    def show_video_file_interface(self):
        """
        Compatibilité avec les appels existants.
        Passe simplement en mode Imported video.
        """
        self.set_analysis_mode("video", update_combo=True)

    def show_direct_interface(self):
        """
        Compatibilité avec les appels existants.
        Passe simplement en mode Real-time camera.
        """
        self.set_analysis_mode("live", update_combo=True)

    def choose_imported_video_file(self):
        video_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open video file",
            os.path.expanduser("~"),
            "Video files (*.avi *.mp4 *.mov *.mkv *.mpg *.mpeg);;All files (*.*)"
        )

        if not video_path:
            return

        self.imported_video_path = video_path
        self.importedVideoPath_label.setText(video_path)
        self.importedVideoStatus_label.setText("Video selected.")
        self.show_video_file_interface()

        self.load_imported_video_first_frame(video_path)

    def load_imported_video_first_frame(self, video_path, keep_crop_values=False):
        try:
            cap = cv2.VideoCapture(video_path)

            if not cap.isOpened():
                QtWidgets.QMessageBox.warning(
                    self,
                    "Video file",
                    "Impossible d'ouvrir la vidéo."
                )
                return False

            ok, frame = cap.read()

            fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

            cap.release()

            if not ok or frame is None:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Video file",
                    "Impossible de lire la première frame."
                )
                return False

            if len(frame.shape) > 2:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            if not keep_crop_values:
                self.configure_imported_video_crop_controls(width, height)

            cropped_gray = self.crop_imported_video_frame(gray)
            self.imported_video_frame = cropped_gray.copy()

            self.video.path = video_path
            self.video.width = cropped_gray.shape[1]
            self.video.height = cropped_gray.shape[0]
            self.video.fps = fps
            self.video.nbFrames = total_frames

            self.show_gray_frame_in_image_dock(cropped_gray)

            try:
                self._apply_imported_video_layout()
            except Exception:
                pass

            try:
                updater = getattr(self.video_capture_widget, "videoDisplayer_updater", None)

                if updater is not None:
                    updater.current_frame_to_display = gray.copy()
            except Exception:
                pass

            self.importedVideoStatus_label.setText(
                "Loaded: {} frames | {:.2f} fps | full {} x {} | shown {} x {}".format(
                    total_frames,
                    fps,
                    width,
                    height,
                    self.video.width,
                    self.video.height
                )
            )

            self.imported_video_total_frames = total_frames
            self.set_imported_review_limits(total_frames)

            return True

        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Video file",
                "Erreur chargement vidéo : {}".format(exc)
            )
            return False

    def get_imported_video_crop_config(self):
        """
        Retourne le crop vidéo importée en coordonnées OpenCV :
        x, y, width, height.

        Si le crop est désactivé, retourne None.
        """
        try:
            if not self.videoCropEnable_ckb.isChecked():
                return None

            raw_w = int(getattr(self, "imported_video_raw_width", 0) or 0)
            raw_h = int(getattr(self, "imported_video_raw_height", 0) or 0)

            if raw_w <= 0 or raw_h <= 0:
                return None

            x = int(self.videoCropOffsetX_slider.value())
            y = int(self.videoCropOffsetY_slider.value())
            w = int(self.videoCropFrameWidth_slider.value())
            h = int(self.videoCropFrameHeight_slider.value())

            x = max(0, min(x, raw_w - 1))
            y = max(0, min(y, raw_h - 1))
            w = max(1, min(w, raw_w - x))
            h = max(1, min(h, raw_h - y))

            return {
                "x": x,
                "y": y,
                "width": w,
                "height": h,
            }

        except Exception:
            return None

    def crop_imported_video_frame(self, gray_frame):
        """
        Applique le crop actuel à une frame grayscale.
        """
        if gray_frame is None:
            return gray_frame

        crop = self.get_imported_video_crop_config()

        if not crop:
            return gray_frame

        try:
            x = int(crop["x"])
            y = int(crop["y"])
            w = int(crop["width"])
            h = int(crop["height"])
            return gray_frame[y:y + h, x:x + w]
        except Exception:
            return gray_frame

    def update_imported_video_crop_labels(self):
        """
        Met à jour les valeurs affichées des sliders de crop.
        Les labels remplacent les anciens champs x/y/w/h.
        """
        try:
            raw_w = int(getattr(self, "imported_video_raw_width", 0) or 0)
            raw_h = int(getattr(self, "imported_video_raw_height", 0) or 0)

            x = int(self.videoCropOffsetX_slider.value())
            y = int(self.videoCropOffsetY_slider.value())

            if raw_w > 0:
                self.videoCropFrameWidth_slider.setMaximum(max(1, raw_w - x))
            if raw_h > 0:
                self.videoCropFrameHeight_slider.setMaximum(max(1, raw_h - y))

            self.videoCropFrameWidth_value.setText(str(int(self.videoCropFrameWidth_slider.value())))
            self.videoCropFrameHeight_value.setText(str(int(self.videoCropFrameHeight_slider.value())))
            self.videoCropOffsetX_value.setText(str(x))
            self.videoCropOffsetY_value.setText(str(y))

        except Exception:
            pass

    def configure_imported_video_crop_controls(self, width, height):
        """
        Initialise les contrôles de crop après ouverture d'une vidéo.
        """
        try:
            width = int(width or 0)
            height = int(height or 0)
        except Exception:
            width = 0
            height = 0

        self.imported_video_raw_width = width
        self.imported_video_raw_height = height

        max_x = max(0, width - 1)
        max_y = max(0, height - 1)

        self.videoCropOffsetX_slider.setMaximum(max_x)
        self.videoCropOffsetY_slider.setMaximum(max_y)
        self.videoCropFrameWidth_slider.setMaximum(max(1, width))
        self.videoCropFrameHeight_slider.setMaximum(max(1, height))

        self.videoCropOffsetX_slider.setValue(0)
        self.videoCropOffsetY_slider.setValue(0)
        self.videoCropFrameWidth_slider.setValue(max(1, width))
        self.videoCropFrameHeight_slider.setValue(max(1, height))
        self.update_imported_video_crop_labels()

    def apply_imported_video_crop(self):
        """
        Applique le crop à l'affichage courant.
        Les ROIs/arcs doivent être placés sur cette image croppée.
        """
        if self.imported_video_path is None:
            return

        try:
            self.load_imported_video_first_frame(self.imported_video_path, keep_crop_values=True)
            self.importedVideoStatus_label.setText(
                "Crop applied: offset x={} offset y={} frame width={} frame height={}".format(
                    self.videoCropOffsetX_slider.value(),
                    self.videoCropOffsetY_slider.value(),
                    self.videoCropFrameWidth_slider.value(),
                    self.videoCropFrameHeight_slider.value()
                )
            )
        except Exception as exc:
            print("apply_imported_video_crop error:", exc)

    def reset_imported_video_crop(self):
        """
        Remet le crop sur toute la vidéo.
        """
        if self.imported_video_raw_width > 0 and self.imported_video_raw_height > 0:
            self.videoCropOffsetX_slider.setValue(0)
            self.videoCropOffsetY_slider.setValue(0)
            self.videoCropFrameWidth_slider.setValue(self.imported_video_raw_width)
            self.videoCropFrameHeight_slider.setValue(self.imported_video_raw_height)
            self.update_imported_video_crop_labels()

        if self.imported_video_path is not None:
            self.load_imported_video_first_frame(self.imported_video_path, keep_crop_values=True)



    def _json_safe_number(self, value):
        """
        Convertit proprement les nombres Qt / NumPy en types JSON standards.
        """
        try:
            if isinstance(value, (np.integer,)):
                return int(value)
            if isinstance(value, (np.floating,)):
                return float(value)
        except Exception:
            pass

        try:
            if isinstance(value, bool):
                return bool(value)
            if isinstance(value, int):
                return int(value)
            if isinstance(value, float):
                return float(value)
        except Exception:
            pass

        return value

    def _read_named_widget_values(self, owner):
        """
        Sauvegarde les valeurs des widgets Qt exposés comme attributs d'un objet.
        Sert à garder la configuration caméra sans devoir lister chaque champ à la main.
        """
        values = {}

        if owner is None:
            return values

        try:
            items = vars(owner).items()
        except Exception:
            return values

        for name, widget in items:
            if name.startswith("_"):
                continue

            try:
                if isinstance(widget, QtWidgets.QSpinBox):
                    values[name] = {"type": "QSpinBox", "value": int(widget.value())}
                elif isinstance(widget, QtWidgets.QDoubleSpinBox):
                    values[name] = {"type": "QDoubleSpinBox", "value": float(widget.value())}
                elif isinstance(widget, QtWidgets.QSlider):
                    values[name] = {"type": "QSlider", "value": int(widget.value())}
                elif isinstance(widget, QtWidgets.QCheckBox):
                    values[name] = {"type": "QCheckBox", "checked": bool(widget.isChecked())}
                elif isinstance(widget, QtWidgets.QRadioButton):
                    values[name] = {"type": "QRadioButton", "checked": bool(widget.isChecked())}
                elif isinstance(widget, QtWidgets.QComboBox):
                    values[name] = {
                        "type": "QComboBox",
                        "index": int(widget.currentIndex()),
                        "text": str(widget.currentText()),
                    }
                elif isinstance(widget, QtWidgets.QLineEdit):
                    values[name] = {"type": "QLineEdit", "text": str(widget.text())}
            except Exception:
                pass

        return values

    def _apply_named_widget_values(self, owner, values):
        """
        Recharge les valeurs Qt sauvegardées par _read_named_widget_values().
        """
        if owner is None or not isinstance(values, dict):
            return

        for name, info in values.items():
            widget = getattr(owner, name, None)
            if widget is None or not isinstance(info, dict):
                continue

            try:
                old_block = widget.blockSignals(True)
            except Exception:
                old_block = None

            try:
                if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox, QtWidgets.QSlider)):
                    if "value" in info:
                        widget.setValue(info["value"])
                elif isinstance(widget, (QtWidgets.QCheckBox, QtWidgets.QRadioButton)):
                    if "checked" in info:
                        widget.setChecked(bool(info["checked"]))
                elif isinstance(widget, QtWidgets.QComboBox):
                    text = info.get("text", None)
                    index = int(info.get("index", -1))
                    if text is not None and widget.findText(str(text)) >= 0:
                        widget.setCurrentText(str(text))
                    elif 0 <= index < widget.count():
                        widget.setCurrentIndex(index)
                elif isinstance(widget, QtWidgets.QLineEdit):
                    widget.setText(str(info.get("text", "")))
            except Exception:
                pass

            try:
                if old_block is not None:
                    widget.blockSignals(old_block)
            except Exception:
                pass

    def _tail_reference_points_to_list(self):
        try:
            data = getattr(self.mark, "data", {})
            pos = data.get("pos", None) if isinstance(data, dict) else None

            if pos is None or len(pos) < 3:
                return None

            return [[float(p[0]), float(p[1])] for p in pos[:3]]
        except Exception:
            return None

    def _set_tail_reference_points_from_list(self, points):
        try:
            if points is None or len(points) < 3:
                return False

            pos = np.array([
                [float(points[0][0]), float(points[0][1])],
                [float(points[1][0]), float(points[1][1])],
                [float(points[2][0]), float(points[2][1])],
            ], dtype=float)

            adj = np.array([[0, 1], [0, 2]])
            symbols = ['o', 'o', 'o']
            symbolBrushes = [(255, 0, 255), (255, 0, 255), (0, 255, 0)]
            lines = np.array([
                (255, 0, 255, 255, 2),
                (0, 255, 0, 255, 2),
            ], dtype=[('red', np.ubyte), ('green', np.ubyte), ('blue', np.ubyte), ('alpha', np.ubyte), ('width', float)])
            texts = ["root", "nose", "tail"]

            self.mark.size = 8
            self.mark.setData(
                pos=pos,
                adj=adj,
                pen=lines,
                size=self.mark.size,
                symbolBrush=symbolBrushes,
                symbolPen='w',
                symbol=symbols,
                pxMode=False,
                text=texts,
            )

            try:
                nose = pos[1]
                root = pos[0]
                varM.bodyAxis_Y = float(nose[1])
                varM.bodyAngle = math.atan2(float(nose[1] - root[1]), float(nose[0] - root[0])) * 180.0 / math.pi
            except Exception:
                pass

            return True
        except Exception as exc:
            print("_set_tail_reference_points_from_list error:", exc)
            return False

    def _eye_rois_to_settings(self):
        rois = []

        try:
            for roi in self.roisEye:
                x, y = roi.pos()
                w, h = roi.size()
                rois.append({
                    "pos": [float(x), float(y)],
                    "size": [float(w), float(h)],
                })
        except Exception:
            pass

        return rois

    def _clear_eye_rois(self):
        try:
            for label in list(getattr(self, "roisEyeLabels", [])):
                try:
                    self.videoDisplay_Widget.plotView.scene().removeItem(label)
                except Exception:
                    pass

            for roi in list(getattr(self, "roisEye", [])):
                try:
                    self.videoDisplay_Widget.plotView.removeItem(roi)
                except Exception:
                    try:
                        self.videoDisplay_Widget.plotView.scene().removeItem(roi)
                    except Exception:
                        pass

            for ellipse in list(getattr(self, "roisEllipseEye", [])):
                try:
                    self.videoDisplay_Widget.plotView.removeItem(ellipse)
                except Exception:
                    try:
                        self.videoDisplay_Widget.plotView.scene().removeItem(ellipse)
                    except Exception:
                        pass

            # Quand on recharge des settings, les anciennes droites d'axe des yeux
            # ne doivent pas rester dans le plot. Elles seront recalculées juste après
            # avec la frame courante et les seuils restaurés.
            for line in list(getattr(self, "eyeAxeLines", [])):
                try:
                    self.videoDisplay_Widget.plotView.removeItem(line)
                except Exception:
                    try:
                        self.videoDisplay_Widget.plotView.scene().removeItem(line)
                    except Exception:
                        pass

            self.roisEye = []
            self.roisEllipseEye = []
            self.roisEyeLabels = []
        except Exception as exc:
            print("_clear_eye_rois error:", exc)

    def _restore_eye_rois_from_settings(self, rois):
        if not isinstance(rois, list):
            return

        try:
            self._clear_eye_rois()

            max_w = max(1, int(getattr(self.video, "width", 0) or getattr(self, "imported_video_raw_width", 1) or 1))
            max_h = max(1, int(getattr(self.video, "height", 0) or getattr(self, "imported_video_raw_height", 1) or 1))
            roi_limits = QRectF(0, 0, max_w, max_h)

            for item in rois[:2]:
                pos = item.get("pos", [0, 0]) if isinstance(item, dict) else [0, 0]
                size = item.get("size", [100, 80]) if isinstance(item, dict) else [100, 80]

                roi = Roi(
                    [float(pos[0]), float(pos[1])],
                    [float(size[0]), float(size[1])],
                    maxBounds=roi_limits,
                    centered=True,
                    pen=("b"),
                    removable=True,
                )
                roi.sigRemoveRequested.connect(self.remove_ROI)
                self.roisEye.append(roi)
                self.videoDisplay_Widget.plotView.addItem(roi)
                self.add_eye_roi_label(roi)

                ellipse = define_rois.EllipseROI_Centered_NoHandle(
                    pos=roi.pos(),
                    size=[1, 1],
                    pen=(3, 5),
                )
                self.roisEllipseEye.append(ellipse)
                self.videoDisplay_Widget.plotView.addItem(ellipse)

            self.refresh_eye_roi_labels()
        except Exception as exc:
            print("_restore_eye_rois_from_settings error:", exc)

    def _tail_arc_settings_to_dict(self):
        settings = {}

        for label in ["R", "M", "C"]:
            roi = self.get_tail_arc_rois().get(label)
            slider = self.get_tail_arc_curve_slider(label)

            threshold_slider = {
                "R": self.threshTail_slider,
                "M": getattr(self, "threshTailM_slider", self.threshTail_slider),
                "C": getattr(self, "threshTailC_slider", self.threshTail_slider),
            }.get(label, self.threshTail_slider)

            settings[label] = {
                "enabled": bool(self.is_tail_arc_enabled(label)),
                "threshold": int(threshold_slider.value()),
                "curve": int(slider.value()) if slider is not None else 40,
                "initialized": bool(roi.initialized) if roi is not None else False,
                "geometry": roi.get_parameters() if roi is not None and roi.initialized else None,
            }

        return settings

    def _apply_tail_arc_settings(self, settings):
        if not isinstance(settings, dict):
            return

        threshold_sliders = {
            "R": self.threshTail_slider,
            "M": getattr(self, "threshTailM_slider", self.threshTail_slider),
            "C": getattr(self, "threshTailC_slider", self.threshTail_slider),
        }

        for label in ["R", "M", "C"]:
            item = settings.get(label, {})
            if not isinstance(item, dict):
                item = {}

            checkbox = getattr(self, "tailArc{}_checkBox".format(label), None)
            if checkbox is not None and "enabled" in item:
                checkbox.blockSignals(True)
                checkbox.setChecked(bool(item.get("enabled")))
                checkbox.blockSignals(False)

            slider = threshold_sliders.get(label)
            if slider is not None and "threshold" in item:
                slider.blockSignals(True)
                slider.setValue(int(item.get("threshold", slider.value())))
                slider.blockSignals(False)

            curve_slider = self.get_tail_arc_curve_slider(label)
            if curve_slider is not None and "curve" in item:
                curve_slider.blockSignals(True)
                curve_slider.setValue(int(item.get("curve", curve_slider.value())))
                curve_slider.blockSignals(False)

            roi = self.get_tail_arc_rois().get(label)
            geometry = item.get("geometry")

            if roi is not None and isinstance(geometry, dict):
                try:
                    roi.center = np.array(geometry.get("center", [50.0, 150.0]), dtype=float)
                    roi.inner_radius = float(geometry.get("inner_radius", 35.0))
                    roi.outer_radius = float(geometry.get("outer_radius", 95.0))
                    roi.start_angle = float(geometry.get("start_angle", -0.75))
                    roi.end_angle = float(geometry.get("end_angle", 0.75))
                    roi.initialized = bool(item.get("initialized", True))
                    roi._dirty_version = int(geometry.get("version", getattr(roi, "_dirty_version", 0)))
                    roi._last_curve_value = float(item.get("curve", 50.0))
                    roi._save_curve_reference()
                    roi.update_graph()
                except Exception as exc:
                    print("restore arc {} error:".format(label), exc)

        self.update_tail_arc_enabled_states(update_preview=False)

        try:
            self.threshTailValue_label.setNum(int(self.threshTail_slider.value()))
            self.threshTailMValue_label.setNum(int(self.threshTailM_slider.value()))
            self.threshTailCValue_label.setNum(int(self.threshTailC_slider.value()))
            self.tailArcCurveRValue_label.setNum(int(self.tailArcCurveR_slider.value()))
            self.tailArcCurveMValue_label.setNum(int(self.tailArcCurveM_slider.value()))
            self.tailArcCurveCValue_label.setNum(int(self.tailArcCurveC_slider.value()))
        except Exception:
            pass

    def _collect_analysis_settings(self):
        """
        Rassemble les réglages des deux modes : Real-time camera et Imported video.
        Le même JSON peut donc restaurer une session live ou une session vidéo importée.
        """
        crop_config = self.get_imported_video_crop_config()

        return {
            "format": "xenopus_analysis_settings",
            "version": 2,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "analysis_mode": str(getattr(self, "analysis_mode", "live")),
            "video_state": {
                "path": getattr(self.video, "path", None),
                "width": int(getattr(self.video, "width", 0) or 0),
                "height": int(getattr(self.video, "height", 0) or 0),
                "fps": float(getattr(self.video, "fps", 0) or 0),
                "nb_frames": int(getattr(self.video, "nbFrames", 0) or 0),
                "features_file": getattr(self.video, "featuresFile", None),
                "pixel_format": getattr(self.video, "pixFormat", None),
                "grab_frame_rate": self._json_safe_number(getattr(self.video, "grabFrameRate", None)),
                "offset_x": self._json_safe_number(getattr(self.video, "offsetX", None)),
                "offset_y": self._json_safe_number(getattr(self.video, "offsetY", None)),
            },
            "imported_video": {
                "path": self.imported_video_path,
                "raw_width": int(getattr(self, "imported_video_raw_width", 0) or 0),
                "raw_height": int(getattr(self, "imported_video_raw_height", 0) or 0),
                "total_frames": int(getattr(self, "imported_video_total_frames", 0) or 0),
            },
            "imported_crop": {
                "enabled": bool(self.videoCropEnable_ckb.isChecked()),
                "config": crop_config,
                "slider_values": {
                    "frame_width": int(self.videoCropFrameWidth_slider.value()),
                    "frame_height": int(self.videoCropFrameHeight_slider.value()),
                    "offset_x": int(self.videoCropOffsetX_slider.value()),
                    "offset_y": int(self.videoCropOffsetY_slider.value()),
                },
            },
            "tail_reference_points": self._tail_reference_points_to_list(),
            "tail_arcs": self._tail_arc_settings_to_dict(),
            "eye_rois": self._eye_rois_to_settings(),
            "thresholds": {
                "eye1": int(self.threshEye1_slider.value()),
                "eye2": int(self.threshEye2_slider.value()),
                "tail_R": int(self.threshTail_slider.value()),
                "tail_M": int(getattr(self, "threshTailM_slider", self.threshTail_slider).value()),
                "tail_C": int(getattr(self, "threshTailC_slider", self.threshTail_slider).value()),
            },
            "analysis_controls": {
                "mode_index": int(self.manipType_comboBox.currentIndex()),
                "mode_text": str(self.manipType_comboBox.currentText()),
                "selection_eye": bool(self.selectEyes_radioButton.isChecked()),
                "selection_tail_root": bool(self.selectTailRoot_radioButton.isChecked()),
                "white_background": bool(self.whiteBgd_radioButton.isChecked()),
                "black_background": bool(self.blackBgd_radioButton.isChecked()),
                "preview_during_analysis": bool(self.previewDuringAnalysis_ckb.isChecked()),
            },
            "output": {
                "folder": self.result_save_dir,
                "stage": str(self.stageLineEdit.text()),
                "file_number": int(self.fileNumber_spinBox.value()),
            },
            "camera_ui": self._read_named_widget_values(getattr(self, "video_capture_widget", None)),
            "main_ui": self._read_named_widget_values(self),
        }

    def save_analysis_settings(self):
        """
        Enregistre tous les réglages du mode courant.
        Fonctionne en Real-time camera et en Imported video.
        """
        try:
            base_dir = self.result_save_dir or os.path.dirname(self.imported_video_path or "") or os.path.dirname(getattr(self.video, "path", "") or "") or os.path.expanduser("~")
            mode = str(getattr(self, "analysis_mode", "live"))
            default_name = "xenopus_{}_settings.json".format("live" if mode == "live" else "imported")
            default_path = os.path.join(base_dir, default_name)

            file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                "Save Xenopus settings",
                default_path,
                "Xenopus settings (*.json);;JSON file (*.json)"
            )

            if not file_path:
                return

            if not file_path.lower().endswith(".json"):
                file_path += ".json"

            settings = self._collect_analysis_settings()

            with open(file_path, "w", encoding="utf-8") as settings_file:
                json.dump(settings, settings_file, indent=2, ensure_ascii=False)

            self._show_settings_status("Settings saved: {}".format(file_path))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Save settings",
                "Impossible d'enregistrer les settings : {}".format(exc)
            )

    def _show_settings_status(self, message):
        try:
            if getattr(self, "analysis_mode", "live") == "video":
                self.importedVideoStatus_label.setText(message)
            else:
                print(message)
        except Exception:
            print(message)

    def refresh_eye_axes_from_current_frame(self):
        """
        Recalcule les ellipses et les droites des yeux après un Load settings.

        Les settings sauvegardent les ROIs et les seuils. La droite affichée sur
        chaque œil est un résultat calculé depuis la frame courante, donc elle
        doit être reconstruite après la restauration des ROIs.
        """
        try:
            if self.track_checkBox.isChecked():
                return

            frame = self.get_current_analysis_frame()

            if frame is None:
                return

            if len(getattr(self, "roisEye", [])) == 0:
                return

            # update_eyes_overlay utilise l'axe du corps pour l'angle corrigé.
            # On force donc une mise à jour depuis root/nose si ces points existent.
            try:
                if len(self.mark.data) > 0:
                    root = self.mark.data['pos'][0]
                    nose = self.mark.data['pos'][1]
                    varM.bodyAxis_Y = float(nose[1])
                    varM.bodyAngle = math.atan2(
                        float(nose[1] - root[1]),
                        float(nose[0] - root[0])
                    ) * 180.0 / math.pi
            except Exception:
                pass

            threshold_values = [
                int(self.threshEye1_slider.value()),
                int(self.threshEye2_slider.value()),
            ]

            max_count = min(
                len(getattr(self, "roisEye", [])),
                len(getattr(self, "eyeAxeLines", [])),
                2
            )

            for roi_index in range(max_count):
                try:
                    self.update_eyes_overlay(threshold_values[roi_index], roi_index)
                except Exception as exc:
                    print("refresh eye axis {} error:".format(roi_index + 1), exc)

        except Exception as exc:
            print("refresh_eye_axes_from_current_frame error:", exc)

    def _apply_imported_video_crop_settings(self, crop_settings):
        if not isinstance(crop_settings, dict):
            return

        try:
            self.videoCropEnable_ckb.setChecked(bool(crop_settings.get("enabled", True)))

            slider_values = crop_settings.get("slider_values", {})
            config = crop_settings.get("config") or {}

            frame_width = slider_values.get("frame_width", config.get("width", self.videoCropFrameWidth_slider.value()))
            frame_height = slider_values.get("frame_height", config.get("height", self.videoCropFrameHeight_slider.value()))
            offset_x = slider_values.get("offset_x", config.get("x", self.videoCropOffsetX_slider.value()))
            offset_y = slider_values.get("offset_y", config.get("y", self.videoCropOffsetY_slider.value()))

            self.videoCropOffsetX_slider.setValue(int(offset_x))
            self.videoCropOffsetY_slider.setValue(int(offset_y))
            self.videoCropFrameWidth_slider.setValue(int(frame_width))
            self.videoCropFrameHeight_slider.setValue(int(frame_height))
            self.update_imported_video_crop_labels()
        except Exception as exc:
            print("_apply_imported_video_crop_settings error:", exc)

    def _apply_analysis_settings(self, settings):
        if not isinstance(settings, dict):
            raise ValueError("Invalid settings file.")

        fmt = settings.get("format")
        if fmt == "xenopus_imported_video_settings":
            # Compatibilité avec les anciens JSON créés avant le support du mode live.
            settings = dict(settings)
            settings["format"] = "xenopus_analysis_settings"
            settings["analysis_mode"] = "video"
            settings["imported_video"] = settings.get("video", {})
        elif fmt != "xenopus_analysis_settings":
            raise ValueError("This JSON file is not a Xenopus settings file.")

        requested_mode = settings.get("analysis_mode", getattr(self, "analysis_mode", "live"))
        if requested_mode not in ("live", "video"):
            requested_mode = getattr(self, "analysis_mode", "live")

        # On restaure d'abord les dimensions connues, utiles pour remettre les ROI même si aucune caméra/vidéo n'est ouverte.
        video_state = settings.get("video_state", {})
        try:
            self.video.width = int(video_state.get("width", getattr(self.video, "width", 0) or 0) or 0)
            self.video.height = int(video_state.get("height", getattr(self.video, "height", 0) or 0) or 0)
            self.video.fps = float(video_state.get("fps", getattr(self.video, "fps", 0) or 0) or 0)
            self.video.nbFrames = int(video_state.get("nb_frames", getattr(self.video, "nbFrames", 0) or 0) or 0)
            if video_state.get("path"):
                self.video.path = video_state.get("path")
            if video_state.get("features_file"):
                self.video.featuresFile = video_state.get("features_file")
        except Exception:
            pass

        # Ne pas relancer set_analysis_mode() si on est déjà dans le bon mode.
        # Sinon le DockArea recalcule les splitters et peut faire réapparaître
        # une barre/séparation parasite dans le panneau video Capture après Load settings.
        current_mode = getattr(self, "analysis_mode", "live")
        if requested_mode != current_mode:
            try:
                self.set_analysis_mode(requested_mode, update_combo=True)
            except Exception:
                pass
        else:
            try:
                self._sync_analysis_mode_combo(requested_mode)
            except Exception:
                pass

        analysis_controls = settings.get("analysis_controls", {})
        output_settings = settings.get("output", {})

        try:
            if "mode_index" in analysis_controls:
                index = int(analysis_controls.get("mode_index", 0))
                if 0 <= index < self.manipType_comboBox.count():
                    self.manipType_comboBox.setCurrentIndex(index)
            self.selectEyes_radioButton.setChecked(bool(analysis_controls.get("selection_eye", self.selectEyes_radioButton.isChecked())))
            self.selectTailRoot_radioButton.setChecked(bool(analysis_controls.get("selection_tail_root", self.selectTailRoot_radioButton.isChecked())))
            self.whiteBgd_radioButton.setChecked(bool(analysis_controls.get("white_background", self.whiteBgd_radioButton.isChecked())))
            self.blackBgd_radioButton.setChecked(bool(analysis_controls.get("black_background", self.blackBgd_radioButton.isChecked())))
            self.previewDuringAnalysis_ckb.setChecked(bool(analysis_controls.get("preview_during_analysis", self.previewDuringAnalysis_ckb.isChecked())))
        except Exception:
            pass

        try:
            self.result_save_dir = output_settings.get("folder", self.result_save_dir)
            self.outputFolder_label.setText(self.result_save_dir if self.result_save_dir else "No folder selected")
            self.stageLineEdit.setText(str(output_settings.get("stage", self.stageLineEdit.text())))
            self.fileNumber_spinBox.setValue(int(output_settings.get("file_number", self.fileNumber_spinBox.value())))
            self.update_next_csv_preview()
        except Exception:
            pass

        # Config caméra / live : sauvegardée comme valeurs UI génériques.
        self._apply_named_widget_values(getattr(self, "video_capture_widget", None), settings.get("camera_ui", {}))

        # Vidéo importée : uniquement si le JSON vient d'une session Imported video ou contient un fichier vidéo.
        imported_settings = settings.get("imported_video", {}) or settings.get("video", {})
        video_path = imported_settings.get("path")
        loaded_video = False

        if requested_mode == "video" and video_path:
            self.imported_video_path = video_path
            self.importedVideoPath_label.setText(video_path)

            if os.path.exists(video_path):
                loaded_video = bool(self.load_imported_video_first_frame(video_path, keep_crop_values=False))
            else:
                self.importedVideoStatus_label.setText("Settings loaded, but video file not found: {}".format(video_path))

        if requested_mode == "video" and not loaded_video:
            try:
                raw_w = int(imported_settings.get("raw_width", 0) or 0)
                raw_h = int(imported_settings.get("raw_height", 0) or 0)
                if raw_w > 0 and raw_h > 0:
                    self.configure_imported_video_crop_controls(raw_w, raw_h)
            except Exception:
                pass

        self._apply_imported_video_crop_settings(settings.get("imported_crop", {}))

        if requested_mode == "video" and loaded_video:
            self.load_imported_video_first_frame(video_path, keep_crop_values=True)

        thresholds = settings.get("thresholds", {})
        try:
            self.threshEye1_slider.setValue(int(thresholds.get("eye1", self.threshEye1_slider.value())))
            self.threshEye2_slider.setValue(int(thresholds.get("eye2", self.threshEye2_slider.value())))
        except Exception:
            pass

        self._restore_eye_rois_from_settings(settings.get("eye_rois", []))
        self._set_tail_reference_points_from_list(settings.get("tail_reference_points"))
        self._apply_tail_arc_settings(settings.get("tail_arcs", {}))

        try:
            if self.selectTailRoot_radioButton.isChecked():
                self.update_tail_segment_thresh(self.threshTail_slider.value())
        except Exception:
            pass

        try:
            self.update_tail_segment_overlay()
        except Exception:
            pass

        # Après un Load settings, les ROIs des yeux sont restaurées mais les
        # droites/ellipses doivent être recalculées depuis la frame courante.
        self.refresh_eye_axes_from_current_frame()
        try:
            QTimer.singleShot(0, self.refresh_eye_axes_from_current_frame)
            QTimer.singleShot(120, self.refresh_eye_axes_from_current_frame)
        except Exception:
            pass

        # En mode Real-time, Load settings ne doit pas modifier la géométrie
        # des docks. On garantit juste que le dock Imported video reste masqué,
        # sans appeler _restore_live_layout() ni resizeDocks().
        if requested_mode == "live":
            try:
                self._set_dock_clean_visible(self.d15, False)
                self._set_dock_clean_visible(self.d11, True)
            except Exception:
                pass

    def load_analysis_settings(self):
        """
        Recharge un fichier JSON de réglages. Fonctionne pour live et imported.
        """
        try:
            default_dir = self.result_save_dir or os.path.dirname(self.imported_video_path or "") or os.path.dirname(getattr(self.video, "path", "") or "") or os.path.expanduser("~")

            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Load Xenopus settings",
                default_dir,
                "Xenopus settings (*.json);;JSON file (*.json)"
            )

            if not file_path:
                return

            with open(file_path, "r", encoding="utf-8") as settings_file:
                settings = json.load(settings_file)

            self._apply_analysis_settings(settings)
            self._show_settings_status("Settings loaded: {}".format(file_path))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Load settings",
                "Impossible de charger les settings : {}".format(exc)
            )

    # Compatibilité avec les noms utilisés par les versions précédentes du patch Imported video.
    def save_imported_video_settings(self):
        self.save_analysis_settings()

    def load_imported_video_settings(self):
        self.load_analysis_settings()

    def set_imported_review_limits(self, total_frames):
        """
        Configure le slider et le spinbox de navigation vidéo.
        """
        try:
            total_frames = int(total_frames or 0)
        except Exception:
            total_frames = 0

        self.imported_video_total_frames = total_frames
        max_frame = max(0, total_frames - 1)

        self.imported_review_ignore_signals = True
        try:
            self.reviewFrameSlider.setRange(0, max_frame)
            self.reviewFrameSpin.setRange(0, max_frame)
            self.reviewFrameSlider.setValue(0)
            self.reviewFrameSpin.setValue(0)

            enabled = total_frames > 0
            self.reviewFrameSlider.setEnabled(enabled)
            self.reviewFrameSpin.setEnabled(enabled)
            self.reviewFramePrev_btn.setEnabled(enabled)
            self.reviewFrameNext_btn.setEnabled(enabled)
        finally:
            self.imported_review_ignore_signals = False

    def on_review_frame_slider_changed(self, value):
        if getattr(self, "imported_review_ignore_signals", False):
            return

        self.show_imported_video_review_frame(int(value), update_controls=False)

    def on_review_frame_spin_changed(self, value):
        if getattr(self, "imported_review_ignore_signals", False):
            return

        self.show_imported_video_review_frame(int(value), update_controls=False)

    def review_imported_video_previous_frame(self):
        try:
            frame_id = max(0, int(self.reviewFrameSpin.value()) - 1)
            self.show_imported_video_review_frame(frame_id)
        except Exception:
            pass

    def review_imported_video_next_frame(self):
        try:
            max_frame = max(0, int(self.reviewFrameSlider.maximum()))
            frame_id = min(max_frame, int(self.reviewFrameSpin.value()) + 1)
            self.show_imported_video_review_frame(frame_id)
        except Exception:
            pass

    def show_imported_video_review_frame(self, frame_id, update_controls=True):
        """
        Affiche une frame de la vidéo importée et applique les overlays de tracking
        si cette frame a déjà été analysée.

        Permet de naviguer frame par frame après l'analyse.
        """
        if self.imported_video_path is None:
            return

        try:
            frame_id = int(frame_id)
        except Exception:
            frame_id = 0

        max_frame = max(0, int(getattr(self, "imported_video_total_frames", 0) or 0) - 1)
        frame_id = max(0, min(max_frame, frame_id))

        try:
            cap = cv2.VideoCapture(self.imported_video_path)

            if not cap.isOpened():
                return

            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
            ok, frame = cap.read()
            cap.release()

            if not ok or frame is None:
                return

            if len(frame.shape) > 2:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            cropped_gray = self.crop_imported_video_frame(gray)

            self.imported_video_frame = cropped_gray.copy()
            self.show_gray_frame_in_image_dock(cropped_gray)

            if update_controls:
                self.imported_review_ignore_signals = True
                try:
                    self.reviewFrameSlider.setValue(frame_id)
                    self.reviewFrameSpin.setValue(frame_id)
                finally:
                    self.imported_review_ignore_signals = False
            else:
                self.imported_review_ignore_signals = True
                try:
                    self.reviewFrameSlider.setValue(frame_id)
                    self.reviewFrameSpin.setValue(frame_id)
                finally:
                    self.imported_review_ignore_signals = False

            result = None

            try:
                if self.controller is not None:
                    result = self.controller.get_video_file_result(frame_id)
            except Exception:
                result = None

            if result is not None:
                try:
                    self.controller.apply_result_overlay(result, move_eye_roi=False)
                    self.importedVideoStatus_label.setText(
                        "Review frame {} / {} — tracking overlay displayed".format(
                            frame_id,
                            max_frame
                        )
                    )
                except Exception as exc:
                    print("review overlay error:", exc)
            else:
                self.importedVideoStatus_label.setText(
                    "Review frame {} / {} — no tracking result yet".format(
                        frame_id,
                        max_frame
                    )
                )

        except Exception as exc:
            print("show_imported_video_review_frame error:", exc)

    def maybe_auto_preview_imported_video_frame(self, frame_id):
        """
        Pendant l'analyse, affiche périodiquement la frame en cours avec overlays.
        On ne le fait pas à chaque frame pour ne pas ralentir l'analyse.
        """
        if not self.previewDuringAnalysis_ckb.isChecked():
            return

        try:
            frame_id = int(frame_id)
        except Exception:
            return

        if frame_id < 0:
            return

        if self.imported_review_last_auto_frame >= 0:
            if frame_id - self.imported_review_last_auto_frame < 25:
                return

        self.imported_review_last_auto_frame = frame_id
        self.show_imported_video_review_frame(frame_id)


    def start_imported_video_analysis(self):
        if getattr(self, "analysis_mode", "live") != "video":
            self.set_analysis_mode("video", update_combo=True)

            if getattr(self, "analysis_mode", "live") != "video":
                return

        if self.controller is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Controller",
                "AppController is not initialized."
            )
            return

        if self.imported_video_path is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Video file",
                "Choose a video file before starting analysis."
            )
            return

        ok = self.check_analysis_parameters()

        if not ok:
            return

        if not self.validate_result_settings():
            return

        self.reset_Plot()
        self.reset_Buffer()
        self.clear_tail_arc_tracking_markers()

        try:
            self.controller.start_video_file_analysis(
                self.imported_video_path,
                video_crop=self.get_imported_video_crop_config()
            )
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self,
                "Video analysis",
                str(exc)
            )
            return

        self.importedVideoProgress.setValue(0)
        self.imported_review_last_auto_frame = -1
        self.importedVideoStatus_label.setText("Imported video analysis running...")
        self.startImportedVideo_btn.setEnabled(False)
        self.stopImportedVideo_btn.setEnabled(True)
        self.track_checkBox.setEnabled(False)
        self.video_file_status_timer.start()
        self.show_video_file_interface()

    def stop_imported_video_analysis(self):
        try:
            if self.controller is not None:
                self.controller.stop_video_file_analysis()
        except Exception:
            pass

        self.importedVideoStatus_label.setText("Stopping imported video analysis...")

    def update_imported_video_status(self):
        if self.controller is None:
            return

        status = self.controller.get_video_file_analysis_status()

        progress = int(round(float(status.get("progress", 0.0))))
        progress = max(0, min(100, progress))
        self.importedVideoProgress.setValue(progress)

        frame_id = int(status.get("frame_id", 0) or 0)
        total_frames = int(status.get("total_frames", 0) or 0)
        rows_written = int(status.get("rows_written", 0) or 0)

        if status.get("running", False):
            self.importedVideoStatus_label.setText(
                "Running: frame {} / {} | CSV rows {}".format(
                    frame_id,
                    total_frames,
                    rows_written
                )
            )

            preview_frame_id = int(status.get("preview_frame_id", frame_id - 1) or 0)
            self.maybe_auto_preview_imported_video_frame(preview_frame_id)
            return

        if status.get("error", ""):
            self.video_file_status_timer.stop()
            self.startImportedVideo_btn.setEnabled(True)
            self.stopImportedVideo_btn.setEnabled(False)
            self.track_checkBox.setEnabled(True)
            self.importedVideoStatus_label.setText(
                "Error: {}".format(status.get("error", ""))
            )
            return

        if status.get("done", False):
            self.video_file_status_timer.stop()
            self.importedVideoProgress.setValue(100)
            self.startImportedVideo_btn.setEnabled(True)
            self.stopImportedVideo_btn.setEnabled(False)
            self.track_checkBox.setEnabled(True)
            self.importedVideoStatus_label.setText(
                "Done: {} frames analyzed | CSV rows {}".format(
                    frame_id,
                    rows_written
                )
            )
            self.updatePlot_Full()
            self.update_next_csv_preview()

            # Afficher automatiquement la dernière frame analysée avec ses points.
            if frame_id > 0:
                self.show_imported_video_review_frame(frame_id - 1)

            return

        if status.get("stopped", False):
            self.video_file_status_timer.stop()
            self.startImportedVideo_btn.setEnabled(True)
            self.stopImportedVideo_btn.setEnabled(False)
            self.track_checkBox.setEnabled(True)
            self.importedVideoStatus_label.setText(
                "Stopped: {} frames analyzed | CSV rows {}".format(
                    frame_id,
                    rows_written
                )
            )
            self.updatePlot_Full()
            self.update_next_csv_preview()

            if frame_id > 0:
                self.show_imported_video_review_frame(frame_id - 1)

            return


    def choose_result_folder(self):
        folder = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Choose output folder",
            os.path.expanduser("~")
        )

        if folder:
            self.result_save_dir = folder
            self.outputFolder_label.setText(folder)
            self.update_next_csv_preview()


    def get_result_stage(self):
        stage = self.stageLineEdit.text().strip()

        if stage.lower().startswith("st"):
            stage = stage[2:]

        return stage

    def get_result_file_number(self):
        number = self.fileNumber_spinBox.value()

        if number == 0:
            return None

        return number

    def update_next_csv_preview(self):
        try:
            if self.result_save_dir is None:
                self.nextCsv_label.setText("Next CSV: -")
                return

            stage = self.get_result_stage()

            if stage == "":
                self.nextCsv_label.setText("Next CSV: missing stage")
                return

            if self.controller is None:
                self.nextCsv_label.setText("Next CSV: -")
                return

            filename = self.controller.preview_next_result_filename()
            self.nextCsv_label.setText("Next CSV: " + filename)

        except Exception:
            self.nextCsv_label.setText("Next CSV: -")

    def validate_result_settings(self):
        if self.result_save_dir is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Output folder",
                "Choose an output folder before starting tracking."
            )
            return False

        if self.get_result_stage() == "":
            QtWidgets.QMessageBox.warning(
                self,
                "Stage",
                "Define the Xenopus stage before starting tracking."
            )
            return False

        try:
            if self.controller is not None:
                result_path = self.controller.preview_next_result_file_path()

                if os.path.exists(result_path):
                    reply = QtWidgets.QMessageBox.warning(
                        self,
                        "Existing CSV file",
                        "This file already exists and will be overwritten:\n\n"
                        + result_path
                        + "\n\nContinue ?",
                        QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                        QtWidgets.QMessageBox.No
                    )

                    if reply != QtWidgets.QMessageBox.Yes:
                        return False

        except Exception as exc:
            print("CSV overwrite check error:", exc)

        return True

    def clear_old_acquisition_buffers(self):
        """
        Vide les anciennes frames encore présentes dans l'ancien thread d'acquisition.

        Important :
        quand le pipeline démarre avant la nouvelle acquisition, il ne doit pas lire
        les frames restantes de l'acquisition précédente.
        """
        try:
            acquisition_thread = getattr(self.video_capture_widget, "acquisition_thread", None)

            if acquisition_thread is None:
                return

            frame_buffer = getattr(acquisition_thread, "frame_buffer", None)

            if frame_buffer is not None:
                analysis_lock = getattr(frame_buffer, "analysis_lock", None)
                analysis_frames = getattr(frame_buffer, "analysis_frames", None)

                if analysis_lock is not None and analysis_frames is not None:
                    with analysis_lock:
                        analysis_frames.clear()

                display_lock = getattr(frame_buffer, "display_lock", None)
                display_frames = getattr(frame_buffer, "display_frames", None)

                if display_lock is not None and display_frames is not None:
                    with display_lock:
                        display_frames.clear()

            display_queue = getattr(acquisition_thread, "display_queue", None)

            if display_queue is not None:
                while True:
                    try:
                        display_queue.get_nowait()
                    except Exception:
                        break

            # Très important : l'AcquisitionWorker ne doit plus pointer vers l'ancien thread.
            self.video_capture_widget.acquisition_thread = None

            print("Old acquisition buffers cleared")

        except Exception as exc:
            print("Erreur clear_old_acquisition_buffers:", exc)

    @pyqtSlot()
    def toggle_tracking(self):
        if self.controller is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Controller",
                "AppController is not initialized in the main."
            )
            self.track_checkBox.setChecked(False)
            return

        if self.track_checkBox.isChecked():
            if getattr(self, "analysis_mode", "live") != "live":
                QtWidgets.QMessageBox.warning(
                    self,
                    "Analysis mode",
                    "Switch to Real-time camera mode before starting live tracking."
                )
                self.track_checkBox.setChecked(False)
                return

            ok = self.check_analysis_parameters()

            if not ok:
                self.track_checkBox.setChecked(False)
                return

            if not self.validate_result_settings():
                self.track_checkBox.setChecked(False)
                return

            self.reset_Plot()
            self.reset_Buffer()

            try:
                if hasattr(self.video_capture_widget, "stop_acquisition"):
                    self.video_capture_widget.stop_acquisition()
                    # Petite pause de sécurité uniquement pour laisser l'ancien thread se fermer.
                    # On ne garde plus 0.1 s pour éviter de perdre les premières frames.
                    time.sleep(0.02)

                # On supprime les frames restantes de l'ancienne acquisition.
                # Sans ça, le CSV peut commencer par des frame_id de l'ancien run
                # puis repartir à 0 quand la nouvelle acquisition démarre.
                self.clear_old_acquisition_buffers()

            except Exception as exc:
                print("Erreur stop acquisition:", exc)
                self.track_checkBox.setChecked(False)
                return

            try:
                # Le pipeline tracking/CSV démarre avant l'acquisition.
                # Comme ça, dès que la première frame arrive, elle peut être récupérée.
                self.controller.start_tracking()
            except Exception as exc:
                print("Erreur start pipeline:", exc)
                self.track_checkBox.setChecked(False)
                return

            try:
                if hasattr(self.video_capture_widget, "start_acquisition"):
                    self.video_capture_widget.start_acquisition()
            except Exception as exc:
                print("Erreur start acquisition:", exc)
                try:
                    self.controller.stop_tracking()
                except Exception:
                    pass
                self.track_checkBox.setChecked(False)
                return

            try:
                if self.video_capture_widget.trigger_checkBox.isChecked():
                    trigger()
            except Exception:
                pass

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
                self.clear_old_acquisition_buffers()
            except Exception as exc:
                print("Erreur stop acquisition:", exc)

            self.track_checkBox.setText("Track")
            self.updatePlot_Full()
            self.update_next_csv_preview()
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

        frame=self.get_current_analysis_frame()

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

    def update_tail_arc_curve(self, value):
        # Compatibilité : agit sur R si l'ancien nom est appelé.
        self.update_tail_arc_curve_R(value)

    def _update_tail_arc_curve_for_label(self, label, value):
        if not self.is_tail_arc_enabled(label):
            return

        roi = self.get_tail_arc_rois().get(label)

        if roi is not None and roi.initialized:
            roi.set_curve_value(value)

        if not self.track_checkBox.isChecked() and self.selectTailRoot_radioButton.isChecked()==True:
            self.update_tail_segment_thresh(self.threshTail_slider.value())

    def update_tail_arc_curve_R(self, value):
        self._update_tail_arc_curve_for_label("R", value)

    def update_tail_arc_curve_M(self, value):
        self._update_tail_arc_curve_for_label("M", value)

    def update_tail_arc_curve_C(self, value):
        self._update_tail_arc_curve_for_label("C", value)

    def update_tail_segment_thresh(self,thresh_value):
        if not self.track_checkBox.isChecked() and self.selectTailRoot_radioButton.isChecked()==True:

            if len(self.mark.data) == 0:
                return

            frame = self.get_current_analysis_frame()

            if frame is None:
                return

            if not self._initialize_tail_arcs_if_needed():
                return

            thresholds = self.get_tail_arc_thresholds()
            last_valid_tail = None

            # Preview des trois arcs : chaque arc affiche son propre point
            # et sa propre droite root -> point.
            for label in ["R", "M", "C"]:
                roi = self.get_tail_arc_rois().get(label)

                if not self.is_tail_arc_enabled(label):
                    self.set_tail_arc_tracking_marker(label, None)
                    continue

                if roi is None or not roi.initialized:
                    self.set_tail_arc_tracking_marker(label, None)
                    continue

                iframe, tail_pos, tail_angle = tail_Track_arc_fast(
                    0,
                    frame,
                    thresholds.get(label, self.threshTail_slider.value()),
                    roi.get_parameters(),
                    append_to_lists=False
                )

                if tail_pos[0] != 0 or tail_pos[1] != 0:
                    self.set_tail_arc_tracking_marker(label, tail_pos)
                    last_valid_tail = tail_pos
                else:
                    self.set_tail_arc_tracking_marker(label, None)

            # On garde le point tail historique pour compatibilité, mais
            # les trois points R/M/C sont affichés séparément.
            if last_valid_tail is not None:
                self.mark.data['pos'][2] = [last_valid_tail[0], last_valid_tail[1]]
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

                varM.bodyAxis_Y=noseY

                xv= noseX-rootX
                yv= noseY-rootY

                varM.bodyAngle = math.atan2(yv, xv)* 180 / math.pi
                print("angle de l'axe du corps : ",varM.bodyAngle)

                self._initialize_tail_arcs_if_needed()

                for roi in self.get_tail_arc_rois().values():
                    if roi.initialized:
                        roi.update_graph()

                self.update_tail_arc_enabled_states(update_preview=False)

                frame = self.get_current_analysis_frame()

                if frame is not None:
                    self.update_tail_segment_thresh(self.threshTail_slider.value())

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
            # Ancien Video player supprimé de l'interface.
            # On garde ce bloc uniquement pour éviter une erreur si un vieux signal l'appelle.
            if self.videoPlayer_Widget is not None:
                self.videoPlayer_Widget.playVideo_btn.setEnabled(True)
                self.videoPlayer_Widget.playVideo_btn.setChecked(False)
                self.videoPlayer_Widget.timeLine_slider.setEnabled(True)
                self.videoPlayer_Widget.stepFwdVideo_btn.setEnabled(True)
                self.videoPlayer_Widget.stepBwdVideo_btn.setEnabled(True)
            return

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

        for list_name in [
            "tailAngleListR", "tailAngleListM", "tailAngleListC",
            "tailPosListR", "tailPosListM", "tailPosListC",
        ]:
            if hasattr(self, list_name):
                getattr(self, list_name)[:]=[]

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

            first_tail_plot = True
            for values, pen in [
                (getattr(self, "tailAngleListR", []), self.penTailR),
                (getattr(self, "tailAngleListM", []), self.penTailM),
                (getattr(self, "tailAngleListC", []), self.penTailC),
            ]:
                dataArray3 = np.asarray(values[currentIdx-200: currentIdx])

                if dataArray3.size > 0:
                    self.w5.plot(dataArray3,pen=pen,clear=first_tail_plot)
                    first_tail_plot = False

            if first_tail_plot and len(self.tailAngleList)!=0:
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

                first_tail_plot = True
                for values, pen in [
                    (getattr(self, "tailAngleListR", []), self.penTailR),
                    (getattr(self, "tailAngleListM", []), self.penTailM),
                    (getattr(self, "tailAngleListC", []), self.penTailC),
                ]:
                    dataArray3 = np.asarray(values)

                    if dataArray3.size > 0:
                        self.w5.plot(dataArray3,pen=pen,clear=first_tail_plot)
                        first_tail_plot = False

                if first_tail_plot and len(self.tailAngleList)!=0:
                    dataArray3 = np.asarray(self.tailAngleList)

                    self.w5.plot(dataArray3,pen=self.penGreen,clear=True)

    def closeEvent(self, event):

        reply = QtWidgets.QMessageBox.question(
            self,
            'User confirm',
            'Have you saved track and optokinetic results ?',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )
        if reply == QtWidgets.QMessageBox.Yes:
            try:
                if self.controller is not None:
                    self.controller.stop_video_file_analysis()
            except Exception:
                pass
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
