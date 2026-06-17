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


def tail_Track_arc_fast(iframe, analysimg, thresh_tail, arc_roi):
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
    def __init__(self, plot_view):
        self.plot_view = plot_view

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
        self.fill_item.setPen(pg.mkPen(color=(40, 140, 255), width=1))
        self.fill_item.setBrush(QtgGui.QBrush(QtgGui.QColor(40, 140, 255, 45)))

        pen = pg.mkPen(color=(40, 140, 255), width=2)
        side_pen = pg.mkPen(color=(40, 140, 255), width=1)

        self.inner_curve = pg.PlotDataItem(x=[], y=[], pen=pen)
        self.outer_curve = pg.PlotDataItem(x=[], y=[], pen=pen)
        self.side_curve_1 = pg.PlotDataItem(x=[], y=[], pen=side_pen)
        self.side_curve_2 = pg.PlotDataItem(x=[], y=[], pen=side_pen)
        self.handles = TailArcHandleGraph(on_change=self._handle_moved)

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
        self.set_visible(False)

    def set_visible(self, visible):
        for item in [
            self.fill_item,
            self.inner_curve,
            self.outer_curve,
            self.side_curve_1,
            self.side_curve_2,
            self.handles,
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

        root_gap = max(170.0, img_h * 0.38)
        arc_width = max(42.0, img_h * 0.085)

        self.inner_radius = root_gap
        self.outer_radius = root_gap + arc_width

        # Ouverture un peu plus grande pour une zone plus haute.
        self.start_angle = angle_mid - math.radians(60.0)
        self.end_angle = angle_mid + math.radians(60.0)

        self.initialized = True
        self._dirty_version += 1

        self._save_curve_reference()
        self._last_curve_value = 50.0
        self.set_visible(True)
        self.update_graph()

        try:
            ui.tailArcCurve_slider.blockSignals(True)
            ui.tailArcCurve_slider.setValue(40)
            ui.tailArcCurve_slider.blockSignals(False)
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
                    ui.tailArcCurve_slider.blockSignals(True)
                    ui.tailArcCurve_slider.setValue(50)
                    ui.tailArcCurve_slider.blockSignals(False)
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
                    ui.tailArcCurve_slider.blockSignals(True)
                    ui.tailArcCurve_slider.setValue(50)
                    ui.tailArcCurve_slider.blockSignals(False)
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
        self.videoPlayer_Widget=video_player.UIVideoPlayer(self.videoDisplay_Widget,self.video,tracking)

        self.video_capture_widget=video_capture.UIVideoCapture(self.videoDisplay_Widget,self.video,self.display_updater)

        self.tracking = False
        self.analysis_thread = None
        self.result_save_dir = None
        self.controller = None

        self.plot_timer = QTimer()
        self.plot_timer.setInterval(300)
        self.plot_timer.timeout.connect(self.update_plot)

        self.roisEye = []
        self.roisEllipseEye=[]
        self.roisEyeLabels=[]
        self.tailAngleList=[]
        self.tailPosList=[]

        self.mark = graphMark()
        self.listTailSegments=[]

        self.eyeAxeLine1 = pg.InfiniteLine(movable=False)
        self.eyeAxeLine2 = pg.InfiniteLine(movable=False)
        self.eyeAxeLines=[self.eyeAxeLine1,self.eyeAxeLine2]

        self.initUI()

    def initUI(self):

        self.setWindowIcon(QtgGui.QIcon(os.path.join('Imagys_blue', 'logoAnimotion-square-112.png')))

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
        self.regionlr.setVisible(False)

        self.tailArcROI = TailArcROI(self.videoDisplay_Widget.plotView)

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

        tailArcCurve_label = QtWidgets.QLabel(splitter_ThreshTail)
        tailArcCurve_label.setText("curve")
        self.tailArcCurve_slider = QtWidgets.QSlider(splitter_ThreshTail)
        self.tailArcCurve_slider.setMinimum(0)
        self.tailArcCurve_slider.setMaximum(100)
        self.tailArcCurve_slider.setPageStep(5)
        self.tailArcCurve_slider.setOrientation(Qt.Horizontal)
        tailArcCurveValue_label = QtWidgets.QLabel(splitter_ThreshTail)
        tailArcCurveValue_label.setText("50")
        tailArcCurveValue_label.setAlignment(Qt.AlignCenter)
        self.tailArcCurve_slider.valueChanged.connect(tailArcCurveValue_label.setNum)
        self.tailArcCurve_slider.valueChanged.connect(self.update_tail_arc_curve)

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
        self.tailArcCurve_slider.setValue(40)
        self.threshEye1_slider.setValue(60)
        self.threshEye2_slider.setValue(60)

        resetPlots_btn = QtWidgets.QPushButton('Reset plots')
        resetPlots_btn.clicked.connect(self.reset_Plot)
        resetPlots_btn.setMinimumHeight(28)

        resetBuffer_btn = QtWidgets.QPushButton('Reset buffer')
        resetBuffer_btn.clicked.connect(self.reset_Buffer)
        resetBuffer_btn.setMinimumHeight(28)

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
            "0 creates file ..._000.csv\n"
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
        selectionLayout.addWidget(self.selectLimbs_radioButton)
        selectionLayout.addWidget(self.selectExclusion_radioButton)

        bgGroup = QtWidgets.QGroupBox("Background")
        bgLayout = QtWidgets.QHBoxLayout(bgGroup)
        bgLayout.setContentsMargins(8, 8, 8, 8)
        bgLayout.setSpacing(18)
        bgLayout.addWidget(self.whiteBgd_radioButton)
        bgLayout.addWidget(self.blackBgd_radioButton)
        bgLayout.addStretch(1)

        eyesGroup = QtWidgets.QGroupBox("Eyes")
        eyesLayout = QtWidgets.QVBoxLayout(eyesGroup)
        eyesLayout.setContentsMargins(8, 8, 8, 8)
        eyesLayout.setSpacing(6)
        eyesLayout.addWidget(splitter_ThreshEyes)

        tailGroup = QtWidgets.QGroupBox("Tail")
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
        self.w8.addWidget(selectionGroup, row=1, col=0, rowspan=3)

        self.w8.addWidget(bgGroup, row=0, col=1, colspan=2)
        self.w8.addWidget(eyesGroup, row=1, col=1, colspan=2)
        self.w8.addWidget(tailGroup, row=2, col=1, colspan=2)
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

    def get_tail_arc_roi_params(self):
        if hasattr(self, "tailArcROI") and self.tailArcROI.initialized:
            return self.tailArcROI.get_parameters()
        return None

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
                    time.sleep(0.1)

                if hasattr(self.video_capture_widget, "start_acquisition"):
                    self.video_capture_widget.start_acquisition()
                    time.sleep(0.1)
            except Exception as exc:
                print("Erreur start acquisition:", exc)
                self.track_checkBox.setChecked(False)
                return

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

    def update_tail_arc_curve(self, value):
        if hasattr(self, "tailArcROI") and self.tailArcROI.initialized:
            self.tailArcROI.set_curve_value(value)

            if not self.track_checkBox.isChecked() and self.selectTailRoot_radioButton.isChecked()==True:
                self.update_tail_segment_thresh(self.threshTail_slider.value())

    def update_tail_segment_thresh(self,thresh_value):
        if not self.track_checkBox.isChecked() and self.selectTailRoot_radioButton.isChecked()==True:

            if len(self.mark.data) == 0:
                return

            frame = self.video_capture_widget.videoDisplayer_updater.current_frame_to_display

            if frame is None:
                return

            if not self.tailArcROI.initialized:
                tailRoot = self.mark.data['pos'][0]
                nose = self.mark.data['pos'][1]
                tail = self.mark.data['pos'][2]
                self.tailArcROI.initialize_from_points(tailRoot, nose, tail)
                self.tailArcROI.set_curve_value(self.tailArcCurve_slider.value())

            arc_roi = self.get_tail_arc_roi_params()

            if arc_roi is None:
                return

            iframe,tail_pos,tail_angle = tail_Track_arc_fast(0, frame, thresh_value, arc_roi)

            if tail_pos[0] != 0 or tail_pos[1] != 0:
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

                varM.bodyAxis_Y=noseY

                xv= noseX-rootX
                yv= noseY-rootY

                varM.bodyAngle = math.atan2(yv, xv)* 180 / math.pi
                print("angle de l'axe du corps : ",varM.bodyAngle)

                if not self.tailArcROI.initialized:
                    self.tailArcROI.initialize_from_points(tailRoot, nose, tail)
                    self.tailArcROI.set_curve_value(self.tailArcCurve_slider.value())
                else:
                    self.tailArcROI.update_graph()

                frame = self.video_capture_widget.videoDisplayer_updater.current_frame_to_display

                if frame is not None:
                    arc_roi = self.get_tail_arc_roi_params()

                    if arc_roi is not None:
                        iframe,tail_pos,tail_angle = tail_Track_arc_fast(
                            0,
                            frame,
                            self.threshTail_slider.value(),
                            arc_roi
                        )

                        if tail_pos[0] != 0 or tail_pos[1] != 0:
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
