# -*- coding: utf-8 -*-
"""
Created on Fri Nov 19 14:46:52 2021

@author: courtand
"""

from PyQt5 import QtWidgets,QtCore
from PyQt5.QtCore import pyqtSignal
#pyqtgraph
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore as QtgCore, QtGui as QtgGui
from pyqtgraph.Point import Point
from pyqtgraph import functions as fn

import numpy as np
import math
import cv2

import sys
sys.path.append("D:\Developpements\Python\modules")
import video_player_6 as vp  




#liste des rois principales
#rois = []
main_rois_dict = {}
""" dictionnaire de régions d'intérêt contenant 
- en clés : les noms des roi,
- en valeurs : une liste : [mainROI,[childROI1,childROI2,...]]
"""
analysis_rois_dict = {}
""" dictionnaire de régions d'intérêt pour l'analyse multiple 
- en clés : les noms des roi,
- en valeurs : leurs attributs (instance de Roi())
"""
objects_rois_dict = {}
""" dictionnaire de régions d'intérêt entourant les objets placés dans la zone d'analyse contenant 
- en clés : les noms des roi (objets),
- en valeurs : leurs attributs (instance de Roi())
"""
exclusion_rois_dict ={}
""" dictionnaire de régions d'intérêt recouvrant les zones à exclure de l'analyse 
- en clés : les noms des roi,
- en valeurs : leurs attributs (instance de Roi())
"""
#liste des rois contenant les zones à exclure de l'analyse 
roisExclusion=[]



class UIVideoDisplayRoi(pg.GraphicsLayoutWidget):
    def __init__(self,video, parent=None):
        super(UIVideoDisplayRoi,self).__init__()
        
        self.vid=video
        
        #instance de la viewbox spécifique pour le dessin de roi intéractif
        self.gviewBox=gViewBox_InteractROI()
        #ajout d'un plotView avec comme viewbox la gviewBox
        self.plotView=self.addPlot(viewBox=self.gviewBox)
        ## lock the aspect ratio so pixels are always square
        self.plotView.setAspectLocked(True)
        
        ## Create image item
        self.img = pg.ImageItem()
        
        self.gviewBox.addItem(self.img)
        
    def show_frame_in_pyqtgraph(self,cv2frame):
        """
        convert the cv2 frame to pyqtgraph format
        """
        tFrame=vp.convert_imageToPyqtgraph(cv2frame,self.vid)
        self.img.setImage(tFrame,autoLevels=False)

def del_main_roi(display) :
    """
    Supprimer toutes les zones d'analyse principales : surface dans laquelle le déplacement est mesuré
    """
    print(" ")
    print("Main Roi : ",main_rois_dict)
    print("removal ... ")
    for roi in main_rois_dict.values() :
        print("delete ", roi)
        display.removeItem(roi)
        if len(roi.children)>1 : #présence d'une liste de childroi dans la liste roi
            for child in roi.children:
                print("delete ", child)
                display.removeItem(child)
    main_rois_dict.clear()
    

def del_object_roi(display) :
    """
    Supprimer toutes les zones d'intéraction avec un objet
    """
    print(" ")
    print("object Roi dictionnary : ",main_rois_dict)
    print("removal ... ")
    for key,roi in objects_rois_dict.items() :
        print("delete interact area ", key)
        display.removeItem(roi.interactArea)
        print("delete ", key)
        display.removeItem(roi)
    objects_rois_dict.clear()

def del_exclusion_roi(display):
    """
    Supprimer toutes les zones d'exclusions
    """
    print(" ")
    print("exclusion Roi dictionnary : ",main_rois_dict)
    print("supression ... ")
    for key,roi in objects_rois_dict.items() :
        print("delete ", key)
        display.removeItem(roi)            
    exclusion_rois_dict.clear()    
 



class gViewBox_InteractROI(pg.ViewBox):
    """
    sous classe de ViewBox permettant à l'utilisateur de définir une ROI
    par click n drag en mode RectMode (1 button)
    """
    
    createROISignal=pyqtSignal(tuple)
    
    def __init__(self, parent=None, border=None, lockAspect=False, enableMouse=True, invertY=False, enableMenu=True, name=None, invertX=False):
        # On appelle explicitement le constructeur de viewBox
        pg.ViewBox.__init__(self, parent=None, border=None, lockAspect=False, enableMouse=True, invertY=False, enableMenu=True, name=None, invertX=False)
        
    def mouseDragEvent(self, ev, axis=None):
        ## if axis is specified, event will only affect that axis.
        ev.accept()  ## we accept all buttons
        
        pos = ev.pos()
        lastPos = ev.lastPos()
        dif = pos - lastPos
        dif = dif * -1

        ## Ignore axes if mouse is disabled
        mouseEnabled = np.array(self.state['mouseEnabled'], dtype=float)
        mask = mouseEnabled.copy()
        if axis is not None:
            mask[1-axis] = 0.0

        ## Scale or translate based on mouse button
        # if ev.button() & (QtCore.Qt.LeftButton | QtCore.Qt.MidButton):
        if (QtCore.Qt.LeftButton):# | QtCore.Qt.MidButton):
            if self.state['mouseMode'] == pg.ViewBox.RectMode:
                if ev.isFinish():  ## This is the final move in the drag; change the view scale now
                    
                    """
                    self.rbScaleBox.hide()
                    #ax = QtCore.QRectF(Point(self.pressPos), Point(self.mousePos))
                    ax = QtCore.QRectF(Point(ev.buttonDownPos(ev.button())), Point(pos))
                    ax = self.childGroup.mapRectFromParent(ax)
                    self.showAxRect(ax)
                    self.axHistoryPointer += 1
                    self.axHistory = self.axHistory[:self.axHistoryPointer] + [ax]
                    """
                    self.rbScaleBox.hide()
                    #récupération de la position et de la taille du rectangle tracé
                    ax = QtCore.QRectF(Point(ev.buttonDownPos(ev.button())), Point(pos))
                    ax = self.childGroup.mapRectFromParent(ax)
                    #print(ax)
                    #print(ax.getRect())
                    
                    #-----------------------------------------------------------------------------------------------
                    #createROI(pos,size)
                    # create_ROI(ax.getRect())#--------------------------------------------------------------CREATE ROI
                    #TODO : remplacer par un signal proxy sur le bouton relaché ?
                    # print(ax.getRect())
                    self.createROISignal.emit(ax.getRect())
                    
                    #------------------------------------------------------------------------------------------------
                    self.setMouseMode(self.PanMode)
                else:
                    ## update shape of scale box
                    self.updateScaleBox(ev.buttonDownPos(), ev.pos())
            else:
                tr = dif*mask
                tr = self.mapToView(tr) - self.mapToView(Point(0,0))
                x = tr.x() if mask[0] == 1 else None
                y = tr.y() if mask[1] == 1 else None
                
                self._resetTarget()
                if x is not None or y is not None:
                    self.translateBy(x=x, y=y)
                self.sigRangeChangedManually.emit(self.state['mouseEnabled'])
        
        elif ev.button() & QtCore.Qt.RightButton:
            #print "vb.rightDrag"
            if self.state['aspectLocked'] is not False:
                mask[0] = 0
            
            dif = ev.screenPos() - ev.lastScreenPos()
            dif = np.array([dif.x(), dif.y()])
            dif[0] *= -1
            s = ((mask * 0.02) + 1) ** dif
            
            tr = self.childGroup.transform()
            tr = fn.invertQTransform(tr)
            
            x = s[0] if mouseEnabled[0] == 1 else None
            y = s[1] if mouseEnabled[1] == 1 else None
            
            center = Point(tr.map(ev.buttonDownPos(QtCore.Qt.RightButton)))
            self._resetTarget()
            self.scaleBy(x=x, y=y, center=center)
            self.sigRangeChangedManually.emit(self.state['mouseEnabled'])    
            
            

class gRectROI(pg.RectROI):
    """classe héritant des roi rectangulaires de pyqtgraph
    - région d'intérêt créée suite au tracé de l'utilisateur 
    - roi filles d'une roi principale dans lesquelles le comportement de la cible est analysé
        nombre d'entrées, temps passé à l'intérieur, coordonnées de la cible dans la roi         
    """
    sigRenameRequested = QtCore.pyqtSignal(object)   # self
    
    def __init__(self, pos, size, centered, sideScalers=False, **args):
        #pg.RectROI.__init__(self, pos, size, centered, sideScalers=False, **args)
        super(gRectROI,self).__init__(pos, size, centered, sideScalers, **args)
        #self.name="None" # self.__name="None" pour protéger la valeur après instanciation      
        #self.time={0:0} # {numéro_ou_nom_de_phase:temps_passé_dans_la_zone_pour_cette_phase}
        #self.function="None"
        self.name=None
        # self.label=None
        #taille de la roi à l'origine pour le redimentionnement
        self.width0=size[0]
        self.height0=size[1]
        #position de la roi à l'origine
        self.x0=pos[0]
        self.y0=pos[1]
        self.geometry="rectangle"
        self.targetEllipse=None
        
        self.interactArea=None
        
        #nombre d'entrées dans la roi
        self.nbEntries=0
        #temps cumulé dans la roi
        self.timeIn=0
        #cible dans la zone : vrai ou faux
        self.targetIn = False
        # #liste des frames pour lesquels la cible est dans la zone
        # self.detectionList=[]
        #dictionnaire des détections dans une zone : {iframe:"IN",iframe:"-"}
        self.detectionDict={}
        
        #dictionnaire pour organiser les coordonnées par frame pour le features detector
        #plus rapide à parcourir pour modification qu'une liste
        #roi.coords={frame:(frame,x,y),} !ajout de frame dans la liste pour éviter les problème de redondance de liste avec pandas
        self.coords={}
        self.distanceList=[]
        self.activityList=[]
        
        self.children=[] #liste des rois composant la roi qui a été "splitée" en 2 zones d'analyse
        
    def getMenu(self):
        if self.menu is None:
            self.menu = QtWidgets.QMenu()
            self.menu.setTitle(QtCore.QCoreApplication.translate("ROI", "ROI"))
            remAct = QtgGui.QAction(QtCore.QCoreApplication.translate("ROI", "Remove ROI"), self.menu)
            remAct.triggered.connect(self.removeClicked)
            self.menu.addAction(remAct)
            self.menu.remAct = remAct
        
            renameAct = QtgGui.QAction(QtCore.QCoreApplication.translate("ROI", "Rename ROI"), self.menu)
            renameAct.triggered.connect(self.renameClicked)
            self.menu.addAction(renameAct)
            self.menu.renameAct = renameAct
        
        # ROI menu may be requested when showing the handle context menu, so
        # return the menu but disable it if the ROI isn't removable
        self.menu.setEnabled(self.contextMenuEnabled())
        return self.menu
        
    def renameClicked(self):
        #permettre à l'utilisateur de nommer la roi
        roiName, ok = QtWidgets.QInputDialog.getText(None, 'Tracker name', 'Give the new tracker a name:')
        self.name=roiName
        print("roi renamed : ",roiName)
        ## Send remove event only after we have exited the menu event handler
        QtCore.QTimer.singleShot(0, self._emitRenameRequest)
        # roisTrackNames.append(roiName)
   
    def _emitRenameRequest(self):
       self.sigRenameRequested.emit(self)
    
        

class gCircleROI(pg.CircleROI):
    """classe héritant des roi circulaires de pyqtgraph
    class pyqtgraph.CircleROI(pos, size=None, radius=None, **args)
    région d'intérêt créée suite au tracé de l'utilisateur pour définir un objet d'intérêt :
    """
    sigRenameRequested = QtCore.pyqtSignal(object)   # self
    
    def __init__(self, pos, size, **args):
        super(gCircleROI, self).__init__(pos, size, **args)
        
        self.name=None
        #taille de la roi à l'origine pour le redimentionnement
        self.width0=size[0]
        self.height0=size[1]
        #position de la roi à l'origine
        self.x0=pos[0]
        self.y0=pos[1]
        self.geometry="circle"
        self.targetEllipse=None
        
        self.interactArea=None
        
        #nombre d'entrées dans la roi
        self.nbEntries=0
        #temps cumulé dans la roi
        self.timeIn=0
        #cible dans la zone : vrai ou faux
        self.targetIn = False
        #liste des frames pour lesquels la cible est dans la zone
        # self.detectionList=[]
        self.detectionDict={}
        
        self.distanceList=[]
        self.activityList=[]
        
    def getMenu(self):
        if self.menu is None:
            self.menu = QtWidgets.QMenu()
            self.menu.setTitle(QtCore.QCoreApplication.translate("ROI", "ROI"))
            remAct = QtgGui.QAction(QtCore.QCoreApplication.translate("ROI", "Remove ROI"), self.menu)
            remAct.triggered.connect(self.removeClicked)
            self.menu.addAction(remAct)
            self.menu.remAct = remAct
        
            renameAct = QtgGui.QAction(QtCore.QCoreApplication.translate("ROI", "Rename ROI"), self.menu)
            renameAct.triggered.connect(self.renameClicked)
            self.menu.addAction(renameAct)
            self.menu.renameAct = renameAct
        
        # ROI menu may be requested when showing the handle context menu, so
        # return the menu but disable it if the ROI isn't removable
        self.menu.setEnabled(self.contextMenuEnabled())
        return self.menu
        
    def renameClicked(self):
        #permettre à l'utilisateur de nommer la roi
        newName, ok = QtWidgets.QInputDialog.getText(None, 'Tracker name', 'Give the new tracker a name:')
        self.name=newName
        print("roi renamed : ",newName)
        # self.label.setText(newName)
        ## Send remove event only after we have exited the menu event handler
        QtCore.QTimer.singleShot(0, self._emitRenameRequest)
        # roisTrackNames.append(roiName)
   
    def _emitRenameRequest(self):
        print("signal emission : renamerequest") #ne fonctionne pas !
        self.sigRenameRequested.emit(self)


class gMainRectROI(pg.RectROI):
    """classe héritant des roi rectangulaires de pyqtgraph
    - région d'intérêt principale définissant une manip (experiment?)
    - cette roi a comme attribut child : les rois définissant des zones d'analyse du comportement 
        composant la zone principale sous forme de liste
    """
    def __init__(self, pos, size, centered, sideScalers=False, **args):
        #pg.RectROI.__init__(self, pos, size, centered, sideScalers=False, **args)
        super(gMainRectROI,self).__init__(pos, size, centered, sideScalers, **args)
        
        self.geometry="rectangle"
        self.children=[] #liste des rois composant la roi principale
        self.objects=[] #liste des objets désignés dans la roi principale
        self.handlePen = pg.QtGui.QPen(pg.QtGui.QColor(0, 255, 0),2)
        self.handleSize = 6
        
        
class gMainCircROI(pg.CircleROI):
    """classe héritant des roi circulaires de pyqtgraph
    - région d'intérêt principale définissant une manip (experiment?)
    - cette roi a comme attribut child : les rois définissant des zones d'analyse du comportement 
        composant la zone principale sous forme de liste
    """
    def __init__(self, pos, size, **args):
        #pg.RectROI.__init__(self, pos, size, centered, sideScalers=False, **args)
        super(gMainCircROI,self).__init__(pos, size, **args)
        
        self.geometry="circle"
        self.children=[] #liste des rois composant la roi principale
        self.objects=[] #liste des objets désignés dans la roi principale
        

class RectROI_centered_noHandle(pg.ROI):
    """
    Rectangular ROI subclass with a single scale handle at the top-right corner.
    - roi définissant une zone dans laquelle le comportement de la cible est analysé
    - roi définissant la zone d'intéraction autour d'un objet 
    ============== =============================================================
    **Arguments**
    pos            (length-2 sequence) The position of the ROI origin.
                   See ROI().
    size           (length-2 sequence) The size of the ROI. See ROI().
    movable        (bool) If True, the ROI can be moved by dragging anywhere 
                     inside the ROI. Default is True.    
    \**args        All extra keyword arguments are passed to ROI()
    ============== =============================================================
    
    """
    def __init__(self, pos, size,sideScalers=False, **args):
        #QtGui.QGraphicsRectItem.__init__(self, 0, 0, size[0], size[1])
        #pg.ROI.__init__(self, pos, size ,movable=False, **args)
        super(RectROI_centered_noHandle,self).__init__(pos, size, sideScalers, movable=False, **args)
        
        #nombre d'entrées dans la roi
        self.nbEntries=0
        #temps cumulé dans la roi
        self.timeIn=0
        #cible dans la zone : vrai ou faux
        self.targetIn = False
        #liste des frames pour lesquels la cible est dans la zone
        # self.detectionList=[]
        self.detectionDict={}
            

class CircROI_centered_noHandle(pg.CircleROI):
    """
    Circular ROI subclass with a single scale handle at the top-right corner.
    roi définissant une zone dans laquelle le comportement de la cible est analysé 
    ============== =============================================================
    **Arguments**
    pos            (length-2 sequence) The position of the ROI origin.
                   See ROI().
    size           (length-2 sequence) The size of the ROI. See ROI().
    movable        (bool) If True, the ROI can be moved by dragging anywhere 
                     inside the ROI. Default is True.    
    \**args        All extra keyword arguments are passed to ROI()
    ============== =============================================================
    
    """
    def __init__(self, pos, size, **args):
        #QtGui.QGraphicsRectItem.__init__(self, 0, 0, size[0], size[1])
        #pg.ROI.__init__(self, pos, size ,movable=False, **args)
        super(CircROI_centered_noHandle,self).__init__(pos, size, movable=False, **args)
        
        #nombre d'entrées dans la roi
        self.nbEntries=0
        #temps cumulé dans la roi
        self.timeIn=0
        #cible dans la zone : vrai ou faux
        self.targetIn = False
        #liste des frames pour lesquels la cible est dans la zone
        # self.detectionList=[]
        self.detectionDict={}
    
    def _addHandles(self):
        pass            


class EllipseROI_Centered_NoHandle(pg.ROI):
    """
    Elliptical ROI subclass without scale handle nor rotation handle.
        
    ============== =============================================================
    **Arguments**
    pos            (length-2 sequence) The position of the ROI's origin.
    size           (length-2 sequence) The size of the ROI's bounding rectangle.
    
    \**args        All extra keyword arguments are passed to ROI()
    ============== =============================================================
    
    """
    def __init__(self, pos, size, **args):
        #QtGui.QGraphicsRectItem.__init__(self, 0, 0, size[0], size[1])
        pg.ROI.__init__(self, pos, size,movable=False, **args)
        #super().__init__(self, pos, size, **args)
        #self.addRotateHandle([1.0, 0.5], [0.5, 0.5])
        #self.addScaleHandle([0.5*2.**-0.5 + 0.5, 0.5*2.**-0.5 + 0.5], [0.5, 0.5])
        
        self.savedState=None
        self.descriptor={}
        # en live le numero de frame n'est pas accessible, descriptorList n'est pas conservé
        self.descriptorList=[0,0,0,0,0,0,0,0,0]#[xp,yp,Ma,ma,angle,vx,vy,xrot,yrot] 

        self.diameterList=[] #liste pour l'affichage du plot
        self.distanceList=[] #liste pour l'affichage du plot
        self.minDistList=[] #liste pour les distances cumulées
        self.activityList=[]
        # # self.angleList=[]#np.zeros(100000,dtype=float)#
        # # self.inAreaList=[]
        #les coordonnées et le diamètre de l'ellipse target 
        #ainsi que la distance avec la position précédente sont stockées 
        #dans un dictionnaire : .values={framei:[xi,yi,distancei,diameterMaxi,diameterMini]}
        self.values={}
        #.values remplacée par .results{iframe : [centroidX,centroidY,maxDiam,distance]}
        self.results={}
        
        self.angleList=[[0,0]]#np.zeros(100000,dtype=float)#
        self.angleList0=[[0,0]]
        self.yList=[[0,0]]
            
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

def remove_ROI():
    print("pas encore implémenté")


def create_newMainRoi(display,video,image,experiment,shape) :
    """
    création de la zone principale définissant un champ d'analyse
    peut être partagé en plusieurs sous-zones "childROI"
    """
    # print("main ROI's shape : ",shape)
    pos=[0,0]
    size=[video.width*video.rescale,video.height*video.rescale]
    #pen = QPen(Qt.green, 3, Qt.DashLine, Qt.RoundCap, Qt.RoundJoin)
    pen1=pg.mkPen((0,255,0), width=4)
    
    n=len(main_rois_dict)
    roiName="parentROI"+str(n)
    if shape=="rect" or shape==False:
        #    main_rois_dict[roiName]=[pg.RectROI(pos, size, pen=pen1)]
        main_rois_dict[roiName]=gMainRectROI(pos, size, centered=True, pen=pen1)
        # main_rois_dict[roiName].sigRemoveRequested.connect(remove_ROI)
#        main_rois_dict[roiName].handlePen = pg.QtGui.QPen(pg.QtGui.QColor(0, 255, 0))
        add_scale_handle(main_rois_dict[roiName])
    elif shape=="circ":
        main_rois_dict[roiName]=gMainCircROI(pos, size, pen=pen1)
        # main_rois_dict[roiName].sigRemoveRequested.connect(remove_ROI)
        add_scale_handle(main_rois_dict[roiName])
    elif shape=="poly":
#        main_rois_dict[roiName]=pg.PolyLineROI([[0,450],[450,450],
#                                               [450,400],[250,400],
#                                               [250,0],[200,0],
#                                               [200,400],[0,400]], closed=True)    
        main_rois_dict[roiName]=pg.MultiRectROI([[200,400], [200,100], [400,100]], width=60, pen=(2,9))
    
    main_rois_dict[roiName].sigRemoveRequested.connect(remove_ROI)
    # main_rois_dict[roiName].sigRegionChanged.connect(mainRoi_Changed)
    main_rois_dict[roiName].sigRegionChanged.connect(lambda ROI: mainRoi_changed(video,image,experiment,ROI,roiName))
    display.plotView.addItem(main_rois_dict[roiName])
    # image.mask=create_mask(video,False)

def create_childRoi(display,video,image,experiment,ratioCenterArea):
    w=video.width
    h=video.height
    
    pen1=pg.mkPen((0,255,0), width=4)
    if experiment=="Open field centered":
        for parentName, parentROI in main_rois_dict.items():
            #parentROI est une liste ne contenant à ce stade que l'instance de la ROI parent
#            parentROI[0].sigRegionChanged.connect(mainRoi_Changed)
            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))
            
            # xrc=parentROI.pos()[0]+parentROI.size()[0]/4
            # yrc=parentROI.pos()[1]+parentROI.size()[1]/4
            
            centerAreaWidth=parentROI.size()[0]*ratioCenterArea/100
            centerAreaHeight=parentROI.size()[1]*ratioCenterArea/100
            centerAreaPosx=parentROI.pos()[0]+parentROI.size()[0]/2-centerAreaWidth/2
            centerAreaPosy=parentROI.pos()[1]+parentROI.size()[1]/2-centerAreaHeight/2
            
            childROIcentered=RectROI_centered_noHandle([centerAreaPosx,centerAreaPosy], [centerAreaWidth, centerAreaHeight], pen=pen1)
            childROIcentered.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROIcentered)
#            child_rois_dict["childROIcentered"]=childROIcentered
            parentROI.children.append(childROIcentered)    
    
    elif experiment=="Open field quadrant":
        childROI=[]
#        for parentROI in main_rois_dict.values():
        for parentName, parentROI in main_rois_dict.items():
#            parentROI[0].sigRegionChanged.connect(mainRoi_Changed)
            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))
            ##roi principale partagée en 4 :
            #top-left
            xtl=parentROI.pos()[0]#+parentROI.size()[0]/4
            ytl=parentROI.pos()[1]+parentROI.size()[1]/2
            childROItopleft=RectROI_centered_noHandle([xtl, ytl], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROItopleft.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROItopleft)
#            child_rois_dict["childROItopleft"]=childROItopleft
            childROI.append(childROItopleft)
            #top-right
            xtr=parentROI.pos()[0]+parentROI.size()[0]/2
            ytr=parentROI.pos()[1]+parentROI.size()[1]/2
            childROItopright=RectROI_centered_noHandle([xtr, ytr], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROItopright.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROItopright)
#            child_rois_dict["childROItopright"]=childROItopright
            childROI.append(childROItopright)
            #down-right
            xdr=parentROI.pos()[0]+parentROI.size()[0]/2
            ydr=parentROI.pos()[1]
            childROIdownright=RectROI_centered_noHandle([xdr, ydr], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROIdownright.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROIdownright)
#            child_rois_dict["childROIdownright"]=childROIdownright
            childROI.append(childROIdownright)
            #down-left
            xdl=parentROI.pos()[0]
            ydl=parentROI.pos()[1]
            childROIdownleft=RectROI_centered_noHandle([xdl, ydl], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROIdownleft.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROIdownleft)
#            child_rois_dict["childROIdownleft"]=childROIdownleft
            childROI.append(childROIdownleft)
            
            #ajouter la liste des childROI dans la liste contenant parentROI dans le dictionnaire main_rois_dict
#            parentROI.children.append(childROI)
            parentROI.children.extend(childROI)
    
    elif experiment=="Three chambers":
        for parentName, parentROI in main_rois_dict.items():
            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))                 
            #three chambers area
            #one simple area plus 3 childs area
            
            #champ à gauche sur l'image
            wp,hp=parentROI.size()
            #boundRect=QtgCore.QRect(0,0,wp,hp)
            #Roi(self, pos, size, centered, sideScalers=False, **args)
                       
            childLeftROI=RectROI_centered_noHandle(parentROI.pos(), [wp/3, h], parent=(parentROI),pen=pen1)
            childLeftROI.addScaleHandle([1, 0.5], [0, 0.5])

            parentROI.children.append(childLeftROI)
            #childLeftROI.sigRemoveRequested.connect(remove_ROI)
            #rois[-1].sigRegionChanged.connect(childRoi_Changed)
            display.plotView.addItem(childLeftROI)
            
            #champ central
            childCenterROI=RectROI_centered_noHandle([parentROI.pos()[0]+wp/3,parentROI.pos()[1]], [wp/3, h],parent=(parentROI),pen=pen1)

            parentROI.children.append(childCenterROI)
            #child_Center_ROI.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childCenterROI)
            
            #champ à droite sur l'image
            childRightROI=RectROI_centered_noHandle([parentROI.pos()[0]+2*wp/3, parentROI.pos()[1]], [wp/3, h],parent=(parentROI),pen=pen1)
            childRightROI.addScaleHandle([0, 0.5], [1, 0.5])

            parentROI.children.append(childRightROI)
            #childRightROI.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childRightROI)
            
            parentROI.children[0].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))
            parentROI.children[2].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))
            
            parentROI.width=w
            parentROI.children[0].width=w/3
            parentROI.children[1].width=w/3
            parentROI.children[2].width=w/3     
    
    elif experiment=="T maze":
        #T maze calculé sur la base de couloirs de 10cm de large et de 30cm de long
        for parentName, parentROI in main_rois_dict.items():
            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))                 
           
            #champ à gauche sur l'image
            wp,hp=parentROI.size()
            xp,yp=parentROI.pos()
            #boundRect=QtgCore.QRect(0,0,wp,hp)
            #Roi(self, pos, size, centered, sideScalers=False, **args)
                       
            childLeftROI=RectROI_centered_noHandle([xp,yp+hp*3/4], [wp*3/7, hp/4], parent=(parentROI),pen=pen1)
            childLeftROI.addScaleHandle([1, 0.5], [0, 0.5])

            parentROI.children.append(childLeftROI)
            #childLeftROI.sigRemoveRequested.connect(remove_ROI)
            #rois[-1].sigRegionChanged.connect(childRoi_Changed)
            display.plotView.addItem(childLeftROI)
            
            #champ central
            childCenterROI=RectROI_centered_noHandle([xp+wp*3/7,yp], [wp*1/7, hp],parent=(parentROI),pen=pen1)

            parentROI.children.append(childCenterROI)
            #child_Center_ROI.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childCenterROI)
            
            #champ à droite sur l'image
            childRightROI=RectROI_centered_noHandle([xp+wp*4/7,yp+hp*3/4], [wp*3/7, hp/4],parent=(parentROI),pen=pen1)
            childRightROI.addScaleHandle([0, 0.5], [1, 0.5])

            parentROI.children.append(childRightROI)
            #childRightROI.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childRightROI)
            
            parentROI.children[0].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))
            parentROI.children[2].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))
            #initialisation des propriétés
            parentROI.width=w
            parentROI.children[0].width=wp*3/7
            parentROI.children[1].width=wp*1/7
            parentROI.children[2].width=wp*3/7
            parentROI.children[0].height=hp/4
            parentROI.children[1].height=hp
            parentROI.children[2].height=hp/4




def duplicate_roi(display):
    """
    création d'une nouvelle zone principale définissant un champ d'analyse
    à partir de la précedente (si elle a été créée avec le bouton addROI)
    """
    if len(main_rois_dict)>0:
        previousRoi="parentROI"+str(len(main_rois_dict)-1)
        pos=main_rois_dict[previousRoi].pos()
        size=main_rois_dict[previousRoi].size()
        pos[0]=pos[0]+size[0]+6
        pen1=pg.mkPen((0,255,0), width=4)
        roiName="parentROI"+str(len(main_rois_dict))
#        main_rois_dict[roiName]=[pg.RectROI(pos, size, pen=pen1)]
        main_rois_dict[roiName]=gMainRectROI(pos, size, centered=True, pen=pen1)
        main_rois_dict[roiName].sigRemoveRequested.connect(remove_ROI)
        add_scale_handle(main_rois_dict[roiName])
       
        display.plotView.addItem(main_rois_dict[roiName])

def add_scale_handle(roi):
    ## handles scaling vertically from opposite edge
    roi.addScaleHandle([0.5, 0], [0.5, 1])
    roi.addScaleHandle([0.5, 1], [0.5, 0])
    ## handles scaling horizontally from opposite edge
    roi.addScaleHandle([0, 0.5], [1, 0.5])
    roi.addScaleHandle([1, 0.5], [0, 0.5])
    #scale+rotation handle 
    #roi.addRotateHandle([1,0], [0.5,0.5])
    
    
def split_roi(display,video,image,experiment,ratioCenterArea):
    #parcourir le dictionnaire des rois parent
    for key,roi in main_rois_dict.items():
        #tester l'orientation
        if roi.size()[0]>roi.size()[1]:
            horizontal_split(display,video,image,experiment,key,roi,roi.size(),roi.pos(),ratioCenterArea)
            
        else:
            vertical_split(display,video,image,experiment,key,roi,roi.size(),roi.pos())

def horizontal_split(display,video,image,experiment,parentName,roi,size,pos,ratioCenterArea):
    pen1=pg.mkPen((0,255,0), width=4)
    # roi.sigRegionChanged.connect(lambda ROI: mainRoi_changed(video,image,experiment,ROI,parentName))
    roi.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea)) 
    #left ROI
    xtl=pos[0]
    ytl=pos[1]
    roi.children.append(RectROI_centered_noHandle([xtl, ytl], [size[0]/2, size[1]], pen=pen1))
    roi.children[0].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[0])
    
    #right ROI  de couleur différente 
    pen2=pg.mkPen((255,0,0), width=3)
    xtr=pos[0]+size[0]/2
    ytr=pos[1]
    roi.children.append(RectROI_centered_noHandle([xtr, ytr], [size[0]/2, size[1]], pen=pen2))
    roi.children[1].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[1])
    
    
def vertical_split(display,video,image,experiment,parentName,roi,size,pos,ratioCenterArea):
    pen1=pg.mkPen((0,255,0), width=4)
    # roi.sigRegionChanged.connect(lambda ROI: mainRoi_changed(video,image,experiment,ROI,parentName))
    roi.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea)) 
    #bottom ROI
    xtl=pos[0]
    ytl=pos[1]
    roi.children.append(RectROI_centered_noHandle([xtl, ytl], [size[0], size[1]/2], pen=pen1))
    roi.children[0].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[0])
    
    #up ROI
    pen2=pg.mkPen((255,0,0), width=3)
    xtr=pos[0]
    ytr=pos[1]+size[1]/2
    roi.children.append(RectROI_centered_noHandle([xtr, ytr], [size[0], size[1]/2], pen=pen2))
    roi.children[1].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[1])    
    
    # parentROI.children[0].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))
    # parentROI.children[2].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))


def rotate_around_point_highperf(xy, radians, origin=(0, 0)):
    """Rotate a point around a given point.
    I call this the "high performance" version since we're caching some
    values that are needed >1 time.
    """
    x, y = xy
    offset_x, offset_y = origin
    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = math.cos(radians)
    sin_rad = math.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return qx, qy


def mainRoi_changed(video,image,experiment,roi,parentName):
    image.mask=create_mask(video,None)    


def mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea):

    #TODO : intégrer ces méthode à la classe gROI (chambersROI)
    #print("roi-mainroi_Changed : ",roi)

    newSizeW0,newSizeH0=roi.saveState()['size']
    newPosx0,newPosy0=roi.saveState()['pos']
    newAngle=roi.saveState()['angle'] #en degrès
    
    if experiment=="Open field centered":
        # newSizeW1,newSizeH1=newSizeW0/2,newSizeH0/2
        # newPosx1=newPosx0+newSizeW0/4
        # newPosy1=newPosy0+newSizeH0/4
        """    
        # composante du vecteur d'orientation après rotation
        radangle=newAngle/180*math.pi
        vx=math.cos(radangle)*newSizeW0
        vy=math.sin(radangle)*newSizeW0
        
        xnewpos1=newPos0[0]#+0*vx
        ynewpos1=newPos0[1]#+0*vy
        """
        centerAreaWidth=newSizeW0*ratioCenterArea/100
        centerAreaHeight=newSizeH0*ratioCenterArea/100
        centerAreaPosx=newPosx0+newSizeW0/2-centerAreaWidth/2
        centerAreaPosy=newPosy0+newSizeH0/2-centerAreaHeight/2
        
        main_rois_dict[parentName].children[0].setState({'size': (centerAreaWidth, centerAreaHeight), 'pos': (centerAreaPosx,centerAreaPosy), 'angle': newAngle})
       
    elif experiment=="Open field quadrant" :
        #TODO : rotation
        #calcul des coordonnées après rotation de la roi
#        newPosx1,newPosy1=rotate_around_point_highperf(xy, radians, origin=(0, 0))
        
        newSizeW1,newSizeH1=newSizeW0/2,newSizeH0/2
        newPosx1=newPosx0
        newPosy1=newPosy0+newSizeH0/2
        
        main_rois_dict[parentName].children[0].setState({'size': (newSizeW1, newSizeH1), 'pos': (newPosx1,newPosy1), 'angle': newAngle})
        newPosx2=newPosx0+newSizeW0/2
        newPosy2=newPosy0+newSizeH0/2
        main_rois_dict[parentName].children[1].setState({'size': (newSizeW1, newSizeH1), 'pos': (newPosx2,newPosy2), 'angle': newAngle})
        newPosx3=newPosx0+newSizeW0/2
        newPosy3=newPosy0
        main_rois_dict[parentName].children[2].setState({'size': (newSizeW1, newSizeH1), 'pos': (newPosx3,newPosy3), 'angle': newAngle})
        newPosx4=newPosx0
        newPosy4=newPosy0
        main_rois_dict[parentName].children[3].setState({'size': (newSizeW1, newSizeH1), 'pos': (newPosx4,newPosy4), 'angle': newAngle})
    
    elif experiment=="Three chambers":
        #TODO : intégrer ces méthode à la classe gROI (chambersROI)
        #childRightROI.setState( parentROI.saveState() )
        #rois[0].saveState() :
        #{'size': (325.4656283463197, 240.0), 'pos': (-5.465628346319704, 0.0), 'angle': 0.0}
       
        newSizeW1=main_rois_dict[parentName].children[0].width/roi.width*newSizeW0
        newSizeW3=main_rois_dict[parentName].children[2].width/roi.width*newSizeW0
        newSizeW2=newSizeW0-newSizeW1-newSizeW3
            
        # composante du vecteur d'orientation après rotation
        radangle=newAngle/180*math.pi
        vx=math.cos(radangle)*newSizeW0
        vy=math.sin(radangle)*newSizeW0
        
        #factPos2=newSizeW1/newSizeW0
        factPos3=(newSizeW1+newSizeW2)/newSizeW0
        
        xnewpos1=newPosx0
        ynewpos1=newPosy0
        #xnewpos2=newPos0[0]+factpos2*vx
        #ynewpos2=newPos0[1]+factpos2*vy
        xnewpos3=newPosx0+factPos3*vx
        ynewpos3=newPosy0+factPos3*vy
    
        main_rois_dict[parentName].children[0].setState({'size': (newSizeW1, newSizeH0), 'pos': (xnewpos1,ynewpos1), 'angle': newAngle})
        main_rois_dict[parentName].children[2].setState({'size': (newSizeW3, newSizeH0), 'pos': (xnewpos3,ynewpos3), 'angle': newAngle})
        
        roi.width=roi.saveState()['size'][0]
        main_rois_dict[parentName].children[0].width=main_rois_dict[parentName].children[0].saveState()['size'][0]
        main_rois_dict[parentName].children[2].width=main_rois_dict[parentName].children[2].saveState()['size'][0]
    
                   
    elif experiment=="CPP 4 boxes" or experiment=="Dark Light box (red)":
        if roi.size()[0]>roi.size()[1]:
            newPosx1=newPosx0
            newPosy1=newPosy0
            newPosx2=newPosx0+roi.size()[0]/2
            newPosy2=newPosy1
            newSizeW1,newSizeH1=newSizeW0/2,newSizeH0
                    
        else:
            newPosx1=newPosx0
            newPosy1=newPosy0
            newPosx2=newPosx1
            newPosy2=newPosy1+roi.size()[1]/2
            newSizeW1,newSizeH1=newSizeW0,newSizeH0/2
            
        main_rois_dict[parentName].children[0].setState({'size': (newSizeW1, newSizeH1), 'pos': (newPosx1,newPosy1), 'angle': newAngle})
        main_rois_dict[parentName].children[1].setState({'size': (newSizeW1, newSizeH1), 'pos': (newPosx2,newPosy2), 'angle': newAngle})     

    elif experiment=="T maze":
        #TODO : intégrer ces méthode à la classe gROI (chambersROI)
        #childRightROI.setState( parentROI.saveState() )
        #rois[0].saveState() :
        #{'size': (325.4656283463197, 240.0), 'pos': (-5.465628346319704, 0.0), 'angle': 0.0}
       
        newSizeW1=main_rois_dict[parentName].children[0].width/roi.width*newSizeW0
        newSizeW3=main_rois_dict[parentName].children[2].width/roi.width*newSizeW0
        newSizeW2=newSizeW0-newSizeW1-newSizeW3
            
        # composante du vecteur d'orientation après rotation
        radangle=newAngle/180*math.pi
        vx=math.cos(radangle)*newSizeW0
        vy=math.sin(radangle)*newSizeW0
        
        #factPos2=newSizeW1/newSizeW0
        factxPos3=(newSizeW1+newSizeW2)/newSizeW0
        factyPos3=newSizeH0*3/4
        
        xnewpos1=newPosx0
        ynewpos1=newPosy0+newSizeH0*3/4
        #xnewpos2=newPos0[0]+factpos2*vx
        #ynewpos2=newPos0[1]+factpos2*vy
        xnewpos3=newPosx0+factxPos3*vx
        ynewpos3=newPosy0+newSizeH0*3/4+factyPos3*vy
    
        main_rois_dict[parentName].children[0].setState({'size': (newSizeW1, newSizeH0/4), 'pos': (xnewpos1,ynewpos1), 'angle': newAngle})
        main_rois_dict[parentName].children[2].setState({'size': (newSizeW3, newSizeH0/4), 'pos': (xnewpos3,ynewpos3), 'angle': newAngle})
        
        roi.width=roi.saveState()['size'][0]
        main_rois_dict[parentName].children[0].width=main_rois_dict[parentName].children[0].saveState()['size'][0]
        main_rois_dict[parentName].children[2].width=main_rois_dict[parentName].children[2].saveState()['size'][0]
    
    image.mask=create_mask(video,None)
    


def childRoi_Changed(roi,parentROI):
    ##seulement utilisé en mode three chambers
    #childRightROI.setState( parentROI.saveState() )
    #rois[0].saveState() :
    #{'size': (325.4656283463197, 240.0), 'pos': (-5.465628346319704, 0.0), 'angle': 0.0}
    newPos0=parentROI.saveState()['pos']
    newSizeW0,newSizeH=parentROI.saveState()['size']
    newAngle=parentROI.saveState()['angle']
    #print('newAngle :',newAngle)
    newSizeW1= parentROI.children[0].saveState()['size'][0]
    newSizeW3= parentROI.children[2].saveState()['size'][0]
    
    newSizeW2,newSizeH=[newSizeW0-newSizeW1-newSizeW3,newSizeH]
    # composante du vecteur d'orientation après rotation
    radangle=newAngle/180*math.pi
    vx=math.cos(radangle)*newSizeW0
    vy=math.sin(radangle)*newSizeW0
    
    factPos=(newSizeW1)/newSizeW0
    xnewpos2=newPos0[0]+factPos*vx
    ynewpos2=newPos0[1]+factPos*vy
    
    parentROI.width=parentROI.saveState()['size'][0]
    parentROI.children[0].width= parentROI.children[0].saveState()['size'][0]
    parentROI.children[2].width= parentROI.children[2].saveState()['size'][0]
    parentROI.children[1].setState({'size': (newSizeW2, newSizeH), 'pos': (xnewpos2,ynewpos2), 'angle': newAngle})




def create_mask(video,insideMask):
    """
    créer un masque noir et blanc de la taille d'un frame
    en blanc les zones de tracking, en noir les zones masquées, ignorées (non seuillées avec la souris)
    insideMask : ui.insideMask_checkBox.isChecked()
    """
    # print("mask creation...")
    #Shape of image is accessed by img.shape : It returns a tuple of number of rows, columns and channels (if image is color):
    #largeur et hauteur de l'image : shape pour l'image opencv donne h et w dans cet ordre
    areaH,areaW=video.currentFrame.shape[0],video.currentFrame.shape[1]
    #masque noir par défaut :
    mask=np.zeros((areaH,areaW),np.uint8)
    #mask = np.zeros(image.currentFrame.shape,np.uint8)
    #cv2.imshow("mask0",mask)
       
    #tester si des régions d'analyse (multitrack) ont été définies
    if len(analysis_rois_dict)==0 :
        #si pas de roiAnanlysis remplir la zone mainRoi en blanc
        for parentROI in main_rois_dict.values():
             #récupérer les dimensions de la zone d'observation "parent ROI"
            x,y=parentROI.pos()
            w,h=parentROI.size()
            #print(x,y,w,h)
            
            #y1,x1,h,w=convert_coordToOpencv(x,y,w,h,areaW,areaH)
            x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)
            #print("x1,y1,w,h : ", x1,y1,w,h)
            x2=x1+w
            y2=y1+h
            #dessiner un rectangle blanc dans la zone ROI principale
            #cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) → img
            #cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) → None
            if parentROI.geometry=="rectangle":
                cv2.rectangle(mask,(x1,y1),(x2,y2),(255,255,255),-1)
            elif parentROI.geometry=="circle":
                cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[255,255,255],-1)
            else :
                print("unknown geometry : ",parentROI.geometry)
    else :
        #parcourir le dico des zones d'analyse, récupérer leurs coordonnées et dessiner les zones en blanc
        #ne pas dessiner la mainRoi
        for roi in analysis_rois_dict.values() :
            # print("mask analysis : ",roi.pos())
            x,y=roi.pos()
            w,h=roi.size()
            x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)
            #print("x1,y1,w,h : ", x1,y1,w,h)
            x2=x1+w
            y2=y1+h
            #dessiner un rectangle ou cercle noir dans la zone ROI à masquer
            #cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) → img
            if roi.geometry=="rectangle":
                cv2.rectangle(mask,(x1,y1),(x2,y2),(255,255,255),-1)
            elif roi.geometry=="circle":
                cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[255,255,255],-1)
            else :
                print("unknown geometry : ",roi.geometry)
        
#    cv2.imshow("mask",mask)
    #parcourir la liste des zones masquées, récupérer leurs coordonnées et dessiner un masque noir
    for roi in roisExclusion :
        print("mask exclusion : ",roi.pos())
        x,y=roi.pos()
        w,h=roi.size()
        x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)
        #print("x1,y1,w,h : ", x1,y1,w,h)
        x2=x1+w
        y2=y1+h
        #dessiner un rectangle ou cercle noir dans la zone ROI à masquer
        #cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) → img
        if roi.geometry=="rectangle":
            cv2.rectangle(mask,(x1,y1),(x2,y2),(0,0,0),-1)
        elif roi.geometry=="circle":
            cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[0,0,0],-1)
        else :
            print("unknown geometry : ",roi.geometry)
    
    
    #si l'option "insideMask" est cochée, créer un masque pour chaque zone interne des objets sélectionnés
    if insideMask==True :
        for roi in objects_rois_dict.values() :
#            print("mask inside: ",roi.pos())
            x,y=roi.pos()
            w,h=roi.size()
            x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)
            #print("x1,y1,w,h : ", x1,y1,w,h)
            x2=x1+w
            y2=y1+h
            #dessiner un rectangle ou cercle noir dans la zone ROI à masquer
            #cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) → img
            if roi.geometry=="rectangle":
                cv2.rectangle(mask,(x1,y1),(x2,y2),(0,0,0),-1)
            elif roi.geometry=="circle":
                cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[0,0,0],-1)
            else :
                print("unknown geometry : ",roi.geometry)
            
    # cv2.imshow("mask",mask)    
    #inversion des valeurs pour ajouter le masque à l'image seuillée  : élimination des zones seuillées non pertinentes
#    inv_mask=cv2.bitwise_not(mask) #v7 le masque est appliqué à l'image seuillée par "bitwise_add"         
    # print("mask created")
    return mask

def convert_coordToOpencv(x,y,w,h,areaW,areaH):
    #conversion des coordonnées dans pyqtgraph pour opencv
    #x,y,w,h coord et taille dans pyqtgraph
    #areaW,areaH taille de la zone considérée pour exprimer les coordonnées dans pyqtgraph
    #xCV,yCV,wCV,hCV=int(areaH-y-h),int(x),int(h),int(w)
    yCV,xCV,hCV,wCV=int(areaH-y-h),int(x),int(h),int(w)
    return xCV,yCV,wCV,hCV        
    
def convert_coordToPyQtg(x,y,w,h,areaW,areaH):
    #conversion des coordonnées dans opencv pour pyqtgraph
    #x,y,w,h coord et taille dans opencv
    # xroi,yroi=roiTarget.pos()
    # wroi,hroi=roiTarget.size()
    # xroicv,yroicv,wroicv,hroicv=convert_coordToOpencv(xroi,yroi,wroi,hroi,video.width,video.height)
    #areaW,areaH taille de la zone considérée pour exprimer les coordonnées dans opencv
    ypg,xpg,hpg,wpg=int(areaH-y-h),int(x),int(h),int(w)
    # ypg,xpg,hpg,wpg=int(areaH-y),int(x),int(h),int(w)
    
    return xpg,ypg,wpg,hpg            





def install_event_filter(roiDict):
    """
    Intercept events going to the GraphicsView widget using an event filter 
    (http://qt-project.org/doc/qt-4.8/eventsandfilters.html#event-filters)
    """
    for roi in roiDict.values():
        filter = MouseDragFilter(roi)
        roi.installEventFilter(filter)

class MouseDragFilter(QtCore.QObject):
    
    def eventFilter(self,  obj,  event):
        if event.type() == QtCore.QEvent.MouseMove: #gRoi.mouseDragEvent:
            print("filter event")
            return True
       
        return False


            

    
if __name__ == '__main__':
    
    app = QtgGui.QApplication([])
    video=vp.Video()