# -*- coding: utf-8 -*-
"""
Created on Fri Nov 19 14:46:52 2021

@author: Courtand, Kadri 

ROI widgets and helper functions used to define analysis regions on the video view.
"""

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSignal

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore as QtgCore, QtGui as QtgGui
from pyqtgraph.Point import Point
from pyqtgraph import functions as fn

import numpy as np
import math
import cv2

from xenopus_app.video import video_state as vp




main_rois_dict = {}
analysis_rois_dict = {}
objects_rois_dict = {}
exclusion_rois_dict ={}

roisExclusion=[]


class UIVideoDisplayRoi(pg.GraphicsLayoutWidget):
    """Pyqtgraph image display widget configured for interactive ROI drawing."""
    def __init__(self,video, parent=None):
        """Initialize the object state and graphical items."""
        super(UIVideoDisplayRoi,self).__init__()

        self.vid=video


        self.gviewBox=gViewBox_InteractROI()

        self.plotView=self.addPlot(viewBox=self.gviewBox)

        self.plotView.setAspectLocked(True)


        self.img = pg.ImageItem()

        self.gviewBox.addItem(self.img)

    def show_frame_in_pyqtgraph(self,cv2frame):
        """Convert and display a camera frame in the pyqtgraph image item."""
        tFrame=vp.convert_imageToPyqtgraph(cv2frame,self.vid)
        self.img.setImage(tFrame,autoLevels=False)

def del_main_roi(display) :
    """Remove all main analysis ROIs from the display."""
    print(" ")
    print("Main Roi : ",main_rois_dict)
    print("removal ... ")
    for roi in main_rois_dict.values() :
        print("delete ", roi)
        display.removeItem(roi)
        if len(roi.children)>1 :
            for child in roi.children:
                print("delete ", child)
                display.removeItem(child)
    main_rois_dict.clear()


def del_object_roi(display) :
    """Remove all object-interaction ROIs from the display."""
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
    """Remove all exclusion ROIs from the display."""
    print(" ")
    print("exclusion Roi dictionnary : ",main_rois_dict)
    print("removal ... ")
    for key,roi in objects_rois_dict.items() :
        print("delete ", key)
        display.removeItem(roi)
    exclusion_rois_dict.clear()



class gViewBox_InteractROI(pg.ViewBox):
    """Custom ViewBox that emits a signal when the user draws a rectangular ROI."""

    createROISignal=pyqtSignal(tuple)

    def __init__(self, parent=None, border=None, lockAspect=False, enableMouse=True, invertY=False, enableMenu=True, name=None, invertX=False):
        """Initialize the object state and graphical items."""

        pg.ViewBox.__init__(self, parent=None, border=None, lockAspect=False, enableMouse=True, invertY=False, enableMenu=True, name=None, invertX=False)

    def mouseDragEvent(self, ev, axis=None):
        """Handle mouse dragging for ROI creation, view panning, or marker movement."""

        ev.accept()

        pos = ev.pos()
        lastPos = ev.lastPos()
        dif = pos - lastPos
        dif = dif * -1


        mouseEnabled = np.array(self.state['mouseEnabled'], dtype=float)
        mask = mouseEnabled.copy()
        if axis is not None:
            mask[1-axis] = 0.0



        if (QtCore.Qt.LeftButton):
            if self.state['mouseMode'] == pg.ViewBox.RectMode:
                if ev.isFinish():

                    self.rbScaleBox.hide()

                    ax = QtCore.QRectF(Point(ev.buttonDownPos(ev.button())), Point(pos))
                    ax = self.childGroup.mapRectFromParent(ax)








                    self.createROISignal.emit(ax.getRect())


                    self.setMouseMode(self.PanMode)
                else:

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
    """Interactive rectangular ROI with behavior-analysis metadata and a rename action."""
    sigRenameRequested = QtCore.pyqtSignal(object)

    def __init__(self, pos, size, centered, sideScalers=False, **args):
        """Initialize the object state and graphical items."""

        super(gRectROI,self).__init__(pos, size, centered, sideScalers, **args)



        self.name=None


        self.width0=size[0]
        self.height0=size[1]

        self.x0=pos[0]
        self.y0=pos[1]
        self.geometry="rectangle"
        self.targetEllipse=None

        self.interactArea=None


        self.nbEntries=0

        self.timeIn=0

        self.targetIn = False



        self.detectionDict={}




        self.coords={}
        self.distanceList=[]
        self.activityList=[]

        self.children=[]

    def getMenu(self):
        """Create or return the context menu associated with the ROI."""
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



        self.menu.setEnabled(self.contextMenuEnabled())
        return self.menu

    def renameClicked(self):
        """Ask the user for a new ROI name and emit the rename signal."""

        roiName, ok = QtWidgets.QInputDialog.getText(None, 'Tracker name', 'Give the new tracker a name:')
        self.name=roiName
        print("roi renamed : ",roiName)

        QtCore.QTimer.singleShot(0, self._emitRenameRequest)


    def _emitRenameRequest(self):
        """Emit the delayed ROI rename signal."""
        self.sigRenameRequested.emit(self)



class gCircleROI(pg.CircleROI):
    """Interactive circular ROI with behavior-analysis metadata and a rename action."""
    sigRenameRequested = QtCore.pyqtSignal(object)

    def __init__(self, pos, size, **args):
        """Initialize the object state and graphical items."""
        super(gCircleROI, self).__init__(pos, size, **args)

        self.name=None

        self.width0=size[0]
        self.height0=size[1]

        self.x0=pos[0]
        self.y0=pos[1]
        self.geometry="circle"
        self.targetEllipse=None

        self.interactArea=None


        self.nbEntries=0

        self.timeIn=0

        self.targetIn = False


        self.detectionDict={}

        self.distanceList=[]
        self.activityList=[]

    def getMenu(self):
        """Create or return the context menu associated with the ROI."""
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



        self.menu.setEnabled(self.contextMenuEnabled())
        return self.menu

    def renameClicked(self):
        """Ask the user for a new ROI name and emit the rename signal."""

        newName, ok = QtWidgets.QInputDialog.getText(None, 'Tracker name', 'Give the new tracker a name:')
        self.name=newName
        print("roi renamed : ",newName)


        QtCore.QTimer.singleShot(0, self._emitRenameRequest)


    def _emitRenameRequest(self):
        """Emit the delayed ROI rename signal."""
        print("signal emission : renamerequest")
        self.sigRenameRequested.emit(self)


class gMainRectROI(pg.RectROI):
    """Main rectangular ROI used as the parent area for an experiment."""
    def __init__(self, pos, size, centered, sideScalers=False, **args):
        """Initialize the object state and graphical items."""

        super(gMainRectROI,self).__init__(pos, size, centered, sideScalers, **args)

        self.geometry="rectangle"
        self.children=[]
        self.objects=[]
        self.handlePen = pg.QtGui.QPen(pg.QtGui.QColor(0, 255, 0),2)
        self.handleSize = 6


class gMainCircROI(pg.CircleROI):
    """Main circular ROI used as the parent area for an experiment."""
    def __init__(self, pos, size, **args):
        """Initialize the object state and graphical items."""

        super(gMainCircROI,self).__init__(pos, size, **args)

        self.geometry="circle"
        self.children=[]
        self.objects=[]


class RectROI_centered_noHandle(pg.ROI):
    """Non-movable rectangular ROI used for child analysis areas or interaction zones."""
    def __init__(self, pos, size,sideScalers=False, **args):
        """Initialize the object state and graphical items."""


        super(RectROI_centered_noHandle,self).__init__(pos, size, sideScalers, movable=False, **args)


        self.nbEntries=0

        self.timeIn=0

        self.targetIn = False


        self.detectionDict={}


class CircROI_centered_noHandle(pg.CircleROI):
    """Non-movable circular ROI used for child analysis areas or interaction zones."""
    def __init__(self, pos, size, **args):
        """Initialize the object state and graphical items."""


        super(CircROI_centered_noHandle,self).__init__(pos, size, movable=False, **args)


        self.nbEntries=0

        self.timeIn=0

        self.targetIn = False


        self.detectionDict={}

    def _addHandles(self):
        """Disable automatic handles for this ROI subclass."""
        pass


class EllipseROI_Centered_NoHandle(pg.ROI):
    """Read-only elliptical ROI used to display detected eye orientation."""
    def __init__(self, pos, size, **args):
        """Initialize the object state and graphical items."""

        pg.ROI.__init__(self, pos, size,movable=False, **args)




        self.savedState=None
        self.descriptor={}

        self.descriptorList=[0,0,0,0,0,0,0,0,0]

        self.diameterList=[]
        self.distanceList=[]
        self.minDistList=[]
        self.activityList=[]





        self.values={}

        self.results={}

        self.angleList=[[0,0]]
        self.angleList0=[[0,0]]
        self.yList=[[0,0]]

    def paint(self, p, opt, widget):
        """Draw the ellipse while preserving antialiasing."""
        r = self.boundingRect()
        p.setRenderHint(QtgGui.QPainter.Antialiasing)
        p.setPen(self.currentPen)
        p.scale(r.width(), r.height())
        r = QtgCore.QRectF(r.x()/r.width(), r.y()/r.height(), 1,1)
        p.drawEllipse(r)

    def shape(self):
        """Return the interactive shape used by Qt hit testing."""
        self.path = QtgGui.QPainterPath()
        self.path.addEllipse(self.boundingRect())
        return self.path





def remove_ROI():
    """Placeholder callback kept for compatibility with legacy ROI removal signals."""
    print("not implemented yet")


def create_newMainRoi(display,video,image,experiment,shape) :
    """Create a main ROI covering the current video frame."""

    pos=[0,0]
    size=[video.width*video.rescale,video.height*video.rescale]

    pen1=pg.mkPen((0,255,0), width=4)

    n=len(main_rois_dict)
    roiName="parentROI"+str(n)
    if shape=="rect" or shape==False:

        main_rois_dict[roiName]=gMainRectROI(pos, size, centered=True, pen=pen1)


        add_scale_handle(main_rois_dict[roiName])
    elif shape=="circ":
        main_rois_dict[roiName]=gMainCircROI(pos, size, pen=pen1)

        add_scale_handle(main_rois_dict[roiName])
    elif shape=="poly":




        main_rois_dict[roiName]=pg.MultiRectROI([[200,400], [200,100], [400,100]], width=60, pen=(2,9))

    main_rois_dict[roiName].sigRemoveRequested.connect(remove_ROI)

    main_rois_dict[roiName].sigRegionChanged.connect(lambda ROI: mainRoi_changed(video,image,experiment,ROI,roiName))
    display.plotView.addItem(main_rois_dict[roiName])


def create_childRoi(display,video,image,experiment,ratioCenterArea):
    """Create child ROIs inside each main ROI for the selected experiment layout."""
    w=video.width
    h=video.height

    pen1=pg.mkPen((0,255,0), width=4)
    if experiment=="Open field centered":
        for parentName, parentROI in main_rois_dict.items():


            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))




            centerAreaWidth=parentROI.size()[0]*ratioCenterArea/100
            centerAreaHeight=parentROI.size()[1]*ratioCenterArea/100
            centerAreaPosx=parentROI.pos()[0]+parentROI.size()[0]/2-centerAreaWidth/2
            centerAreaPosy=parentROI.pos()[1]+parentROI.size()[1]/2-centerAreaHeight/2

            childROIcentered=RectROI_centered_noHandle([centerAreaPosx,centerAreaPosy], [centerAreaWidth, centerAreaHeight], pen=pen1)
            childROIcentered.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROIcentered)

            parentROI.children.append(childROIcentered)

    elif experiment=="Open field quadrant":
        childROI=[]

        for parentName, parentROI in main_rois_dict.items():

            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))


            xtl=parentROI.pos()[0]
            ytl=parentROI.pos()[1]+parentROI.size()[1]/2
            childROItopleft=RectROI_centered_noHandle([xtl, ytl], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROItopleft.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROItopleft)

            childROI.append(childROItopleft)

            xtr=parentROI.pos()[0]+parentROI.size()[0]/2
            ytr=parentROI.pos()[1]+parentROI.size()[1]/2
            childROItopright=RectROI_centered_noHandle([xtr, ytr], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROItopright.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROItopright)

            childROI.append(childROItopright)

            xdr=parentROI.pos()[0]+parentROI.size()[0]/2
            ydr=parentROI.pos()[1]
            childROIdownright=RectROI_centered_noHandle([xdr, ydr], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROIdownright.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROIdownright)

            childROI.append(childROIdownright)

            xdl=parentROI.pos()[0]
            ydl=parentROI.pos()[1]
            childROIdownleft=RectROI_centered_noHandle([xdl, ydl], [parentROI.size()[0]/2, parentROI.size()[1]/2], pen=pen1)
            childROIdownleft.sigRemoveRequested.connect(remove_ROI)
            display.plotView.addItem(childROIdownleft)

            childROI.append(childROIdownleft)



            parentROI.children.extend(childROI)

    elif experiment=="Three chambers":
        for parentName, parentROI in main_rois_dict.items():
            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))




            wp,hp=parentROI.size()



            childLeftROI=RectROI_centered_noHandle(parentROI.pos(), [wp/3, h], parent=(parentROI),pen=pen1)
            childLeftROI.addScaleHandle([1, 0.5], [0, 0.5])

            parentROI.children.append(childLeftROI)


            display.plotView.addItem(childLeftROI)


            childCenterROI=RectROI_centered_noHandle([parentROI.pos()[0]+wp/3,parentROI.pos()[1]], [wp/3, h],parent=(parentROI),pen=pen1)

            parentROI.children.append(childCenterROI)

            display.plotView.addItem(childCenterROI)


            childRightROI=RectROI_centered_noHandle([parentROI.pos()[0]+2*wp/3, parentROI.pos()[1]], [wp/3, h],parent=(parentROI),pen=pen1)
            childRightROI.addScaleHandle([0, 0.5], [1, 0.5])

            parentROI.children.append(childRightROI)

            display.plotView.addItem(childRightROI)

            parentROI.children[0].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))
            parentROI.children[2].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))

            parentROI.width=w
            parentROI.children[0].width=w/3
            parentROI.children[1].width=w/3
            parentROI.children[2].width=w/3

    elif experiment=="T maze":

        for parentName, parentROI in main_rois_dict.items():
            parentROI.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))


            wp,hp=parentROI.size()
            xp,yp=parentROI.pos()



            childLeftROI=RectROI_centered_noHandle([xp,yp+hp*3/4], [wp*3/7, hp/4], parent=(parentROI),pen=pen1)
            childLeftROI.addScaleHandle([1, 0.5], [0, 0.5])

            parentROI.children.append(childLeftROI)


            display.plotView.addItem(childLeftROI)


            childCenterROI=RectROI_centered_noHandle([xp+wp*3/7,yp], [wp*1/7, hp],parent=(parentROI),pen=pen1)

            parentROI.children.append(childCenterROI)

            display.plotView.addItem(childCenterROI)


            childRightROI=RectROI_centered_noHandle([xp+wp*4/7,yp+hp*3/4], [wp*3/7, hp/4],parent=(parentROI),pen=pen1)
            childRightROI.addScaleHandle([0, 0.5], [1, 0.5])

            parentROI.children.append(childRightROI)

            display.plotView.addItem(childRightROI)

            parentROI.children[0].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))
            parentROI.children[2].sigRegionChanged.connect(lambda roi:childRoi_Changed(roi,parentROI))

            parentROI.width=w
            parentROI.children[0].width=wp*3/7
            parentROI.children[1].width=wp*1/7
            parentROI.children[2].width=wp*3/7
            parentROI.children[0].height=hp/4
            parentROI.children[1].height=hp
            parentROI.children[2].height=hp/4


def duplicate_roi(display):
    """Duplicate the last main ROI and place the copy next to it."""
    if len(main_rois_dict)>0:
        previousRoi="parentROI"+str(len(main_rois_dict)-1)
        pos=main_rois_dict[previousRoi].pos()
        size=main_rois_dict[previousRoi].size()
        pos[0]=pos[0]+size[0]+6
        pen1=pg.mkPen((0,255,0), width=4)
        roiName="parentROI"+str(len(main_rois_dict))

        main_rois_dict[roiName]=gMainRectROI(pos, size, centered=True, pen=pen1)
        main_rois_dict[roiName].sigRemoveRequested.connect(remove_ROI)
        add_scale_handle(main_rois_dict[roiName])

        display.plotView.addItem(main_rois_dict[roiName])

def add_scale_handle(roi):
    """Add the standard vertical and horizontal scale handles to an ROI."""

    roi.addScaleHandle([0.5, 0], [0.5, 1])
    roi.addScaleHandle([0.5, 1], [0.5, 0])

    roi.addScaleHandle([0, 0.5], [1, 0.5])
    roi.addScaleHandle([1, 0.5], [0, 0.5])




def split_roi(display,video,image,experiment,ratioCenterArea):
    """Split each main ROI horizontally or vertically depending on its geometry."""

    for key,roi in main_rois_dict.items():

        if roi.size()[0]>roi.size()[1]:
            horizontal_split(display,video,image,experiment,key,roi,roi.size(),roi.pos(),ratioCenterArea)

        else:
            vertical_split(display,video,image,experiment,key,roi,roi.size(),roi.pos())

def horizontal_split(display,video,image,experiment,parentName,roi,size,pos,ratioCenterArea):
    """Split a parent ROI into left and right child ROIs."""
    pen1=pg.mkPen((0,255,0), width=4)

    roi.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))

    xtl=pos[0]
    ytl=pos[1]
    roi.children.append(RectROI_centered_noHandle([xtl, ytl], [size[0]/2, size[1]], pen=pen1))
    roi.children[0].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[0])


    pen2=pg.mkPen((255,0,0), width=3)
    xtr=pos[0]+size[0]/2
    ytr=pos[1]
    roi.children.append(RectROI_centered_noHandle([xtr, ytr], [size[0]/2, size[1]], pen=pen2))
    roi.children[1].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[1])


def vertical_split(display,video,image,experiment,parentName,roi,size,pos,ratioCenterArea):
    """Split a parent ROI into bottom and top child ROIs."""
    pen1=pg.mkPen((0,255,0), width=4)

    roi.sigRegionChanged.connect(lambda roi: mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea))

    xtl=pos[0]
    ytl=pos[1]
    roi.children.append(RectROI_centered_noHandle([xtl, ytl], [size[0], size[1]/2], pen=pen1))
    roi.children[0].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[0])


    pen2=pg.mkPen((255,0,0), width=3)
    xtr=pos[0]
    ytr=pos[1]+size[1]/2
    roi.children.append(RectROI_centered_noHandle([xtr, ytr], [size[0], size[1]/2], pen=pen2))
    roi.children[1].sigRemoveRequested.connect(remove_ROI)
    display.plotView.addItem(roi.children[1])





def rotate_around_point_highperf(xy, radians, origin=(0, 0)):
    """Rotate a point around an origin using cached sine and cosine values."""
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
    """Recompute the analysis mask after a main ROI changes."""
    image.mask=create_mask(video,None)


def mainRoiChild_Changed(video,image,experiment,roi,parentName,ratioCenterArea):
    """Update child ROI geometry when a parent ROI changes."""




    newSizeW0,newSizeH0=roi.saveState()['size']
    newPosx0,newPosy0=roi.saveState()['pos']
    newAngle=roi.saveState()['angle']

    if experiment=="Open field centered":



        centerAreaWidth=newSizeW0*ratioCenterArea/100
        centerAreaHeight=newSizeH0*ratioCenterArea/100
        centerAreaPosx=newPosx0+newSizeW0/2-centerAreaWidth/2
        centerAreaPosy=newPosy0+newSizeH0/2-centerAreaHeight/2

        main_rois_dict[parentName].children[0].setState({'size': (centerAreaWidth, centerAreaHeight), 'pos': (centerAreaPosx,centerAreaPosy), 'angle': newAngle})

    elif experiment=="Open field quadrant" :




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





        newSizeW1=main_rois_dict[parentName].children[0].width/roi.width*newSizeW0
        newSizeW3=main_rois_dict[parentName].children[2].width/roi.width*newSizeW0
        newSizeW2=newSizeW0-newSizeW1-newSizeW3


        radangle=newAngle/180*math.pi
        vx=math.cos(radangle)*newSizeW0
        vy=math.sin(radangle)*newSizeW0


        factPos3=(newSizeW1+newSizeW2)/newSizeW0

        xnewpos1=newPosx0
        ynewpos1=newPosy0


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





        newSizeW1=main_rois_dict[parentName].children[0].width/roi.width*newSizeW0
        newSizeW3=main_rois_dict[parentName].children[2].width/roi.width*newSizeW0
        newSizeW2=newSizeW0-newSizeW1-newSizeW3


        radangle=newAngle/180*math.pi
        vx=math.cos(radangle)*newSizeW0
        vy=math.sin(radangle)*newSizeW0


        factxPos3=(newSizeW1+newSizeW2)/newSizeW0
        factyPos3=newSizeH0*3/4

        xnewpos1=newPosx0
        ynewpos1=newPosy0+newSizeH0*3/4


        xnewpos3=newPosx0+factxPos3*vx
        ynewpos3=newPosy0+newSizeH0*3/4+factyPos3*vy

        main_rois_dict[parentName].children[0].setState({'size': (newSizeW1, newSizeH0/4), 'pos': (xnewpos1,ynewpos1), 'angle': newAngle})
        main_rois_dict[parentName].children[2].setState({'size': (newSizeW3, newSizeH0/4), 'pos': (xnewpos3,ynewpos3), 'angle': newAngle})

        roi.width=roi.saveState()['size'][0]
        main_rois_dict[parentName].children[0].width=main_rois_dict[parentName].children[0].saveState()['size'][0]
        main_rois_dict[parentName].children[2].width=main_rois_dict[parentName].children[2].saveState()['size'][0]

    image.mask=create_mask(video,None)



def childRoi_Changed(roi,parentROI):
    """Update parent and sibling ROI geometry after a child ROI changes."""




    newPos0=parentROI.saveState()['pos']
    newSizeW0,newSizeH=parentROI.saveState()['size']
    newAngle=parentROI.saveState()['angle']

    newSizeW1= parentROI.children[0].saveState()['size'][0]
    newSizeW3= parentROI.children[2].saveState()['size'][0]

    newSizeW2,newSizeH=[newSizeW0-newSizeW1-newSizeW3,newSizeH]

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
    """Build an OpenCV mask from the current ROI layout."""



    areaH,areaW=video.currentFrame.shape[0],video.currentFrame.shape[1]

    mask=np.zeros((areaH,areaW),np.uint8)




    if len(analysis_rois_dict)==0 :

        for parentROI in main_rois_dict.values():

            x,y=parentROI.pos()
            w,h=parentROI.size()



            x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)

            x2=x1+w
            y2=y1+h



            if parentROI.geometry=="rectangle":
                cv2.rectangle(mask,(x1,y1),(x2,y2),(255,255,255),-1)
            elif parentROI.geometry=="circle":
                cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[255,255,255],-1)
            else :
                print("unknown geometry : ",parentROI.geometry)
    else :


        for roi in analysis_rois_dict.values() :

            x,y=roi.pos()
            w,h=roi.size()
            x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)

            x2=x1+w
            y2=y1+h


            if roi.geometry=="rectangle":
                cv2.rectangle(mask,(x1,y1),(x2,y2),(255,255,255),-1)
            elif roi.geometry=="circle":
                cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[255,255,255],-1)
            else :
                print("unknown geometry : ",roi.geometry)



    for roi in roisExclusion :
        print("mask exclusion : ",roi.pos())
        x,y=roi.pos()
        w,h=roi.size()
        x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)

        x2=x1+w
        y2=y1+h


        if roi.geometry=="rectangle":
            cv2.rectangle(mask,(x1,y1),(x2,y2),(0,0,0),-1)
        elif roi.geometry=="circle":
            cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[0,0,0],-1)
        else :
            print("unknown geometry : ",roi.geometry)



    if insideMask==True :
        for roi in objects_rois_dict.values() :

            x,y=roi.pos()
            w,h=roi.size()
            x1,y1,w,h=convert_coordToOpencv(x,y,w,h,areaW,areaH)

            x2=x1+w
            y2=y1+h


            if roi.geometry=="rectangle":
                cv2.rectangle(mask,(x1,y1),(x2,y2),(0,0,0),-1)
            elif roi.geometry=="circle":
                cv2.circle(mask,(int((x1+x2)/2),int((y1+y2)/2)),int(w/2),[0,0,0],-1)
            else :
                print("unknown geometry : ",roi.geometry)





    return mask

def convert_coordToOpencv(x,y,w,h,areaW,areaH):
    """Convert pyqtgraph coordinates to OpenCV image coordinates."""




    yCV,xCV,hCV,wCV=int(areaH-y-h),int(x),int(h),int(w)
    return xCV,yCV,wCV,hCV

def convert_coordToPyQtg(x,y,w,h,areaW,areaH):
    """Convert OpenCV image coordinates to pyqtgraph coordinates."""






    ypg,xpg,hpg,wpg=int(areaH-y-h),int(x),int(h),int(w)


    return xpg,ypg,wpg,hpg


def install_event_filter(roiDict):
    """Install a drag event filter on a graphics item."""
    for roi in roiDict.values():
        filter = MouseDragFilter(roi)
        roi.installEventFilter(filter)

class MouseDragFilter(QtCore.QObject):
    """Qt event filter used to emit a callback when a graphics item is dragged."""

    def eventFilter(self,  obj,  event):
        """Forward mouse-drag events to the configured callback."""
        if event.type() == QtCore.QEvent.MouseMove:
            print("filter event")
            return True

        return False





if __name__ == '__main__':

    app = QtgGui.QApplication([])
    video=vp.Video()
