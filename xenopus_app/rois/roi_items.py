# -*- coding: utf-8 -*-
"""
ROI helper items used by the main image view.

@author: Courtand, Kadri 
"""

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt

def testRoiInImageview(x, y, w, h, img):
    """Clamp an ROI rectangle so that it remains inside the image boundaries."""

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
    """Interactive graph item used to display and drag reference markers."""
    def __init__(self):
        """Initialize the object state and graphical items."""
        self.dragPoint = None
        self.dragOffset = None
        self.textItems = []
        pg.GraphItem.__init__(self)
        self.scatter.sigClicked.connect(self.clicked)

        self.lastConformation=[]
        self.size=0

    def setData(self, **kwds):
        """Set marker data and create per-point indexes for dragging."""
        self.text = kwds.pop('text', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = np.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = np.arange(npts)
        self.setTexts(self.text)
        self.updateGraph()

    def setTexts(self, text):
        """Attach text labels to the marker points."""
        for i in self.textItems:
            i.scene().removeItem(i)
        self.textItems = []
        for t in text:
            item = pg.TextItem(t,color="g")
            self.textItems.append(item)
            item.setParentItem(self)

    def updateGraph(self):
        """Refresh the graph item and reposition its text labels."""
        pg.GraphItem.setData(self, **self.data)
        for i,item in enumerate(self.textItems):
            item.setPos(*self.data['pos'][i])

    def mouseDragEvent(self, ev):
        """Handle mouse dragging for ROI creation, view panning, or marker movement."""

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
        """Debug callback invoked when a marker is clicked."""
        print("clicked: %s" % pts)
