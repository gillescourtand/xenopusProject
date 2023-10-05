# -*- coding: utf-8 -*-
"""
Created on Tue Aug  7 14:57:18 2018

@author: courtand
"""

# -*- coding: utf-8 -*-
"""
This example demonstrates the use of pyqtgraph's parametertree system. This provides
a simple way to generate user interfaces that control sets of parameters. The example
demonstrates a variety of different parameter types (int, float, list, etc.)
as well as some customized parameter types

"""




import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

#import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import types as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType


## test subclassing parameters
## This parameter automatically generates two child parameters which are always reciprocals of each other
class ComplexParameter(pTypes.GroupParameter):
    def __init__(self, **opts):
        opts['type'] = 'bool'
        opts['value'] = True
        opts['readonly'] = False
        pTypes.GroupParameter.__init__(self, **opts)
        
        self.addChild({'name': 'Pixel Format', 'type': 'list', 'values': {"Mono8", "BayerBG8"}, 'value': "Mono8"})
        self.addChild({'name': 'Width', 'type': 'int', 'step': 32, 'limits': (32, 1920),'value': 1920})
        self.addChild({'name': 'Height', 'type': 'int', 'step': 2, 'limits': (2, 1200), 'value': 1200})
        self.addChild({'name': 'Resulting Frame Rate [Hz]', 'type': 'float', 'value': 25.0,'readonly': True})
        self.addChild({'name': 'Enable Frame Rate', 'type': 'bool', 'value': True,'readonly': True,'tip': "stop acquisition to modify"})
        self.addChild({'name': 'Acquisition Frame Rate [Hz]', 'type': 'float', 'value': 25.0,'step': 1.0,'tip': "stop acquisition to modify"})
        
        self.width = self.param('Width')
        self.height = self.param('Height')
        self.resfps = self.param('Resulting Frame Rate [Hz]')
        self.resfps.sigValueChanged.connect(self.resfps_changed)
        self.enablefps= self.param('Enable Frame Rate')
        self.fps=self.param('Acquisition Frame Rate [Hz]')
        self.width.sigValueChanged.connect(self.width_changed)
        self.height.sigValueChanged.connect(self.height_changed)
        #self.enablefps.sigValueChanged.connect(self.enablefps_changed)
                
    def width_changed(self):
        print("sensor width sigValueChanged")
        #width doit être divisible par 32
        #dwidth=self.width.value()//32
        #nwidth=32*dwidth
        #self.width.setValue(nwidth)
        #TODO la valeur n'est pas rafraichie dans la table
        #faire perdre le focus à la ligne pour la rafraichir ???
        self.resfps.setValue(self.resfps.value()*1.00001)
        #
    
    def height_changed(self):
        print("sensor height sigValueChanged")
       
        self.resfps.setValue(self.resfps.value()*1.00001)  
    
    def resfps_changed(self):
        #print("restps change, width value : ",self.width.value())
        dwidth=self.width.value()//32
        nwidth=32*dwidth
        self.width.setValue(nwidth,blockSignal=self.width_changed)
        dheight=self.height.value()//32
        nheight=32*dheight
        self.height.setValue(nheight,blockSignal=self.height_changed)
    
    def enablefps_changed(self):
        #TODO : raise error : Invalid keyword argument 'readonly' -->spinBox.py
        if self.enablefps.value()==True :
            self.fps.setWritable()
        else :
            self.fps.setReadonly()
        

## test add/remove
## this group includes a menu allowing the user to add new parameters into its child list
class ScalableGroup(pTypes.GroupParameter):
    def __init__(self, **opts):
        opts['type'] = 'group'
        opts['addText'] = "Add"
        opts['addList'] = ['str', 'float', 'int']
        pTypes.GroupParameter.__init__(self, **opts)
    
    def addNew(self, typ):
        val = {
            'str': '',
            'float': 0.0,
            'int': 0
        }[typ]
        self.addChild(dict(name="ScalableParam %d" % (len(self.childs)+1), type=typ, value=val, removable=True, renamable=True))






class PyqtgParamWidget(QtGui.QWidget):
    def __init__(self):
        QtGui.QWidget.__init__(self)
        
        self.tree = ParameterTree(showHeader=False)
        
        self.params = Parameter.create(name='params', type='group', children=[
            ComplexParameter(name='Acquisition parameter'),
            {'name': 'AOI Control', 'type': 'group', 'children': [
                {'name': 'Offset X', 'type': 'int', 'value': 32,'step': 32, 'limits': (0, 1472)},
                {'name': 'Offset Y', 'type': 'int', 'value': 32,'step': 2, 'limits': (0, 1200)},
                {'name': 'Center X', 'type': 'bool', 'value': False, 'tip': "This is a checkbox"},
                {'name': 'Center Y', 'type': 'bool', 'value': False, 'tip': "This is a checkbox"},
                       
            ]},
            {'name': 'Analog Control', 'type': 'group', 'children': [
                {'name': 'Gain Auto', 'type': 'list', 'values': {"Off","Once","Continuous"}, 'value':"Off"},
                {'name': 'Gain', 'type': 'float', 'value': 5, 'limits': (1, 12)},
                {'name': 'Gamma', 'type': 'float', 'value': 1.0, 'step': 0.1, 'default': 1.0},
                
            ]},
             {'name': 'Acquisition Control', 'type': 'group', 'children': [
                {'name': 'Exposure Time [µs]', 'type': 'float', 'value': 5000.0},
                {'name': 'Sensor Readout Mode', 'type': 'list', 'values':{"Normal","Fast"},'value':"Normal"},
                        
            ]},
             {'name': 'Save State', 'type': 'action'},
             {'name': 'Restore State', 'type': 'action'},
                
                       
            ScalableGroup(name="Expandable Parameter Group", children=[
                {'name': 'ScalableParam 1', 'type': 'str', 'value': "default param 1"},
                {'name': 'ScalableParam 2', 'type': 'str', 'value': "default param 2"},
            ]),
        ])
        
    
        ## Create ParameterTree widget
        #self.t = ParameterTree()
        #self.t.setParameters(self.params, showTop=False)
        #self.t.setWindowTitle('pyqtgraph example: Parameter Tree')
        self.tree.setParameters(self.params, showTop=False)
        self.params.param('Save State').sigActivated.connect(self.save)
        self.params.param('Restore State').sigActivated.connect(self.restore)
        self.params.param('Acquisition Control','Sensor Readout Mode').sigValueChanged.connect(self.changeReadOutMode)
        self.params.sigTreeStateChanged.connect(self.change)
        """
        # Too lazy for recursion:
        for self.child in self.params.children():
            self.child.sigValueChanging.connect(self.valueChanging)
            for self.ch2 in self.child.children():
                self.ch2.sigValueChanging.connect(self.valueChanging)
        """
        #self.params.param('Acquisition parameter','Width').sigValueChanging.connect(self.valueChanging)
        
        
    ## If anything changes in the tree, print a message
    def change(self,param, changes):
        print("tree changes (PyqtgParamWidget) :")
        for param, change, data in changes:
            path = self.params.childPath(param)
            if path is not None:
                childName = '.'.join(path)
            else:
                childName = param.name()
            print('  parameter: %s'% childName)
            print('  change:    %s'% change)
            print('  data:      %s'% str(data))
            print('  ----------')
            
            
    def changeReadOutMode(self,param, value):
        print("oooooh something's changing !")
    
    def valueChanging(self,param, value):
        print("Value changing (not finalized): %s %s" % (param, value))
        param.setValue(value)
    
        
    def save(self):
        fn = str(pg.QtGui.QFileDialog.getSaveFileName(self, "Save State..", "untitled.cfg", "Config Files (*.cfg)"))
        if fn == '':
            return
        state = self.params.saveState()
        pg.configfile.writeConfigFile(state, fn)
        print("state saved !")
        
    def restore(self):
        global state
        add = self.p['Save/Restore functionality', 'Restore State', 'Add missing items']
        rem = self.p['Save/Restore functionality', 'Restore State', 'Remove extra items']
        self.p.restoreState(state, addChildren=add, removeChildren=rem)




## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    #app = QtGui.QApplication([])
    pg.mkQApp()
    
    ## Switch to using white background and black foreground
    #pg.setConfigOption('background', 'w')
    #pg.setConfigOption('foreground', 'k')
    
    win = QtGui.QWidget()# QtGui.QMainWindow()
    win.resize(1500,800)
    win.setWindowTitle('test parameter tree')
    layout = QtGui.QGridLayout()
    win.setLayout(layout)
    layout.addWidget(QtGui.QLabel("These are two views of the same data. They should always display the same values."), 0,  0, 1, 2)
    
    paramTree=PyqtgParamWidget()
            
    layout.addWidget(paramTree.tree, 1, 0, 1, 1)
          
    win.show()
    
    #ui=PyqtgWindow()
    QtGui.QApplication.instance().exec_()    
    #sys.exit(app.exec_())  
    #app.exec_()
   