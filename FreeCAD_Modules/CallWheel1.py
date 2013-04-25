from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
import ParametricWheel
ParametricWheel=reload(ParametricWheel) 

# Wheel Parameter
radiusExtern = 0.071092308 
Path="C:\Users\lis\Documents\FreeCAD\TestBatch\STL_Files\PassiveWheel1.stl" 

#Convert it in mm
radiusExtern *= 1000

wheel = ParametricWheel.makeWheel(radiusExtern)

Part.show(wheel)
Gui.SendMsgToActiveView("ViewFit")

wheel.exportStl(Path)