from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
import ParametricWheg
ParametricWheg=reload(ParametricWheg)

# Wheg Parameter
radiusExtern = 0.079175949 
Path="C:\Users\lis\Documents\FreeCAD\TestBatch\STL_Files\ActiveWheg1.stl" 

#Convert it in mm
radiusExtern *= 1000

wheg = ParametricWheg.makeWheg(radiusExtern)

Part.show(wheg)
Gui.SendMsgToActiveView("ViewFit")

wheg.exportStl(Path)