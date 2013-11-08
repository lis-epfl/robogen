from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui, sys
sys.path.append("D:/Robogen/robogen-simulator-build/robogen-print/robogen-3DPrint/utils/Mod") 
import ParametricWheg
ParametricWheg=reload(ParametricWheg)

# Wheg Parameter
radiusExtern = 0.04 
Path="D:/Robogen/robogen-simulator-build/robogen-print/robogen-3DPrint/../ActiveWheg2.stl" 

#Convert it in mm
radiusExtern *= 1000

wheg = ParametricWheg.makeWheg(radiusExtern)

Part.show(wheg)
Gui.SendMsgToActiveView("ViewFit")

wheg.exportStl(Path)