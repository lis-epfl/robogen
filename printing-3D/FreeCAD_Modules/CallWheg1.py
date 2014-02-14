from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui, sys
sys.path.append("/home/deniz/Robogen/robogen-simulator/printing-3D/utils/Mod")
import ParametricWheg
ParametricWheg=reload(ParametricWheg)

# Wheg Parameter
radiusExtern = 0.045
Path="/home/deniz/Robogen/robogen-simulator/printing-3D/STL_Files/ActiveWheg1.stl"

#Convert it in mm
radiusExtern *= 1000

wheg = ParametricWheg.makeWheg(radiusExtern)

Part.show(wheg)
Gui.SendMsgToActiveView("ViewFit")

wheg.exportStl(Path)