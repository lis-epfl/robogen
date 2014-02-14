from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui, sys
sys.path.append("/home/deniz/Robogen/robogen-simulator/printing-3D/utils/Mod")
import ParametricJointPartB
ParametricJointPartB=reload(ParametricJointPartB)

# JoinB Parameters
angle = .3
Path="/home/deniz/Robogen/robogen-simulator/printing-3D/STL_Files/ParametricJoinPartB1.stl"

#Convert it in degrees
inclAngle *=180/3.1415926

paramJoinPartB = ParametricJointPartB.makeJoinPartB(inclAngle)

Part.show(paramJoinPartB)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartB.exportStl(Path)