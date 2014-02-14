from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui, sys
sys.path.append("/home/deniz/Robogen/robogen-simulator/printing-3D/utils/Mod")
import ParametricJointPartA
ParametricJointPartA = reload(ParametricJointPartA)

#JoinA Parameter
heightJoin = 0.04
angle = .543782
Path="/home/deniz/Robogen/robogen-simulator/printing-3D/STL_Files/ParametricJoinPartA1.stl"

#Convert it in mm and degrees respectively
jointLength *= 1000
rotAngle *=180/3.1415926

paramJoinPartA = ParametricJointPartA.makeJoin(jointLength,rotAngle)

Part.show(paramJoinPartA)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartA.exportStl(Path)