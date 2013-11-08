from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui, sys
sys.path.append("D:\Robogen\robogen-simulator-build\robogen-print\robogen-3DPrint\utils\Mod") 
import ParametricJointPartA
ParametricJointPartA = reload(ParametricJointPartA)

#JoinA Parameter
heightJoin = 2  
rotAngle = .543782 
Path="D:\Robogen\robogen-simulator-build\robogen-print\robogen-3DPrint\..\ParametricJoinPartA1.stl" 

#Convert it in mm and degrees respectively
jointLength *= 1000
rotAngle *=180/3.1415926

paramJoinPartA = ParametricJointPartA.makeJoin(jointLength,rotAngle)

Part.show(paramJoinPartA)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartA.exportStl(Path)