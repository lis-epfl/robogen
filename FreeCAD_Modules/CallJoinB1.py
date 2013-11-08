from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui, sys
sys.path.append("D:/Robogen/robogen-simulator-build/robogen-print/robogen-3DPrint/utils/Mod") 
import ParametricJointPartB
ParametricJointPartB=reload(ParametricJointPartB)

# JoinB Parameters
inclAngle = .3 
Path="D:/Robogen/robogen-simulator-build/robogen-print/robogen-3DPrint/../ParametricJoinPartB1.stl" 

#Convert it in degrees
inclAngle *=180/3.1415926

paramJoinPartB = ParametricJointPartB.makeJoinPartB(inclAngle)

Part.show(paramJoinPartB)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartB.exportStl(Path)