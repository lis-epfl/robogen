from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
import ParametricJointPartB
ParametricJointPartB=reload(ParametricJointPartB)

# JoinB Parameters
angle = -0.10105443 
Path="C:\Users\lis\Documents\FreeCAD\TestBatch\STL_Files\ParametricJoinPartB1.stl" 

#Convert it in degrees
angle *=180/3.1415926

paramJoinPartB = ParametricJointPartB.makeJoinPartB(angle)

Part.show(paramJoinPartB)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartB.exportStl(Path)