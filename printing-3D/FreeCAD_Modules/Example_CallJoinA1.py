from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
import ParametricJointPartA
ParametricJointPartA = reload(ParametricJointPartA)

#JoinA Parameter
heightJoin = 0.0227536  
angle = 0.87934595 
Path="C:\Users\lis\Documents\FreeCAD\TestBatch\STL_Files\ParametricJoinPartA1.stl" 

#Convert it in mm and degrees respectively
heightJoin *= 1000
angle *=180/3.1415926

paramJoinPartA = ParametricJointPartA.makeJoin(heightJoin,angle)

Part.show(paramJoinPartA)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartA.exportStl(Path)