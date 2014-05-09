
#Convert it in mm and degrees respectively
heightJoin *= 1000
angle *=180/3.1415926

paramJoinPartA = ParametricJointPartA.makeJoin(heightJoin,angle)

Part.show(paramJoinPartA)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartA.exportStl(Path)
