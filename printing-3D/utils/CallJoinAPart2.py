
#Convert it in mm and degrees respectively
jointLength *= 1000
rotAngle *=180/3.1415926

paramJoinPartA = ParametricJointPartA.makeJoin(jointLength,rotAngle)

Part.show(paramJoinPartA)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartA.exportStl(Path)