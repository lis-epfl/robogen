
#Convert it in degrees
angle *=180/3.1415926

paramJoinPartB = ParametricJointPartB.makeJoinPartB(angle)

Part.show(paramJoinPartB)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartB.exportStl(Path)