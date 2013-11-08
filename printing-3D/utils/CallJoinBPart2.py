
#Convert it in degrees
inclAngle *=180/3.1415926

paramJoinPartB = ParametricJointPartB.makeJoinPartB(inclAngle)

Part.show(paramJoinPartB)
Gui.SendMsgToActiveView("ViewFit")

paramJoinPartB.exportStl(Path)