
#Convert it in mm
radiusExtern *= 1000

wheel = ParametricWheel.makeWheel(radiusExtern)

Part.show(wheel)
Gui.SendMsgToActiveView("ViewFit")

wheel.exportStl(Path)