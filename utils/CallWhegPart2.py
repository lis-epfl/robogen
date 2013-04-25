
#Convert it in mm
radiusExtern *= 1000

wheg = ParametricWheg.makeWheg(radiusExtern)

Part.show(wheg)
Gui.SendMsgToActiveView("ViewFit")

wheg.exportStl(Path)