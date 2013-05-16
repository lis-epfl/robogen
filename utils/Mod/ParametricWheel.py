from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
from FreeCAD import Base
from PartDesign.Scripts import RadialCopy as rcopy

def makeWheel(	externRadius = 70):
	
	radiusCenterHole = 3
	radiusCenterBody = 9
	heightCenterBody = 6

	thicknessTire = 5
	widthTire = 4

	widthSpoke = externRadius-radiusCenterBody
	lengthSpoke = widthSpoke/9
	heightSpoke = widthTire
	
	radiusScrew = 1
	heightScrew = heightCenterBody

	lengthTeeth = 4
	widthTeeth = lengthTeeth
	heightTeeth = widthTire
	
	#build the structure
	Wheel = Part.makeCylinder(radiusCenterBody,heightCenterBody)
	Wheel.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),30) #needed for fillet below, otherwise the edge29 of one spoke was too close to the edge of the cylinder...
	
	#cut the axis hole
	WheelAxis = Part.makeCylinder(radiusCenterHole,heightCenterBody)
	Wheel = Wheel.cut(WheelAxis)

	# add one spoke
	Spoke = Part.makeBox(lengthSpoke,widthSpoke,heightSpoke)
	Spoke.translate(Base.Vector(-lengthSpoke/2,radiusCenterBody-heightSpoke/2,0))	

	# duplicate each 120 degrees
	radius = 0
	angle = 120
	Spoke = rcopy.makeCopy(Spoke,radius,angle)
	Wheel = Wheel.fuse(Spoke)
	
	# add one hole for the screw
	screw = Part.makeCylinder(radiusScrew,heightScrew)
	screw.translate(Base.Vector(0,3*radiusCenterBody/4,0))
	
	# duplicate each 30 degrees
	radius = 0
	angle = 30
	screw = rcopy.makeCopy(screw,radius,angle)
	Wheel = Wheel.cut(screw)
	
	#add the Tire of the wheel
	Tire = Part.makeCylinder(externRadius,widthTire)
	
	# cut the center part
	WheelCenter = Part.makeCylinder(externRadius-thicknessTire,widthTire)
	Tire = Tire.cut(WheelCenter)

	Wheel = Wheel.fuse(Tire)
	
	# For chamfer you need to know edges	
	edge56=Wheel.Edges[56]
	edge58=Wheel.Edges[58]
	edge59=Wheel.Edges[59]
	edge61=Wheel.Edges[61]
	edge63=Wheel.Edges[63]
	edge64=Wheel.Edges[64]
	edge66=Wheel.Edges[66]
	edge80=Wheel.Edges[80]
	edge84=Wheel.Edges[84]
	edge87=Wheel.Edges[87]
	edge91=Wheel.Edges[91]
	edge93=Wheel.Edges[93]
	edge96=Wheel.Edges[96]

	Wheel=Wheel.makeFillet(1,[edge63])
	Wheel=Wheel.makeFillet(7,[edge56,edge58,edge59,edge61,edge64,edge66])
	Wheel=Wheel.makeFillet(7,[edge80,edge84,edge87,edge91,edge93,edge96])
	
	# add teeth
	Teeth = Part.makeBox(lengthTeeth,widthTeeth,heightTeeth)
	Teeth.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),45)
	Teeth.translate(Base.Vector(externRadius,widthTeeth/2,0))
	
	# duplicate each 120 degrees
	radius = 0
	angle = 10
	Teeth = rcopy.makeCopy(Teeth,radius,angle)
	
	Wheel = Wheel.cut(Teeth)
	
	return Wheel