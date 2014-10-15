from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
from FreeCAD import Base
from PartDesign.Scripts import RadialCopy as rcopy

def makeWheel(	externRadius = 70, test = 0):
	
	radiusCenterHole = 1.5
	radiusCenterBody = 9
	heightCenterBody = 4

	radiusMountingPart = 4	
	lengthMountingPart = radiusCenterBody
	heightMountingPart = 5

	thicknessTire = 5
	widthTire = 4

	widthSpoke = externRadius-radiusCenterBody
	lengthSpoke = widthSpoke/9
	heightSpoke = widthTire
	
	radiusScrew = 1.5
	heightScrew = radiusCenterBody

	lengthTeeth = 4
	widthTeeth = lengthTeeth
	heightTeeth = widthTire
	
	#build the structure
	Wheel = Part.makeCylinder(radiusCenterBody,heightCenterBody)
	Wheel.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),30) #needed for fillet below, otherwise the edge29 of one spoke was too close to the edge of the cylinder...

	MountingPart = Part.makeCylinder(radiusMountingPart,heightMountingPart)
	MountingPart.translate(Base.Vector(0,0,heightCenterBody))
	MountingPart2 = Part.makeBox(radiusCenterBody,radiusMountingPart*2,heightMountingPart)
	MountingPart2.translate(Base.Vector(0,-radiusMountingPart,heightCenterBody))
	MountingPart2.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),30)	

	MountingPart = MountingPart.fuse(MountingPart2)
	Wheel = Wheel.fuse(MountingPart)

	#cut the axis hole
	WheelAxis = Part.makeCylinder(radiusCenterHole,heightCenterBody+heightMountingPart)
	Wheel = Wheel.cut(WheelAxis)

	# add one hole for the screw
	screw = Part.makeCylinder(radiusScrew,heightScrew)
	screw.rotate(Base.Vector(0,0,0),Base.Vector(0,1,0),90)	
	screw.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),30)
	screw.translate(Base.Vector(0,0,heightCenterBody+heightMountingPart/2))
	Wheel = Wheel.cut(screw)

	# For chamfer you need to know edges	
	edge1 = Wheel.Edges[1]
	edge5 = Wheel.Edges[5]
	edge25 = Wheel.Edges[25]
	edge27 = Wheel.Edges[27]
	edge8 = Wheel.Edges[test]
	Wheel=Wheel.makeFillet(1,[edge1,edge5,edge8,edge25,edge27])
	
	# add one spoke
	Spoke = Part.makeBox(lengthSpoke,widthSpoke,heightSpoke)
	Spoke.translate(Base.Vector(-lengthSpoke/2,radiusCenterBody-heightSpoke/2,0))	

	# duplicate each 120 degrees
	radius = 0
	angle = 120
	Spoke = rcopy.makeCopy(Spoke,radius,angle)
	Wheel = Wheel.fuse(Spoke)
	
	#add the Tire of the wheel
	Tire = Part.makeCylinder(externRadius,widthTire)
	
	# cut the center part
	WheelCenter = Part.makeCylinder(externRadius-thicknessTire,widthTire)
	Tire = Tire.cut(WheelCenter)

	Wheel = Wheel.fuse(Tire)
	
	# For chamfer you need to know edges		
#	edgeTest=Wheel.Edges[test]
#	edge19=Wheel.Edges[19]
#	edge21=Wheel.Edges[21]
#	edge25=Wheel.Edges[25]
#	edge30=Wheel.Edges[30]
#	edge33=Wheel.Edges[33]
#	edge50=Wheel.Edges[50]
#	edge57=Wheel.Edges[57]
#	edge75=Wheel.Edges[75]
#	edge80=Wheel.Edges[80]
#	edge81=Wheel.Edges[81]
#	edge85=Wheel.Edges[85]
#	edge86=Wheel.Edges[86]
#	edge89=Wheel.Edges[89]
#	edge90=Wheel.Edges[90]
#
#	Wheel=Wheel.makeFillet(1,[edge19,edge21,edge25,edge30,edge50])
#	Wheel=Wheel.makeFillet(5,[edge57,edge75,edge80,edge81,edge85,edge86,edge89,edge90])
	#Wheel=Wheel.makeFillet(7,[edge80,edge84,edge87,edge91,edge93,edge96])
	
	# add teeth
	Teeth = Part.makeBox(lengthTeeth,widthTeeth,heightTeeth)
	Teeth.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),45)
	Teeth.translate(Base.Vector(externRadius,widthTeeth/2,0))
	
	# duplicate each 120 degrees
	radius = 0
	angle = 10
	Teeth = rcopy.makeCopy(Teeth,radius,angle)
	
	Wheel = Wheel.cut(Teeth)

	#add new joints
	

	return Wheel