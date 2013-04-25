from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
from FreeCAD import Base
from PartDesign.Scripts import RadialCopy as rcopy

def makeWheel(	radiusExtern = 70	):
	
	radiusCenterHole = 5
	radiusCenterBody = 18
	
	thicknessTire = 6
	widthTire = 4
	
	lengthSpoke = 4
	widthSpoke = radiusExtern-radiusCenterBody-widthTire/2
	heightSpoke = widthTire
	
	radiusScrew = 1
	heightScrew = widthTire
	
	lengthTeeth = 3
	widthTeeth = lengthTeeth
	heightTeeth = widthTire
	
	Tire = Part.makeCylinder(radiusExtern,widthTire)
	
	# cut the center part
	WheelCenter = Part.makeCylinder(radiusExtern-thicknessTire,widthTire)
	Tire = Tire.cut(WheelCenter)
	
	#build the structure
	WheelStrucure = Part.makeCylinder(radiusCenterBody,widthTire)
	Tire = Tire.fuse(WheelStrucure)
	
	#cut the axis hole
	WheelAxis = Part.makeCylinder(radiusCenterHole,widthTire)
	Tire = Tire.cut(WheelAxis)
	
	# add one spoke
	Spoke = Part.makeBox(lengthSpoke,widthSpoke,heightSpoke)
	Spoke.translate(Base.Vector(-lengthSpoke/2,radiusCenterBody-widthTire/2,0))
	
	# duplicate each 120 degrees
	radius = 0
	angle = 120
	Spoke = rcopy.makeCopy(Spoke,radius,angle)
	Tire = Tire.fuse(Spoke)

	# For chamfer you need to know edges	
	edge29=Tire.Edges[29]
	edge32=Tire.Edges[32]
	edge33=Tire.Edges[33]
	edge35=Tire.Edges[35]
	edge36=Tire.Edges[36]
	edge38=Tire.Edges[38]
	edge44=Tire.Edges[44]
	edge49=Tire.Edges[49]
	edge51=Tire.Edges[51]
	edge56=Tire.Edges[56]
	edge58=Tire.Edges[58]
	edge62=Tire.Edges[62]
	
	#Tire=Tire.makeFillet(radiusExtern/7,[edge29,edge32,edge33,edge35,edge36,edge38,edge44,edge49,edge51,edge56,edge58,edge62]) #makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	
	# add one hole for the screw
	screw = Part.makeCylinder(radiusScrew,heightScrew)
	screw.translate(Base.Vector(0,3*radiusCenterBody/4,0))
	
	# duplicate each 30 degrees
	radius = 0
	angle = 30
	screw = rcopy.makeCopy(screw,radius,angle)
	Tire = Tire.cut(screw)
	
	# add teeth
	Teeth = Part.makeBox(lengthTeeth,widthTeeth,heightTeeth)
	Teeth.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),45)
	Teeth.translate(Base.Vector(radiusExtern,widthTeeth/2,0))
	
	# duplicate each 120 degrees
	radius = 0
	angle = 5
	Teeth = rcopy.makeCopy(Teeth,radius,angle)
	Tire = Tire.cut(Teeth)

	return Tire