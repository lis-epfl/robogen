from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
from FreeCAD import Base
from PartDesign.Scripts import RadialCopy as rcopy

def makeWheg(	externRadius = 70):
	
	radiusCenterHole = 3
	radiusCenterBody = 9
	heightCenterBody = 6

	widthSpoke = externRadius-radiusCenterBody
	
	lengthSpoke = widthSpoke/8
	heightSpoke = 4
	
	radiusScrew = 1
	heightScrew = heightCenterBody
	
	#build the structure
	Wheg = Part.makeCylinder(radiusCenterBody,heightCenterBody)
	Wheg.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),30) #needed for fillet below, otherwise the edge29 of one spoke was too close to the edge of the cylinder...
	
	#cut the axis hole
	WhegAxis = Part.makeCylinder(radiusCenterHole,heightCenterBody)
	Wheg = Wheg.cut(WhegAxis)

	# add one spoke
	Spoke = Part.makeBox(lengthSpoke,widthSpoke,heightSpoke)
	Spoke.translate(Base.Vector(-lengthSpoke/2,radiusCenterBody-heightSpoke/2,0))	

	# duplicate each 120 degrees
	radius = 0
	angle = 120
	Spoke = rcopy.makeCopy(Spoke,radius,angle)
	Wheg = Wheg.fuse(Spoke)

	# For chamfer you need to know edges	
	edge1=Wheg.Edges[1]
	edge4=Wheg.Edges[4]
	edge6=Wheg.Edges[6]
	edge8=Wheg.Edges[8]
	edge10=Wheg.Edges[10]
	edge14=Wheg.Edges[14]
	edge16=Wheg.Edges[16]
	edge28=Wheg.Edges[28]	
	edge32=Wheg.Edges[32]
	edge35=Wheg.Edges[35]
	edge39=Wheg.Edges[39]
	edge42=Wheg.Edges[42]	
	edge46=Wheg.Edges[46]	

	Wheg=Wheg.makeFillet(1,[edge1])
	Wheg=Wheg.makeFillet(7,[edge4,edge6,edge8,edge10,edge14,edge16])
	Wheg=Wheg.makeFillet(lengthSpoke/2-0.1,[edge28,edge32,edge35,edge39,edge42,edge46])
	
	# add one hole for the screw
	screw = Part.makeCylinder(radiusScrew,heightScrew)
	screw.translate(Base.Vector(0,3*radiusCenterBody/4,0))
	
	# duplicate each 30 degrees
	radius = 0
	angle = 360/12
	screw = rcopy.makeCopy(screw,radius,angle)
	Wheg = Wheg.cut(screw)
	
	return Wheg