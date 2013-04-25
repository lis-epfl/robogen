from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
from FreeCAD import Base
from PartDesign.Scripts import RadialCopy as rcopy

def makeWheg(	externRadius = 70):
	
	radiusCenterHole = 5
	radiusCenterBody = 18

	widthSpoke = externRadius-radiusCenterBody
	
	lengthSpoke = widthSpoke/5
	heightSpoke = 4
	
	radiusScrew = 1
	heightScrew = heightSpoke
	
	#build the structure
	Wheg = Part.makeCylinder(radiusCenterBody,heightSpoke)
	Wheg.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),30) #needed for fillet below, otherwise the edge29 of one spoke was too close to the edge of the cylinder...
	
	#cut the axis hole
	WhegAxis = Part.makeCylinder(radiusCenterHole,heightSpoke)
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
	edge29=Wheg.Edges[29]
	edge32=Wheg.Edges[32]
	edge33=Wheg.Edges[33]
	edge35=Wheg.Edges[35]
	edge36=Wheg.Edges[36]
	edge38=Wheg.Edges[38]
	edge43=Wheg.Edges[43]
	edge48=Wheg.Edges[48]
	edge51=Wheg.Edges[51]
	edge55=Wheg.Edges[55]
	edge58=Wheg.Edges[58]
	edge62=Wheg.Edges[62]

	Wheg=Wheg.makeFillet(14,[edge29,edge32,edge33,edge35,edge36,edge38]) #makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	Wheg=Wheg.makeFillet(widthSpoke/10-1,[edge43,edge48,edge51,edge55,edge58,edge62])
	
	# add one hole for the screw
	screw = Part.makeCylinder(radiusScrew,heightScrew)
	screw.translate(Base.Vector(0,3*radiusCenterBody/4,0))
	
	# duplicate each 30 degrees
	radius = 0
	angle = 30
	screw = rcopy.makeCopy(screw,radius,angle)
	Wheg = Wheg.cut(screw)
	
	return Wheg