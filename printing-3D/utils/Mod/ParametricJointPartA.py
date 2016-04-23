from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
from FreeCAD import Base
from PartDesign.Scripts import RadialCopy as rcopy

def makeJoin(	heightJoin = 40, angle = -60):
	
	lengthSlide = 34
	widthSlide = 34
	heightSlide = 1.5
	radiusHole = 1
	lengthJoin = 20
	widthJoin = 9
	
	slide = Part.makeBox(lengthSlide,widthSlide,heightSlide)
	

	slide.translate(Base.Vector(-lengthSlide/2,-widthSlide/2,0))
	
	Join = Part.makeBox(lengthJoin,widthJoin,heightJoin)
	Join2 = Part.makeBox(lengthJoin-4,widthJoin-4,heightJoin-5)
	Join2.translate(Base.Vector(2,2,5))
	Join = Join.cut(Join2)
	Join.translate(Base.Vector(-lengthJoin/2,-widthJoin/2,heightSlide))
	Join.rotate(Base.Vector(0,0,0),Base.Vector(0,0,1),angle)
	
	Line = Part.Line(Base.Vector(0,widthSlide/2,heightSlide),Base.Vector(0,-widthSlide/2,heightSlide))
	
	#holes of the slide
	V9 = Base.Vector(lengthSlide/2-4.5,widthSlide/2-4.5,0)
	V10 = Base.Vector(lengthSlide/2-4.5,-widthSlide/2+4.5,0)
	V11 = Base.Vector(-lengthSlide/2+4.5,-widthSlide/2+4.5,0)
	V12 = Base.Vector(-lengthSlide/2+4.5,widthSlide/2-4.5,0)
	
	heightSlide += 2
	holeFrontRight = Part.makeCylinder(radiusHole,heightSlide,V9,Base.Vector(0,0,1))
	slide = slide.cut(holeFrontRight)
	holeBackRight = Part.makeCylinder(radiusHole,heightSlide,V10,Base.Vector(0,0,1))
	slide = slide.cut(holeBackRight)
	holeBackLeft = Part.makeCylinder(radiusHole,heightSlide,V11,Base.Vector(0,0,1))
	slide = slide.cut(holeBackLeft)
	holeFrontLeft = Part.makeCylinder(radiusHole,heightSlide,V12,Base.Vector(0,0,1))
	slide = slide.cut(holeFrontLeft)
	
	slide=slide.fuse(Join)

	# For chamfer you need to know edges	
	edge0=slide.Edges[0]
	edge1=slide.Edges[1]
	edge2=slide.Edges[2]
	edge3=slide.Edges[3]
	edge4=slide.Edges[4]
	edge5=slide.Edges[5]
	edge6=slide.Edges[6]
	edge7=slide.Edges[7]
	edge8=slide.Edges[8]
	edge9=slide.Edges[9]
	edge10=slide.Edges[10]
	edge11=slide.Edges[11]
	edge12=slide.Edges[12]
	edge13=slide.Edges[13]
	edge14=slide.Edges[14]
	edge15=slide.Edges[15]
	edge16=slide.Edges[16]
	edge17=slide.Edges[17]
	edge18=slide.Edges[18]
	edge19=slide.Edges[19]
	edge20=slide.Edges[20]
	edge21=slide.Edges[21]
	edge22=slide.Edges[22]
	edge23=slide.Edges[23]
	edge24=slide.Edges[24]
	edge25=slide.Edges[25]
	edge26=slide.Edges[26]
	edge27=slide.Edges[27]
	edge28=slide.Edges[28]
	edge29=slide.Edges[29]
	slide=slide.makeChamfer(1.49,[edge1, edge6, edge7,edge8]) #makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	slide=slide.makeChamfer(1,[edge11, edge12, edge13, edge14]) #makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	
	return slide