from __future__ import division # allows floating point division from integers
import Part, FreeCAD, math, FreeCADGui
from FreeCAD import Base

def makeJoinPartB(	angle = 40	):
	
	lengthSlide = 34
	widthSlide = 34
	heightSlide = 1.5
	
	lengthWire = 3
	widthWire = 9
	heightWire = heightSlide
	
	lengthBase = 10
	widthBase = 20
	heightBase = 6
	
	radiusCylinderBase = 5
	heightCylinderBase = 20
	
	lengthTopBase = 9
	widthTopBase = 20
	heightTopBase = 5
	
	lengthEndBase = 4
	widthEndBase = 15
	heightEndBase = 10
	
	radiusHole = 1
	
	lengthJoin = 25
	widthJoin = 10
	heightJoin = 40
	
	slide = Part.makeBox(lengthSlide,widthSlide,heightSlide)
	slide.translate(Base.Vector(-lengthSlide/2,-widthSlide/2,0))
	
	#Cut the Wire holes
	WiresHole1 = Part.makeBox(lengthWire,widthWire,heightWire)
	WiresHole1.translate(Base.Vector(-lengthWire/2-10,-widthWire/2,0))
	WiresHole2 = Part.makeBox(lengthWire,widthWire,heightWire)
	WiresHole2.translate(Base.Vector(-lengthWire/2+10,-widthWire/2,0))
	
	slide = slide.cut(WiresHole1)
	slide = slide.cut(WiresHole2)
	
	#holes of the slide
	V9 = Base.Vector(lengthSlide/2-4.5,widthSlide/2-4.5,0)
	V10 = Base.Vector(lengthSlide/2-4.5,-widthSlide/2+4.5,0)
	V11 = Base.Vector(-lengthSlide/2+4.5,-widthSlide/2+4.5,0)
	V12 = Base.Vector(-lengthSlide/2+4.5,widthSlide/2-4.5,0)
	
	#base of the join = Box
	JoinBase = Part.makeBox(lengthBase,widthBase,heightBase)
	JoinBase.translate(Base.Vector(-lengthBase/2,-widthBase/2,heightSlide))
	
	#next strat of join = Cylinder
	JoinCylinder = Part.makeCylinder(radiusCylinderBase,heightCylinderBase)
	JoinCylinder.rotate(Base.Vector(0,0,0),Base.Vector(1,0,0),90)
	JoinCylinder.translate(Base.Vector(0,heightCylinderBase/2,heightSlide+heightBase))
	JoinBase = JoinBase.fuse(JoinCylinder)
	
	#next strat of join = Box over one smaller Box
	JoinTopBase = Part.makeBox(lengthTopBase,widthTopBase,heightTopBase)
	JoinTopBase.translate(Base.Vector(-lengthTopBase/2,-widthTopBase/2,heightSlide+heightBase))
	JoinEndBase = Part.makeBox(lengthEndBase,widthEndBase,heightEndBase)
	JoinEndBase.translate(Base.Vector(-lengthEndBase/2,-widthEndBase/2,heightSlide+heightBase+heightTopBase-1))
	JoinTopBase = JoinTopBase.fuse(JoinEndBase)
	JoinTopBase.rotate(Base.Vector(0,0,heightSlide+heightBase),Base.Vector(0,1,0),angle)
	JoinBase = JoinBase.fuse(JoinTopBase)
	
	slide = slide.fuse(JoinBase)
	
	# cut 4 holes for screwing the slide to core component
	heightSlide += 2
	holeFrontRight = Part.makeCylinder(radiusHole,heightSlide,V9,Base.Vector(0,0,1))
	slide = slide.cut(holeFrontRight)
	holeBackRight = Part.makeCylinder(radiusHole,heightSlide,V10,Base.Vector(0,0,1))
	slide = slide.cut(holeBackRight)
	holeBackLeft = Part.makeCylinder(radiusHole,heightSlide,V11,Base.Vector(0,0,1))
	slide = slide.cut(holeBackLeft)
	holeFrontLeft = Part.makeCylinder(radiusHole,heightSlide,V12,Base.Vector(0,0,1))
	slide = slide.cut(holeFrontLeft)

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
	#slide=slide.makeChamfer(1.49,[edge1])#,edge27,edge28]) #edges of plate! makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	#slide=slide.makeChamfer(1,[edge15,edge16,edge17,edge18,edge19,edge20,edge21,edge22]) #edges of holes! makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	#slide=slide.makeChamfer(1,[edge15,
	slide=slide.makeChamfer(1.49,[edge1,edge6, edge7, edge8])#edges of plate!! makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	slide=slide.makeChamfer(1,[edge15,edge16,edge17,edge18,edge19,edge20,edge21,edge22,edge11,edge12,edge13,edge14])#,edge24,edge25,edge26]) #makeChamfer list of Edges : myBody.Edges or [Edge1,...]
	
	return slide
	
	#Part.show(slide)