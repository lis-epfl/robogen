/*
 * @(#) TestComponentModel.cpp   1.0   Feb 8, 2013
 *
 * Basil Huber (basil.huber@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include "test/TestComponentModel.h"

namespace robogen {


TestComponentModel::TestComponentModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id) :
		Model(odeWorld, odeSpace, id)
{
	setColor(Color::WHITE);
}

TestComponentModel::~TestComponentModel() {

}

bool TestComponentModel::initModel() {

	/* ----------------------------------------------
	 *    Example for ConvexPolygon (face = 6)
	 *   Note: 
	 * ----------------------------------------------*/

	const int faceCount  = 6;				// number of faces of the prism
	const int pointCount = 2 * faceCount; 	// number of points (vertices) of the prism
	const int planeCount = faceCount+2;		// number of planes forming the prism
    const double WIDTH = inMm(50);
    const double HEIGHT = inMm(50);
	double LENGTH = 1.73 * WIDTH; 			// length in x = sqrt(3)*WIDTH (WIDTH is the width of a slot)
	osg::Vec3 pos(0,0,0);					// position of the prism
	
    /* create a mass for the ConvexPolygon
     * Currently we use a cylinder as approx of prism
     * TODO: calc exact moment of inertia for prism
     */
    dMass massOde;
    dMassSetCylinderTotal (&massOde, inGrams(100), 3, LENGTH/2, HEIGHT);
   	// To use density instead of mass, use the following function
	//void dMassSetCylinder (&massOde, inGrams(), int direction, dReal radius, dReal length);
    

	// Create arrays for ConvexPolygon
	/* An array of points X,Y,Z */
	dReal points[3*pointCount] = 
             {-LENGTH/2,  WIDTH/2,  -HEIGHT/2,	// point0
              -LENGTH/2, -WIDTH/2, 	-HEIGHT/2,	// point1
          	   		    0, -WIDTH  , 	-HEIGHT/2,	// point2
          	   LENGTH/2, -WIDTH/2, 	-HEIGHT/2,	// point3
          	   LENGTH/2,  WIDTH/2, 	-HEIGHT/2,	// point4
          	 		      0,  WIDTH  , 	-HEIGHT/2,	// point5
              -LENGTH/2,  WIDTH/2,   HEIGHT/2,  // point6
              -LENGTH/2, -WIDTH/2,   HEIGHT/2,  // point7
                      0, -WIDTH  ,   HEIGHT/2,  // point8
               LENGTH/2, -WIDTH/2,   HEIGHT/2,  // point9
               LENGTH/2,  WIDTH/2,   HEIGHT/2,  // point10
                      0,  WIDTH  ,   HEIGHT/2}; // point11
			   
              	
    /* An array of indices to the points of each polygon,
     * it should be the number of vertices followed by 
     * that amount of indices to "points" in counter clockwise order */
    unsigned int polygons[planeCount + faceCount * 4 + 2 * faceCount] = 
    		{4,    0,  1,  7,  6,			// face 0
    		 4,    1,  2,  8,  7,			// face 1
    		 4,    2,  3,  9,  8,    		// face 2
    		 4,    3,  4, 10,  9,  			// face 3
    		 4,    4,  5, 11, 10,			// face 4
    		 4,    5,  0,  6, 11,			// face 5 
    		 6,    5,  4,  3,  2,  1,  0,	// bottom face
    		 6,    6,  7,  8,  9, 10, 11};	// top face

   	// An array of planes in the form: normal X, normal Y, normal Z,Distance
   	dReal planes[4*planeCount];
   	// normal of planes corresponds to slot axis
   	for(int i= 0; i < faceCount; i++)
   	{
   		// calc slot axis (normal to the ith plane)
   		osg::Quat rot;
		rot.makeRotate(osg::DegreesToRadians(i*360.0f/faceCount), osg::Vec3(0, 0, 1));
   		osg::Vec3 slotAxis = rot * osg::Vec3(-1,0,0);
   		planes[4*i]   = slotAxis.x();	// x of normal vector
   		planes[4*i+1] = slotAxis.y();	// y of normal vector
   		planes[4*i+2] = slotAxis.z();	// z of normal vector
   		planes[4*i+3] = LENGTH/2;		// Distance of the plane to the origin
   	}
    // add bottom plane
    planes[4*faceCount]   =  0;   // x of normal vector
    planes[4*faceCount+1] =  0;   // y of normal vector
    planes[4*faceCount+2] = -1;   // z of normal vector
    planes[4*faceCount+3] = HEIGHT/2; // Distance of the plane to the origin
    // add top plane
    planes[4*(faceCount+1)]   =  0;   // x of normal vector
    planes[4*(faceCount+1)+1] =  0;   // y of normal vector
    planes[4*(faceCount+1)+2] =  1;   // z of normal vector
    planes[4*(faceCount+1)+3] = HEIGHT/2; // Distance of the plane to the origin

	rootBody_ = addConvex(massOde, pos, pointCount, points, planeCount, planes, polygons, 0);
  //rootBody_ = addBox(inGrams(50), osg::Vec3(), LENGTH, WIDTH, HEIGHT, 0);
	return true;
}

boost::shared_ptr<SimpleBody> TestComponentModel::getRoot() {
	return rootBody_;
}

boost::shared_ptr<SimpleBody> TestComponentModel::getSlot(unsigned int /*i*/) {
	return rootBody_;
}

osg::Vec3 TestComponentModel::getSlotPosition(unsigned int i) {
	return osg::Vec3();
}

osg::Vec3 TestComponentModel::getSlotAxis(unsigned int i) {
	return osg::Vec3();
}

osg::Vec3 TestComponentModel::getSlotOrientation(unsigned int i) {
	return osg::Vec3();
}

void TestComponentModel::setColor(Color color)
{
	switch(color)
	{
		case Color::RED:
			color_ = osg::Vec4(1,0,0,0.3f);
			break;

		case Color::GREEN:
			color_ = osg::Vec4(0,1,0,0.3f);
			break;

		case Color::BLUE:
			color_ = osg::Vec4(0,0,1,0.3f);
			break;

		case Color::BLACK:
			color_ = osg::Vec4(0,0,0,0.3f);
		  break;

		case Color::WHITE:
			color_ = osg::Vec4(1,1,1,0.3f);
      break;

		default:
			std::cout << "Color not defined: " << (int)color << std::endl;
      break;
	}
}

void TestComponentModel::setColor(osg::Vec4 color)
{
  color_ = color;
}

}