/*
 * @(#) ConvexBody.h   1.0   September 16, 2015
 *
 * Basil Huber (basil.huber@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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

#ifndef ROBOGEN_CONVEX_BODY_H_
#define ROBOGEN_CONVEX_BODY_H_

#include "SimpleBody.h"

namespace robogen {

/**
 * A SimpleBody with a dxConvex as geometry.
 */
class ConvexBody : public SimpleBody {

public:
	/**
	 * Constructor
	 * The arrays are copied.
	 * @param model 	 	model that the body belongs to
	 * @param mass 			mass (incl. moment of inertia) of the body
	 * @param pointCount 	number of points of the convex polygon
	 * @param points 		array of corners (vertices) in form: X,Y,Z
	 * @param planeCount 	number of planes (faces) of the convex polygon
	 * @param planes 		array of planes (forming the faces) in form:  normal X, normal Y, normal Z,Distance at the origine
	 * @param polygons 		An array of indices to the points of each polygon,
	 * 						it should be the number of vertices
	 * 						followed by that amount of indices to "points" in counter clockwise order
	 */
	ConvexBody(boost::shared_ptr<Model> model, dMass mass,unsigned int pointCount,
						dReal* points, unsigned int planeCount, dReal* planes, unsigned int* polygons);

	virtual ~ConvexBody();

	inline unsigned int getPointCount() {return pointCount_;}

	inline unsigned int getPlaneCount() {return planeCount_;}

	inline const dReal* getPoints() {return points_;}

	inline const unsigned int* getPolygons(){return polygons_;}

	inline const dReal* getPlanes(){return planes_;}

private:

	/**
	 * Number of points (vertices) of the convex polygon
	 */
	unsigned int pointCount_;
	
	/**
	 * Array of points (vertices) of convex polygon in form: X,Y,Z
	 */
	dReal* points_;

	/**
	 * Number of planes (polygons, faces)
	 */
	unsigned int planeCount_;

	/**
	 * Array of planes in form: normal X, normal Y, normal Z,Distance
	 */
	 dReal* planes_;

	 /**
	 * An array of indices to the points of each polygon,
	 * it should be the number of vertices
	 * followed by that amount of indices to "points" in counter clockwise order
	 */
	 unsigned int* polygons_;

};
}
#endif /* ROBOGEN_CONVEX_BODY_H_ */
