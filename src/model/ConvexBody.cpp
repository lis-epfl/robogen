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

#include "Model.h"
#include "ConvexBody.h"

namespace robogen {

ConvexBody::ConvexBody(boost::shared_ptr<Model> model, dMass mass,unsigned int pointCount,
						dReal* points, unsigned int planeCount, dReal* planes, unsigned int* polygons) :
	SimpleBody( model,
				mass,
				dCreateConvex(model->getCollisionSpace(), planes, planeCount, points, pointCount, polygons),
				osg::Vec3()),
	pointCount_(pointCount),
	planeCount_(planeCount)
{
	/* copy arrays */
	points_ = new dReal[3*pointCount_];
	memcpy(points_, points, 3*pointCount_ * sizeof(dReal));

	planes_ = new dReal[4*planeCount_];
	memcpy(planes_, planes, 4*planeCount_ * sizeof(dReal));

	// find length of polygons array
	unsigned int length = 0;
	for(unsigned int i = 0; i < planeCount_; i++){
		length += polygons[length]+1;
	}
	polygons_ = new unsigned int[length];
	memcpy(polygons_, polygons, length * sizeof(unsigned int));

	// set correct pointers to buffers stored in this class
	dGeomSetConvex(getGeom(), planes_, planeCount_, points_, pointCount_, polygons_);
}


ConvexBody::~ConvexBody(){
	delete [] points_;
	delete [] planes_;
	delete [] polygons_;
}
}
