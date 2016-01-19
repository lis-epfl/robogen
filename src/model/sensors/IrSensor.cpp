/*
 * @(#) IrSensor.cpp   1.0   Oct 27, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
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

#include "IrSensor.h"

#include <algorithm>
#include <iostream>
#include <cmath>

namespace robogen {

const float IrSensor::SENSOR_RANGE = 0.255;

IrSensor::IrSensor(dSpaceID odeSpace,
		std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
		std::string baseLabel):odeSpace_(odeSpace), lastReadOutput_(0) {

	for(unsigned int i=0; i<sensorBodies.size(); i++){
		for (dGeomID g=dBodyGetFirstGeom(sensorBodies[i]->getBody()); g
				; g=dBodyGetNextGeom(g)){
			sensorGeoms_.push_back(g);
		}
	}
	raySpace_ = dHashSpaceCreate(0);


	sensors_.push_back(
		boost::shared_ptr<IrSensorElement>(new IrSensorElement(baseLabel,
				IrSensorElement::IR)));
	// no ambient light sensing here for now
	/*sensors_.push_back(
			boost::shared_ptr<IrSensorElement>(new IrSensorElement(baseLabel,
				IrSensorElement::AMBIENT_LIGHT)));
	 */
}

IrSensor::~IrSensor() {
	dSpaceDestroy(raySpace_);
}



struct RayTrace {
	std::vector<dGeomID> ignoreGeoms;
	bool isColliding;
	osg::Vec3 collisionPoint;

};

void IrSensor::collisionCallback(void *data, dGeomID o1, dGeomID o2){
	RayTrace *r = (RayTrace*) data;
	// ignore what needs ignoring
	if (std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
			o1) != r->ignoreGeoms.end() ||
			std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
					o2) != r->ignoreGeoms.end()){
		return;
	}
	// if colliding get collision point
	dContact contact;

	if (dCollide(o1,o2,1,&contact.geom,sizeof(contact))) {
		r->collisionPoint =  osg::Vec3(contact.geom.pos[0],
				contact.geom.pos[1],
				contact.geom.pos[2]);
		r->isColliding = true;
	}
}

void IrSensor::update(const osg::Vec3& position, const osg::Quat& attitude) {
	this->position_ = position;
	this->attitude_ = attitude;

	osg::Vec3 rayVector = attitude_ *  osg::Vec3(1,0,0);

	// create ray
	dGeomID ray = dCreateRay(raySpace_, SENSOR_RANGE);
	// position ray
	dGeomRaySet(ray, position_.x(), position_.y(), position_.z(),
			rayVector.x(), rayVector.y(), rayVector.z());

	// prepare collision data structure
	RayTrace data;

	// ignore sensor bodies
	data.ignoreGeoms.insert(data.ignoreGeoms.end(),
					sensorGeoms_.begin(), sensorGeoms_.end());

	data.isColliding = false;

	// perform collision
	dSpaceCollide2((dGeomID)raySpace_, (dGeomID)odeSpace_, (void*)&data,
					IrSensor::collisionCallback);

	float distance = SENSOR_RANGE;

	// ray should be capped at SENSOR_RANGE, but just to make sure we don't
	// allow bigger values here
	if(data.isColliding &&
			(data.collisionPoint - position_).length() < SENSOR_RANGE) {
		distance = (data.collisionPoint - position_).length();

		//std::cout << "pos: " << position_.x() << " " << position_.y() <<  " " << position_.z() << std::endl;
		//std::cout << "ray: " << rayVector.x() << " " << rayVector.y() <<  " " << rayVector.z() << std::endl;
		//std::cout << "collision: " << data.collisionPoint.x() << " " << data.collisionPoint.y() <<  " " << data.collisionPoint.z() << std::endl;

	}

	//std::cout << distance << std::endl;

	dGeomDestroy(ray);
	// want 0 when nothing is seen
	sensors_[0]->updateValue( 1.0 - (distance / SENSOR_RANGE) );

	// no ambient light sensing here for now
	//sensors_[1]->updateValue( 0.0 );
}


}



