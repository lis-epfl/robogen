/*
 * @(#) TouchSensor.cpp   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
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
#include "model/sensors/TouchSensor.h"
#include <map>
#include <osg/Vec3>
#include <iostream>

namespace robogen {

TouchSensor::TouchSensor(dSpaceID odeSpace, boost::shared_ptr<SimpleBody> body,
		std::string label) : Sensor(label), collideSpace_(odeSpace),
				body_(body) {
}

TouchSensor::~TouchSensor() {
}

struct TouchData {
	boost::shared_ptr<SimpleBody> body;
	bool touching;
};

void TouchSensor::update() {

	// make a temporary space for checking touches
	dSpaceID sensorSpace = dHashSpaceCreate(0);
	dVector3 boxLengths;
	dGeomBoxGetLengths(body_->getGeom(), boxLengths);

	dGeomID sensorGeometry = dCreateBox(sensorSpace, boxLengths[0],
			boxLengths[1], boxLengths[2]);
	osg::Vec3 pos = body_->getPosition();
	dGeomSetPosition(sensorGeometry, pos.x(), pos.y(), pos.z());


	TouchData data;
	data.touching = false;
	data.body = body_;
	dSpaceCollide2((dGeomID) collideSpace_, (dGeomID) sensorSpace,
			(void*) &data, TouchSensor::collisionCallback);
	// clean up, will automatically delete geom as well
	dSpaceDestroy(sensorSpace);

	updateValue( data.touching );
}

void TouchSensor::collisionCallback(void *data, dGeomID o1, dGeomID o2){
	dContactGeom cont;
	TouchData *touchData = ((TouchData*) data);

	// ignore collisions with any other geom that is part of this body
	// this will be the paired touch sensor as well as the touch sensor base
	dBodyID b = touchData->body->getBody();
	dGeomID g = dBodyGetFirstGeom(b);
	while(g) {
		if (o1 ==  g || o2 == g) {
			return;
		}
		g = dBodyGetNextGeom(g);
	}

	touchData->touching = !!dCollide(o1, o2, 1, &cont, sizeof(cont));

}

}
