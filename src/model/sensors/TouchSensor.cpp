/*
 * @(#) TouchSensor.cpp   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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

TouchSensor::TouchSensor(dSpaceID odeSpace, dBodyID sensorBody, float mass,
		osg::Vec3 pos, float x, float y, float z, std::string label) :
		collideSpace_(odeSpace), label_(label) {
	// create own space to avoid physical collisions with other objects
	sensorSpace_ = dHashSpaceCreate(0);
	// create Sensor geom in this different space
	dMass massOde;
	dMassSetBoxTotal(&massOde, mass, x, y, z);
	dBodySetMass(sensorBody, &massOde);
	sensorGeometry_ = dCreateBox(sensorSpace_, x, y, z);
	dBodySetPosition(sensorBody, pos.x(), pos.y(), pos.z());
	dGeomSetPosition(sensorGeometry_, pos.x(), pos.y(), pos.z());
	dGeomSetBody(sensorGeometry_, sensorBody);
}

TouchSensor::~TouchSensor() {}

std::string &TouchSensor::getLabel(){
	return label_;
}

bool TouchSensor::read() {
	bool touching = false;
	dSpaceCollide2((dGeomID) collideSpace_, (dGeomID) sensorSpace_,
			(void*) &touching, TouchSensor::collisionCallback);
	return touching;
}

void TouchSensor::collisionCallback(void *data, dGeomID o1, dGeomID o2){
	dContactGeom cont;
	*((bool*)data) = !!dCollide(o1, o2, 1, &cont, sizeof(cont));
}

}
