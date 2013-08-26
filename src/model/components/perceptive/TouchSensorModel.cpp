/*
 * @(#) TouchSensorModel.cpp   1.0   Feb 27, 2013
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
#include "model/components/perceptive/TouchSensorModel.h"

namespace robogen {

const float TouchSensorModel::MASS = inGrams(3);
const float TouchSensorModel::SENSOR_BASE_WIDTH = inMm(34);
const float TouchSensorModel::SENSOR_BASE_THICKNESS = inMm(1.5);
const float TouchSensorModel::SENSOR_THICKNESS = inMm(9);
const float TouchSensorModel::SENSOR_WIDTH = inMm(18.5); // Of each left/right sensors
const float TouchSensorModel::SENSOR_HEIGHT = inMm(16);

TouchSensorModel::TouchSensorModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id) :
		PerceptiveComponent(odeWorld, odeSpace, id) {

}

TouchSensorModel::~TouchSensorModel() {

}

bool TouchSensorModel::initModel() {

	// Create the 4 components of the hinge
	sensorRoot_ = this->createBody(B_SENSOR_BASE_ID);
	dBodyID leftSensor = this->createBody(B_SENSOR_LEFT);
	dBodyID rightSensor = this->createBody(B_SENSOR_RIGHT);

	this->createBoxGeom(sensorRoot_, MASS, osg::Vec3(0, 0, 0),
			SENSOR_BASE_THICKNESS, SENSOR_BASE_WIDTH, SENSOR_BASE_WIDTH);

	dReal xSensors = SENSOR_BASE_THICKNESS / 2 + SENSOR_THICKNESS / 2;
	dReal yLeftSensor = -SENSOR_WIDTH / 2 - inMm(1);
	dReal yRightSensor = SENSOR_WIDTH / 2 + inMm(1);

	dxGeom* leftSensorGeom = this->createBoxGeom(leftSensor, MASS,
			osg::Vec3(xSensors, yLeftSensor, 0), SENSOR_THICKNESS, SENSOR_WIDTH,
			SENSOR_HEIGHT);

	dxGeom* rightSensorGeom = this->createBoxGeom(rightSensor, MASS,
			osg::Vec3(xSensors, yRightSensor, 0), SENSOR_THICKNESS,
			SENSOR_WIDTH, SENSOR_HEIGHT);

	this->sensorLeft_.reset(
			new TouchSensor(this->getCollisionSpace(), leftSensorGeom));
	this->sensorRight_.reset(
			new TouchSensor(this->getCollisionSpace(), rightSensorGeom));

	// Connect everything
	this->fixBodies(sensorRoot_, leftSensor, osg::Vec3(1, 0, 0));
	this->fixBodies(sensorRoot_, rightSensor, osg::Vec3(1, 0, 0));

	return true;

}

dBodyID TouchSensorModel::getRoot() {
	return sensorRoot_;
}

dBodyID TouchSensorModel::getSlot(unsigned int /*i*/) {
	return sensorRoot_;
}

osg::Vec3 TouchSensorModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
		std::cout << "[TouchSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->getPosition(sensorRoot_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SENSOR_BASE_THICKNESS / 2);

	}

	return slotPos;

}

osg::Vec3 TouchSensorModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
		std::cout << "[TouchSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->getAttitude(this->sensorRoot_);
		axis.set(-1, 0, 0);

	}

	return quat * axis;

}

osg::Vec3 TouchSensorModel::getSlotOrientation(unsigned int i) {

	if (i > 1) {
		std::cout << "[TouchSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->getAttitude(this->sensorRoot_);
		axis.set(0, 1, 0);

	}
	return quat * axis;

}

void TouchSensorModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
	sensors.resize(2);
	sensors[0] = this->sensorLeft_;
	sensors[1] = this->sensorRight_;
}

void TouchSensorModel::updateSensors(boost::shared_ptr<Environment>& /*env*/) {
	// Do nothing
}

}
