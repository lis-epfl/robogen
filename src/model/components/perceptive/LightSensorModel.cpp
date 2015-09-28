/*
 * @(#) LightSensorModel.cpp   1.0   Feb 25, 2013
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
#include "model/components/perceptive/LightSensorModel.h"

namespace robogen {

const float LightSensorModel::MASS = inGrams(2);
const float LightSensorModel::SENSOR_BASE_WIDTH = inMm(34);
const float LightSensorModel::SENSOR_BASE_THICKNESS = inMm(1.5);
const float LightSensorModel::SENSOR_PLATFORM_THICKNESS = inMm(2);
const float LightSensorModel::SENSOR_PLATFORM_WIDTH = inMm(16);

const float LightSensorModel::SENSOR_CYLINDER_HEIGHT = inMm(6);
const float LightSensorModel::SENSOR_CYLINDER_RADIUS = inMm(4);
const float LightSensorModel::SENSOR_DISPLACEMENT = inMm(6);

LightSensorModel::LightSensorModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id,	bool internalSensor) :
		PerceptiveComponent(odeWorld, odeSpace, id),
		internalSensor_(internalSensor) {
}

LightSensorModel::~LightSensorModel() {

}

bool LightSensorModel::initModel() {


	sensorRoot_ = this->createBody(B_SENSOR_BASE_ID);

	//just dividing the mass by 3 for each component, is there a better way to do this?
	this->createBoxGeom(sensorRoot_, MASS/3.0, osg::Vec3(0, 0, 0),
			SENSOR_BASE_THICKNESS, SENSOR_BASE_WIDTH, SENSOR_BASE_WIDTH);

	dBodyID platform = this->createBody(B_SENSOR_PLATFORM_ID);
	this->createBoxGeom(platform, MASS/3.0, osg::Vec3(SENSOR_BASE_THICKNESS/2.0 + SENSOR_PLATFORM_THICKNESS/2.0, 0, 0),
				SENSOR_PLATFORM_THICKNESS, SENSOR_PLATFORM_WIDTH, SENSOR_PLATFORM_WIDTH);

	this->fixBodies(sensorRoot_, platform, osg::Vec3(1, 0, 0));

	dBodyID cylinder = this->createBody(B_SENSOR_CYLINDER_ID);
	this->createCylinderGeom(cylinder, MASS/3.0, osg::Vec3(SENSOR_BASE_THICKNESS/2.0 + SENSOR_PLATFORM_THICKNESS + SENSOR_CYLINDER_HEIGHT/2.0, 0, 0), 1,
				SENSOR_CYLINDER_RADIUS, SENSOR_CYLINDER_HEIGHT);

	this->fixBodies(platform, cylinder, osg::Vec3(1, 0, 0));

	this->sensor_.reset(new LightSensor(this->getCollisionSpace(),
			this->getBodies(), this->getId()));

	return true;

}

bool LightSensorModel::isInternal() {
	return internalSensor_;
}

dBodyID LightSensorModel::getRoot() {
	return sensorRoot_;
}

dBodyID LightSensorModel::getSlot(unsigned int /*i*/) {
	return sensorRoot_;
}

osg::Vec3 LightSensorModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
		std::cout << "[LightSensorModel] Invalid slot: " << i << std::endl;
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

osg::Vec3 LightSensorModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
		std::cout << "[LightSensorModel] Invalid slot: " << i << std::endl;
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

osg::Vec3 LightSensorModel::getSlotOrientation(unsigned int i) {

	if (i > 1) {
		std::cout << "[LightSensorModel] Invalid slot: " << i << std::endl;
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

void LightSensorModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
	sensors.resize(1);
	sensors[0] = sensor_;
}

void LightSensorModel::updateSensors(boost::shared_ptr<Environment>& /*env*/) {

	// Axis
	osg::Quat quat = this->getAttitude(this->sensorRoot_);

	osg::Vec3 curPos = this->getPosition(sensorRoot_);
	osg::Vec3 axis(1, 0, 0);
	osg::Vec3 sensorPos = curPos + quat * axis * SENSOR_DISPLACEMENT;
	this->sensor_->update(sensorPos, this->getAttitude(sensorRoot_));
}

}
