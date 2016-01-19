/*
 * @(#) IrSensorModel.cpp   1.0   Jan 19, 2016
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

#include "model/components/perceptive/IrSensorModel.h"


namespace robogen {

const float IrSensorModel::MASS = inGrams(2); //todo verify!
const float IrSensorModel::SENSOR_BASE_WIDTH = inMm(34);
const float IrSensorModel::SENSOR_BASE_THICKNESS = inMm(1.5);

const float IrSensorModel::SENSOR_PLATFORM_WIDTH = inMm(23.3);
const float IrSensorModel::SENSOR_PLATFORM_HEIGHT = inMm(23);
const float IrSensorModel::SENSOR_PLATFORM_THICKNESS = inMm(5.5);

const float IrSensorModel::SENSOR_DISPLACEMENT = inMm(6);

IrSensorModel::IrSensorModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id) :
		PerceptiveComponent(odeWorld, odeSpace, id) {
}

IrSensorModel::~IrSensorModel() {

}

bool IrSensorModel::initModel() {


	//just dividing the mass by 2 for each component, is there a better way to do this?
	sensorRoot_ = this->addBox(MASS/2.0, osg::Vec3(0, 0, 0),
			SENSOR_BASE_THICKNESS, SENSOR_BASE_WIDTH, SENSOR_BASE_WIDTH,
			B_SENSOR_BASE_ID);

	boost::shared_ptr<SimpleBody> platform = this->addBox(MASS/2.0,
			osg::Vec3(SENSOR_BASE_THICKNESS/2.0 +
					  SENSOR_PLATFORM_THICKNESS/2.0, 0, 0),
			SENSOR_PLATFORM_THICKNESS, SENSOR_PLATFORM_WIDTH,
			SENSOR_PLATFORM_HEIGHT, B_SENSOR_PLATFORM_ID);

	this->fixBodies(sensorRoot_, platform);

	this->sensor_.reset(new IrSensor(this->getCollisionSpace(),
			this->getBodies(), this->getId()));

	return true;

}

boost::shared_ptr<SimpleBody> IrSensorModel::getRoot() {
	return sensorRoot_;
}

boost::shared_ptr<SimpleBody> IrSensorModel::getSlot(unsigned int /*i*/) {
	return sensorRoot_;
}

osg::Vec3 IrSensorModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
		std::cout << "[IrSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->sensorRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SENSOR_BASE_THICKNESS / 2);

	}

	return slotPos;

}

osg::Vec3 IrSensorModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
		std::cout << "[IrSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->sensorRoot_->getAttitude();
		axis.set(-1, 0, 0);

	}

	return quat * axis;

}

osg::Vec3 IrSensorModel::getSlotOrientation(unsigned int i) {

	if (i > 1) {
		std::cout << "[IrSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->sensorRoot_->getAttitude();
		axis.set(0, 1, 0);

	}
	return quat * axis;

}

void IrSensorModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
	sensor_->getSensors(sensors);
}

void IrSensorModel::updateSensors(boost::shared_ptr<Environment>& env) {

	// Axis
	osg::Quat quat = this->sensorRoot_->getAttitude();

	osg::Vec3 curPos = this->sensorRoot_->getPosition();
	osg::Vec3 axis(1, 0, 0);
	osg::Vec3 sensorPos = curPos + quat * axis * SENSOR_DISPLACEMENT;
	this->sensor_->update(sensorPos, this->sensorRoot_->getAttitude());
}

}
