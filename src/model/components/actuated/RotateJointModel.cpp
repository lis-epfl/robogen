/*
 * @(#) RotateJointModel.cpp   1.0   Feb 18, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Andrea Maesani, Joshua Auerbach
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
#include "model/components/actuated/RotateJointModel.h"
#include "model/motors/RotationMotor.h"

namespace robogen {

const float RotateJointModel::MASS_SLOT = inGrams(2);
const float RotateJointModel::MASS_SERVO = inGrams(11);
const float RotateJointModel::MASS_CONNECTION_SLOT = inGrams(2);

const float RotateJointModel::SLOT_WIDTH = inMm(34);
const float RotateJointModel::SLOT_THICKNESS = inMm(1.5);
const float RotateJointModel::SERVO_WIDTH = inMm(14);
const float RotateJointModel::SERVO_LENGTH = inMm(36.8);
const float RotateJointModel::SERVO_HEIGHT = inMm(14);
const float RotateJointModel::JOINT_CONNECTION_THICKNESS = inMm(9);
const float RotateJointModel::JOINT_CONNECTION_WIDTH = inMm(34);


RotateJointModel::RotateJointModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id) :
		ActuatedComponent(odeWorld, odeSpace, id) {

}

RotateJointModel::~RotateJointModel() {

}

bool RotateJointModel::initModel() {

	// Create the 2 components of the rotator,
	// now created directly with calls to this->add___

	jointRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_ID);

	dReal xServo = SERVO_LENGTH / 2 + SLOT_THICKNESS / 2;

	boost::shared_ptr<SimpleBody> servo = this->addBox(MASS_SERVO,
		   osg::Vec3(xServo, 0, 0),
		 SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT, B_SERVO_ID);

	dReal xJointConnection = xServo + SERVO_LENGTH / 2
			- JOINT_CONNECTION_THICKNESS / 2 ;
	jointConnection_ = this->addBox(MASS_CONNECTION_SLOT,
			osg::Vec3(xJointConnection, 0, 0), JOINT_CONNECTION_THICKNESS,
			JOINT_CONNECTION_WIDTH, JOINT_CONNECTION_WIDTH,
			B_JOINT_CONNECTION_ID);

	// Create joints to hold pieces in position

	// slot <slider> hinge
	this->fixBodies(jointRoot_, servo);

	// servo <(hinge)> wheel
	boost::shared_ptr<Joint> joint = attachWithHinge(servo, jointConnection_,
			osg::Vec3(1, 0, 0), osg::Vec3(xJointConnection, 0, 0));

	// Create servo
	this->motor_.reset(
			 new RotationMotor(ioPair(this->getId(),0), joint,
					 RotationMotor::DEFAULT_MAX_FORCE_ROTATIONAL));

	return true;

}

boost::shared_ptr<SimpleBody> RotateJointModel::getRoot() {
	return jointRoot_;
}

boost::shared_ptr<SimpleBody> RotateJointModel::getSlot(unsigned int i) {

	if (i > 2) {
		std::cout << "[RotateJointModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	if (i == SLOT_A) {
		return jointRoot_;
	} else {
		return jointConnection_;
	}
}

osg::Vec3 RotateJointModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[RotateJointModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	if (i == SLOT_A) {

		osg::Vec3 curPos = this->jointRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->jointConnection_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (JOINT_CONNECTION_THICKNESS / 2);

	}

}

osg::Vec3 RotateJointModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[RotateJointModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;
	if (i == SLOT_A) {
		quat = this->jointRoot_->getAttitude();
		axis.set(-1, 0, 0);
	} else {
		quat = this->jointConnection_->getAttitude();
		axis.set(1, 0, 0);
	}

	return quat * axis;

}

osg::Vec3 RotateJointModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[RotateJointModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;
	if (i == SLOT_A) {
		quat = this->jointRoot_->getAttitude();
		axis.set(0, 1, 0);
	} else {
		quat = this->jointConnection_->getAttitude();
		axis.set(0, 1, 0);
	}

	return quat * axis;

}

void RotateJointModel::getMotors(
		std::vector<boost::shared_ptr<Motor> >& motors) {
	motors.resize(1);
	motors[0] = this->motor_;
}

}
