/*
 * @(#) RotateJointModel.cpp   1.0   Feb 18, 2013
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
#include "model/components/actuated/RotateJointModel.h"
#include "model/motors/ServoMotor.h"

namespace robogen {

const float RotateJointModel::MASS_SLOT = inGrams(2);
const float RotateJointModel::MASS_SERVO = inGrams(7);
const float RotateJointModel::MASS_CONNECTION_SLOT = inGrams(3);

const float RotateJointModel::SLOT_WIDTH = inMm(34);
const float RotateJointModel::SLOT_THICKNESS = inMm(1.5);
const float RotateJointModel::SERVO_Z_OFFSET = inMm(9); // zCenter shift respect to slot z-center
const float RotateJointModel::SERVO_WIDTH = inMm(10);
const float RotateJointModel::SERVO_LENGTH = inMm(29);
const float RotateJointModel::SERVO_HEIGHT = inMm(28);
const float RotateJointModel::JOINT_CONNECTION_THICKNESS = inMm(7.5);
const float RotateJointModel::JOINT_CONNECTION_WIDTH = inMm(34);

RotateJointModel::RotateJointModel(dWorldID odeWorld, dSpaceID odeSpace) :
		ActuatedComponent(odeWorld, odeSpace) {

}

RotateJointModel::~RotateJointModel() {

}

bool RotateJointModel::initModel() {

	// Create the 4 components of the hinge
	jointRoot_ = this->createBody(B_SLOT_ID);
	dBodyID servo = this->createBody(B_SERVO_ID);
	jointConnection_ = this->createBody(B_JOINT_CONNECTION_ID);

	// Set the masses for the various boxes
	dMass mass;

	dMassSetBoxTotal(&mass, MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);
	dBodySetMass(jointRoot_, &mass);

	dMassSetBoxTotal(&mass, MASS_SERVO, SERVO_LENGTH, SERVO_WIDTH,
			SERVO_HEIGHT);
	dBodySetMass(servo, &mass);

	dMassSetBoxTotal(&mass, MASS_CONNECTION_SLOT, JOINT_CONNECTION_THICKNESS,
			JOINT_CONNECTION_WIDTH, JOINT_CONNECTION_WIDTH);
	dBodySetMass(jointConnection_, &mass);

	float separation = inMm(0.1);

	this->createBoxGeom(jointRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	dReal xServo = SLOT_THICKNESS / 2 + separation + SERVO_LENGTH / 2;
	dReal zServo = -SLOT_WIDTH / 2 + SERVO_Z_OFFSET + SERVO_HEIGHT / 2;
	this->createBoxGeom(servo, MASS_SERVO, osg::Vec3(xServo, 0, zServo),
			SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT);

	dReal xJointConnection = xServo + SERVO_LENGTH / 2 + separation
			+ JOINT_CONNECTION_THICKNESS / 2;
	this->createBoxGeom(jointConnection_, MASS_CONNECTION_SLOT,
			osg::Vec3(xJointConnection, 0, 0), JOINT_CONNECTION_THICKNESS,
			JOINT_CONNECTION_WIDTH, JOINT_CONNECTION_WIDTH);

	// Create joints to hold pieces in position

	// slot <slider> hinge
	this->fixBodies(jointRoot_, servo, osg::Vec3(1, 0, 0));

	// servo <(hinge)> wheel
	dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
	dJointAttach(joint, servo, jointConnection_);
	dJointSetHingeAxis(joint, 1, 0, 0);
	dJointSetHingeAnchor(joint, xJointConnection, 0, 0);

	// Create servo
	this->motor_.reset(
			new ServoMotor(joint, ServoMotor::DEFAULT_MAX_FORCE,
					ServoMotor::DEFAULT_GAIN));

	return true;

}

dBodyID RotateJointModel::getRoot() {
	return jointRoot_;
}

dBodyID RotateJointModel::getSlot(unsigned int i) {

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

		osg::Vec3 curPos = this->getPosition(jointRoot_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->getPosition(jointConnection_);
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
		quat = this->getAttitude(this->jointRoot_);
		axis.set(-1, 0, 0);
	} else {
		quat = this->getAttitude(this->jointConnection_);
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
		quat = this->getAttitude(this->jointRoot_);
		axis.set(0, 1, 0);
	} else {
		quat = this->getAttitude(this->jointConnection_);
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
