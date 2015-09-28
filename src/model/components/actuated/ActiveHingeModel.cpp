/*
 * @(#) ActiveHingeModel.cpp   1.0   Feb 12, 2013
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
#include "model/components/actuated/ActiveHingeModel.h"
#include "model/motors/ServoMotor.h"

namespace robogen {

const float ActiveHingeModel::MASS_SERVO = inGrams(13);//inGrams(9);
const float ActiveHingeModel::MASS_SLOT = inGrams(2);//inGrams(7);
const float ActiveHingeModel::MASS_FRAME = inGrams(1);////inGrams(1.2);

const float ActiveHingeModel::SLOT_WIDTH = inMm(34);
const float ActiveHingeModel::SLOT_THICKNESS = inMm(1.5);

const float ActiveHingeModel::FRAME_LENGTH = inMm(20.5);//inMm(18);
const float ActiveHingeModel::FRAME_HEIGHT = inMm(13.0);//inMm(10);
const float ActiveHingeModel::FRAME_WIDTH = inMm(36);
const float ActiveHingeModel::FRAME_ROTATION_OFFSET = inMm(14); // Left to right

const float ActiveHingeModel::SERVO_LENGTH = inMm(27.5);//inMm(32);//inMm(24.5);
const float ActiveHingeModel::SERVO_HEIGHT = inMm(16.25);//inMm(10);
const float ActiveHingeModel::SERVO_WIDTH = inMm(36.5);
const float ActiveHingeModel::SERVO_ROTATION_OFFSET = inMm(21.5);//inMm(20.5); // Right to left
const float ActiveHingeModel::SERVO_POSITION_OFFSET = inMm(0.5);

ActiveHingeModel::ActiveHingeModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id) :
		ActuatedComponent(odeWorld, odeSpace, id) {

}

ActiveHingeModel::~ActiveHingeModel() {

}

bool ActiveHingeModel::initModel() {

	// Create the 4 components of the hinge
	hingeRoot_ = this->createBody(B_SLOT_A_ID);
	dBodyID frame = this->createBody(B_FRAME_ID);
	dBodyID servo = this->createBody(B_SERVO_ID);
	hingeTail_ = this->createBody(B_SLOT_B_ID);

	// Set the masses for the various boxes

	float separation = 0;//inMm(0.1);

	this->createBoxGeom(hingeRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	dReal xFrame = separation + FRAME_LENGTH / 2 + SLOT_THICKNESS / 2;
	this->createBoxGeom(frame, MASS_FRAME, osg::Vec3(xFrame,
			SERVO_POSITION_OFFSET, 0),
			FRAME_LENGTH, FRAME_WIDTH, FRAME_HEIGHT);

	dReal xServo = xFrame + (FRAME_ROTATION_OFFSET - (FRAME_LENGTH / 2))
			+ SERVO_ROTATION_OFFSET - SERVO_LENGTH/2;
	this->createBoxGeom(servo, MASS_SERVO, osg::Vec3(xServo,
			SERVO_POSITION_OFFSET, 0),
			SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT);

	dReal xTail = xServo + SERVO_LENGTH / 2 + separation + SLOT_THICKNESS / 2;
	this->createBoxGeom(hingeTail_, MASS_SLOT, osg::Vec3(xTail, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	// Create joints to hold pieces in position

	// root <-> connectionPartA
	this->fixBodies(hingeRoot_, frame, osg::Vec3(1, 0, 0));

	// connectionPartA <(hinge)> connectionPArtB
	dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
	dJointAttach(joint, frame, servo);
	dJointSetHingeAxis(joint, 0, 1, 0);
	dJointSetHingeAnchor(joint,
			SLOT_THICKNESS / 2 + separation + FRAME_ROTATION_OFFSET, 0, 0);

	// connectionPartB <-> tail
	this->fixBodies(servo, hingeTail_, osg::Vec3(1, 0, 0));

	// Create servo
	this->motor_.reset(
			new ServoMotor(joint, ServoMotor::DEFAULT_MAX_FORCE_SERVO,
					ServoMotor::DEFAULT_GAIN,
					ioPair(this->getId(),0)));

	return true;

}

dBodyID ActiveHingeModel::getRoot() {
	return hingeRoot_;
}

dBodyID ActiveHingeModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return hingeRoot_;
	} else {
		return hingeTail_;
	}
}

osg::Vec3 ActiveHingeModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->getPosition(hingeRoot_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->getPosition(hingeTail_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	}
	return slotPos;

}

osg::Vec3 ActiveHingeModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->getAttitude(this->hingeRoot_);
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->hingeTail_);
		axis.set(1, 0, 0);

	}

	return quat * axis;

}

osg::Vec3 ActiveHingeModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->getAttitude(this->hingeRoot_);
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->hingeTail_);
		axis.set(0, 1, 0);

	}

	return quat * axis;

}

void ActiveHingeModel::getMotors(
		std::vector<boost::shared_ptr<Motor> >& motors) {
	motors.resize(1);
	motors[0] = this->motor_;
}

}
