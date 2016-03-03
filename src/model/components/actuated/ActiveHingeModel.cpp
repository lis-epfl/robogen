/*
 * @(#) ActiveHingeModel.cpp   1.0   Feb 12, 2013
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
	// now created directly with calls to this->add___

	hingeRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_A_ID);

	dReal xFrame = FRAME_LENGTH / 2 + SLOT_THICKNESS / 2;
	boost::shared_ptr<SimpleBody> frame = this->addBox(MASS_FRAME,
			osg::Vec3(xFrame,SERVO_POSITION_OFFSET, 0),
			FRAME_LENGTH, FRAME_WIDTH, FRAME_HEIGHT, B_FRAME_ID);

	dReal xServo = xFrame + (FRAME_ROTATION_OFFSET - (FRAME_LENGTH / 2))
			+ SERVO_ROTATION_OFFSET - SERVO_LENGTH/2;
	boost::shared_ptr<SimpleBody> servo = this->addBox(MASS_SERVO,
			osg::Vec3(xServo,SERVO_POSITION_OFFSET, 0),
			SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT, B_SERVO_ID);

	dReal xTail = xServo + SERVO_LENGTH / 2 + SLOT_THICKNESS / 2;
	hingeTail_ = this->addBox(MASS_SLOT, osg::Vec3(xTail, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_B_ID);

	// Create joints to hold pieces in position

	// root <-> connectionPartA
	this->fixBodies(hingeRoot_, frame);

	// connectionPartA <(hinge)> connectionPArtB
	boost::shared_ptr<Joint> joint = this->attachWithHinge(frame, servo,
			osg::Vec3(0, 1, 0),
			osg::Vec3(SLOT_THICKNESS / 2 + FRAME_ROTATION_OFFSET,
						0, 0));

	// connectionPartB <-> tail
	this->fixBodies(servo, hingeTail_);



	// Create servo
	this->motor_.reset(
			new ServoMotor(ioPair(this->getId(),0), joint,
							ServoMotor::DEFAULT_MAX_FORCE_SERVO,
							ServoMotor::DEFAULT_GAIN));


	return true;

}

boost::shared_ptr<SimpleBody> ActiveHingeModel::getRoot() {
	return hingeRoot_;
}

boost::shared_ptr<SimpleBody> ActiveHingeModel::getSlot(unsigned int i) {
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
		osg::Vec3 curPos = hingeRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);
	} else {
		osg::Vec3 curPos = hingeTail_->getPosition();
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

		quat = this->hingeRoot_->getAttitude();
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->hingeTail_->getAttitude();
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

		quat = this->hingeRoot_->getAttitude();
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->hingeTail_->getAttitude();
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
