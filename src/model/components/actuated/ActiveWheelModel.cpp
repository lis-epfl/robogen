/*
 * @(#) ActiveWheelModel.cpp   1.0   Feb 13, 2013
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
#include "model/components/actuated/ActiveWheelModel.h"
#include "model/motors/RotationMotor.h"

namespace robogen {

// TODO verify these
const float ActiveWheelModel::MASS_SLOT = inGrams(2);
const float ActiveWheelModel::MASS_SERVO = inGrams(11);
const float ActiveWheelModel::MASS_WHEEL = inGrams(5);

const float ActiveWheelModel::SLOT_WIDTH = inMm(34);
const float ActiveWheelModel::SLOT_THICKNESS = inMm(1.5);

const float ActiveWheelModel::SERVO_WIDTH = inMm(14);
const float ActiveWheelModel::SERVO_LENGTH = inMm(36.8);
const float ActiveWheelModel::SERVO_HEIGHT = inMm(14);

const float ActiveWheelModel::WHEEL_THICKNESS = inMm(3);
// attachment part now goes on exterior since students found easier to
// attach that way
const float ActiveWheelModel::WHEEL_ATTACHMENT_THICKNESS = inMm(6);


ActiveWheelModel::ActiveWheelModel(dWorldID odeWorld, dSpaceID odeSpace,
      std::string id, float radius) :
      ActuatedComponent(odeWorld, odeSpace, id), radius_(radius) {

}

ActiveWheelModel::~ActiveWheelModel() {

}

float ActiveWheelModel::getRadius() const {
   return radius_;
}

bool ActiveWheelModel::initModel() {

	// Create the 3 components of the wheel,
	// now created directly with calls to this->add___

	wheelRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_ID);

	dReal xServo = SERVO_LENGTH / 2 + SLOT_THICKNESS / 2;

	boost::shared_ptr<SimpleBody> servo = this->addBox(MASS_SERVO,
		   osg::Vec3(xServo, 0, 0),
		 SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT, B_SERVO_ID);

	// wheel should be placed so outside of attachment aligns with end of
	// motor

	dReal xWheel = xServo + SERVO_LENGTH/2 - WHEEL_ATTACHMENT_THICKNESS -
			WHEEL_THICKNESS/2;

	// TODO change mass based on radius
	boost::shared_ptr<SimpleBody> wheel = this->addCylinder(MASS_WHEEL,
		   osg::Vec3(xWheel, 0, 0), 1, getRadius(), WHEEL_THICKNESS,
		   B_WHEEL_ID);

	// Create joints to hold pieces in position
	this->fixBodies(wheelRoot_, servo);

	boost::shared_ptr<Joint> joint = this->attachWithHinge(
		   servo, wheel,  osg::Vec3(1, 0, 0), osg::Vec3(xWheel, 0, 0));

	// Create servo
	this->motor_.reset(
		 new RotationMotor(ioPair(this->getId(),0), joint,
				 RotationMotor::DEFAULT_MAX_FORCE_ROTATIONAL));

	return true;

}

boost::shared_ptr<SimpleBody> ActiveWheelModel::getRoot() {
	return wheelRoot_;
}

boost::shared_ptr<SimpleBody> ActiveWheelModel::getSlot(unsigned int i) {

	if (i > 1) {
	  std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
	  assert(i <= 1);
	}

	return wheelRoot_;
}

osg::Vec3 ActiveWheelModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
	  std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
	  assert(i <= 1);
	}

	osg::Vec3 curPos = wheelRoot_->getPosition();
	osg::Vec3 slotAxis = this->getSlotAxis(i);
	return curPos + slotAxis * (SLOT_THICKNESS / 2);

}

osg::Vec3 ActiveWheelModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
	  std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
	  assert(i <= 1);
	}

	osg::Quat quat = wheelRoot_->getAttitude();
	osg::Vec3 axis(-1, 0, 0);

	return quat * axis;

}

osg::Vec3 ActiveWheelModel::getSlotOrientation(unsigned int i) {

	if (i > 1) {
	  std::cout << "[ActiveWheelModel] Invalid slot: " << i << std::endl;
	  assert(i <= 1);
	}

	osg::Quat quat = wheelRoot_->getAttitude();
	osg::Vec3 axis(0, 1, 0);

	return quat * axis;

}

void ActiveWheelModel::getMotors(
		std::vector<boost::shared_ptr<Motor> >& motors) {
	motors.resize(1);
	motors[0] = this->motor_;
}

}
