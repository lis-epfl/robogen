/*
 * @(#) ActiveCardanModel.cpp   1.0   Feb 14, 2013
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
#include "model/components/actuated/ActiveCardanModel.h"
#include "model/motors/ServoMotor.h"

namespace robogen {

const float ActiveCardanModel::MASS_SERVO = inGrams(7);
const float ActiveCardanModel::MASS_SLOT = inGrams(2);
const float ActiveCardanModel::MASS_CROSS = inGrams(3);

const float ActiveCardanModel::SLOT_WIDTH = inMm(34);
const float ActiveCardanModel::SLOT_THICKNESS = inMm(1.5);
const float ActiveCardanModel::CONNNECTION_PART_WIDTH = inMm(12.5);
const float ActiveCardanModel::CONNNECTION_PART_LENGTH = inMm(24.5);
const float ActiveCardanModel::CONNECTION_PART_HEIGHT = inMm(21);
const float ActiveCardanModel::CROSS_WIDTH = inMm(10);
const float ActiveCardanModel::CROSS_HEIGHT = inMm(34);
const float ActiveCardanModel::CROSS_THICKNESS = inMm(3);

const float ActiveCardanModel::CONNNECTION_PART_ROTATION_OFFSET = inMm(20.5); // Left to right

// Offset of the cross center from the rotation axis
const float ActiveCardanModel::CROSS_CENTER_OFFSET = inMm(17);

// Offset of the cross center respect to the center of the connection part
const float ActiveCardanModel::CONNECTION_PART_OFFSET = inMm(19.875);

ActiveCardanModel::ActiveCardanModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id) :
		ActuatedComponent(odeWorld, odeSpace, id) {

}

ActiveCardanModel::~ActiveCardanModel() {

}

bool ActiveCardanModel::initModel() {

	// Create the 6 components of the model
	// now created directly with calls to this->add___

	float separation = inMm(0.1);

	cardanRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_A_ID);

	dReal xPartA = SLOT_THICKNESS / 2 + separation
			+ CONNNECTION_PART_LENGTH / 2;
	boost::shared_ptr<SimpleBody> connectionPartA = this->addBox(MASS_SERVO,
			osg::Vec3(xPartA, 0, 0),
			CONNNECTION_PART_LENGTH, CONNNECTION_PART_WIDTH,
			CONNECTION_PART_HEIGHT, B_CONNECTION_A_ID);

	dReal xCrossPartA = xPartA + CONNECTION_PART_OFFSET;
	boost::shared_ptr<SimpleBody> crossPartA = this->addBox(MASS_CROSS / 3,
			osg::Vec3(xCrossPartA, 0, 0), CROSS_THICKNESS, CROSS_WIDTH,
			CROSS_HEIGHT, B_CROSS_PART_A_ID);
	boost::shared_ptr<SimpleBody> crossPartB = this->addBox(MASS_CROSS / 3,
			osg::Vec3(xCrossPartA, 0, 0), CROSS_THICKNESS, CROSS_HEIGHT,
			CROSS_WIDTH, B_CROSS_PART_B_ID);

	dReal xPartB = xCrossPartA + CONNECTION_PART_OFFSET;
	boost::shared_ptr<SimpleBody> connectionPartB = this->addBox(MASS_SERVO,
			osg::Vec3(xPartB, 0, 0),
			CONNNECTION_PART_LENGTH, CONNECTION_PART_HEIGHT,
			CONNNECTION_PART_WIDTH, B_CONNECTION_B_ID);

	dReal xTail = xPartB + CONNNECTION_PART_LENGTH / 2 + separation
			+ SLOT_THICKNESS / 2;
	cardanTail_ = this->addBox(MASS_SLOT, osg::Vec3(xTail, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_B_ID);

	// Cross Geometries

	boost::shared_ptr<SimpleBody> crossPartAedge1 = this->addBox(MASS_CROSS / 12,
			osg::Vec3(xCrossPartA - (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2), 0,
					CROSS_HEIGHT / 2 + (CROSS_THICKNESS / 2)),
			CROSS_CENTER_OFFSET, CROSS_WIDTH, CROSS_THICKNESS, B_CROSS_PART_A_EDGE_1_ID);

	boost::shared_ptr<SimpleBody> crossPartAedge2 = this->addBox(MASS_CROSS / 12,
			osg::Vec3(xCrossPartA - (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2), 0,
					-CROSS_HEIGHT / 2 - (CROSS_THICKNESS / 2)),
			CROSS_CENTER_OFFSET, CROSS_WIDTH, CROSS_THICKNESS, B_CROSS_PART_A_EDGE_2_ID);

	boost::shared_ptr<SimpleBody> crossPartBedge1 = this->addBox(MASS_CROSS / 12,
			osg::Vec3(xCrossPartA + (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2),
					CROSS_HEIGHT / 2 - (CROSS_THICKNESS / 2), 0),
			CROSS_CENTER_OFFSET, CROSS_THICKNESS, CROSS_WIDTH, B_CROSS_PART_B_EDGE_1_ID);

	boost::shared_ptr<SimpleBody> crossPartBedge2 = this->addBox(MASS_CROSS / 12,
			osg::Vec3(xCrossPartA + (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2),
					-CROSS_HEIGHT / 2 + (CROSS_THICKNESS / 2), 0),
			CROSS_CENTER_OFFSET, CROSS_THICKNESS, CROSS_WIDTH, B_CROSS_PART_B_EDGE_2_ID);

	// Create joints to hold pieces in position

	// root <-> connectionPartA
	this->fixBodies(cardanRoot_, connectionPartA);

	// connectionPartA <(hinge)> crossPartA
	boost::shared_ptr<Joint> joint = this->attachWithHinge(connectionPartA,
			connectionPartB, osg::Vec3(0, 0, 1),
			osg::Vec3(xPartA
					+ ((CONNNECTION_PART_LENGTH / 2)
					- (CONNNECTION_PART_LENGTH
					- CONNNECTION_PART_ROTATION_OFFSET)), 0, 0));


	// crossPartA <-> crossPartB
	this->fixBodies(crossPartA, crossPartB);

	// crossPartB <(hinge)> connectionPartB
	boost::shared_ptr<Joint> joint2 = this->attachWithHinge(crossPartB,
			connectionPartB, osg::Vec3(0, 1, 0),
			osg::Vec3(xPartB
					- ((CONNNECTION_PART_LENGTH / 2)
					- (CONNNECTION_PART_LENGTH
					- CONNNECTION_PART_ROTATION_OFFSET)), 0, 0));

	// connectionPartB <-> tail
	this->fixBodies(connectionPartB, cardanTail_);

	// Fix cross Parts
	this->fixBodies(crossPartA, crossPartAedge1);
	this->fixBodies(crossPartA, crossPartAedge2);
	this->fixBodies(crossPartB, crossPartBedge1);
	this->fixBodies(crossPartB, crossPartBedge2);

	// Create motors
	motor1_.reset(
			new ServoMotor(ioPair(this->getId(),0), joint,
							ServoMotor::DEFAULT_MAX_FORCE_SERVO,
							ServoMotor::DEFAULT_GAIN));
	motor2_.reset(
			new ServoMotor(ioPair(this->getId(),1), joint2,
							ServoMotor::DEFAULT_MAX_FORCE_SERVO,
							ServoMotor::DEFAULT_GAIN));
	return true;

}

boost::shared_ptr<SimpleBody> ActiveCardanModel::getRoot() {
	return cardanRoot_;
}

boost::shared_ptr<SimpleBody> ActiveCardanModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return cardanRoot_;
	} else {
		return cardanTail_;
	}
}

osg::Vec3 ActiveCardanModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[ActiveCardanModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->cardanRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->cardanTail_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	}

	return slotPos;

}

osg::Vec3 ActiveCardanModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[ActiveCardanModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->cardanRoot_->getAttitude();
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->cardanTail_->getAttitude();
		axis.set(1, 0, 0);

	}

	return quat * axis;

}

osg::Vec3 ActiveCardanModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[ActiveCardanModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->cardanRoot_->getAttitude();
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->cardanTail_->getAttitude();
		axis.set(0, 1, 0);

	}

	return quat * axis;

}

void ActiveCardanModel::getMotors(
		std::vector<boost::shared_ptr<Motor> >& motors) {

	motors.resize(2);
	motors[0] = motor1_;
	motors[1] = motor2_;
}

}
