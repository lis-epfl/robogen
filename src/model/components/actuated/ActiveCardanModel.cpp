/*
 * @(#) ActiveCardanModel.cpp   1.0   Feb 14, 2013
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

	// Create the 4 components of the hinge
	cardanRoot_ = this->createBody(B_SLOT_A_ID);
	dBodyID connectionPartA = this->createBody(B_CONNECTION_A_ID);
	dBodyID crossPartA = this->createBody(B_CROSS_PART_A_ID);
	dBodyID crossPartB = this->createBody(B_CROSS_PART_B_ID);
	dBodyID connectionPartB = this->createBody(B_CONNECTION_B_ID);
	cardanTail_ = this->createBody(B_SLOT_B_ID);

	float separation = inMm(0.1);

	this->createBoxGeom(cardanRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	dReal xPartA = SLOT_THICKNESS / 2 + separation
			+ CONNNECTION_PART_LENGTH / 2;
	this->createBoxGeom(connectionPartA, MASS_SERVO, osg::Vec3(xPartA, 0, 0),
			CONNNECTION_PART_LENGTH, CONNNECTION_PART_WIDTH,
			CONNECTION_PART_HEIGHT);

	dReal xCrossPartA = xPartA + CONNECTION_PART_OFFSET;
	this->createBoxGeom(crossPartA, MASS_CROSS / 3,
			osg::Vec3(xCrossPartA, 0, 0), CROSS_THICKNESS, CROSS_WIDTH,
			CROSS_HEIGHT);
	this->createBoxGeom(crossPartB, MASS_CROSS / 3,
			osg::Vec3(xCrossPartA, 0, 0), CROSS_THICKNESS, CROSS_HEIGHT,
			CROSS_WIDTH);

	dReal xPartB = xCrossPartA + CONNECTION_PART_OFFSET;
	this->createBoxGeom(connectionPartB, MASS_SERVO, osg::Vec3(xPartB, 0, 0),
			CONNNECTION_PART_LENGTH, CONNECTION_PART_HEIGHT,
			CONNNECTION_PART_WIDTH);

	dReal xTail = xPartB + CONNNECTION_PART_LENGTH / 2 + separation
			+ SLOT_THICKNESS / 2;
	this->createBoxGeom(cardanTail_, MASS_SLOT, osg::Vec3(xTail, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	// Cross Geometries
	dBodyID crossPartAedge1 = this->createBody(B_CROSS_PART_A_EDGE_1_ID);
	dBodyID crossPartAedge2 = this->createBody(B_CROSS_PART_A_EDGE_2_ID);

	dBodyID crossPartBedge1 = this->createBody(B_CROSS_PART_B_EDGE_1_ID);
	dBodyID crossPartBedge2 = this->createBody(B_CROSS_PART_B_EDGE_2_ID);

	this->createBoxGeom(crossPartAedge1, MASS_CROSS / 12,
			osg::Vec3(xCrossPartA - (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2), 0,
					CROSS_HEIGHT / 2 + (CROSS_THICKNESS / 2)),
			CROSS_CENTER_OFFSET, CROSS_WIDTH, CROSS_THICKNESS);

	this->createBoxGeom(crossPartAedge2, MASS_CROSS / 12,
			osg::Vec3(xCrossPartA - (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2), 0,
					-CROSS_HEIGHT / 2 - (CROSS_THICKNESS / 2)),
			CROSS_CENTER_OFFSET, CROSS_WIDTH, CROSS_THICKNESS);

	this->createBoxGeom(crossPartBedge1, MASS_CROSS / 12,
			osg::Vec3(xCrossPartA + (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2),
					CROSS_HEIGHT / 2 - (CROSS_THICKNESS / 2), 0),
			CROSS_CENTER_OFFSET, CROSS_THICKNESS, CROSS_WIDTH);

	this->createBoxGeom(crossPartBedge2, MASS_CROSS / 12,
			osg::Vec3(xCrossPartA + (CROSS_WIDTH / 2 + CROSS_THICKNESS / 2),
					-CROSS_HEIGHT / 2 + (CROSS_THICKNESS / 2), 0),
			CROSS_CENTER_OFFSET, CROSS_THICKNESS, CROSS_WIDTH);

	// Create joints to hold pieces in position

	// root <-> connectionPartA
	this->fixBodies(cardanRoot_, connectionPartA, osg::Vec3(1, 0, 0));

	// connectionPartA <(hinge)> crossPartA
	dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
	dJointAttach(joint, connectionPartA, crossPartA);
	dJointSetHingeAxis(joint, 0, 0, 1);
	dJointSetHingeAnchor(joint,
			xPartA
					+ ((CONNNECTION_PART_LENGTH / 2)
							- (CONNNECTION_PART_LENGTH
									- CONNNECTION_PART_ROTATION_OFFSET)), 0, 0);

	// crossPartA <-> crossPartB
	this->fixBodies(crossPartA, crossPartB, osg::Vec3(1, 0, 0));

	// crossPartB <(hinge)> connectionPartB
	dJointID joint2 = dJointCreateHinge(this->getPhysicsWorld(), 0);
	dJointAttach(joint2, crossPartB, connectionPartB);
	dJointSetHingeAxis(joint2, 0, 1, 0);
	dJointSetHingeAnchor(joint2,
			xPartB
					- ((CONNNECTION_PART_LENGTH / 2)
							- (CONNNECTION_PART_LENGTH
									- CONNNECTION_PART_ROTATION_OFFSET)), 0, 0);

	// connectionPartB <-> tail
	this->fixBodies(connectionPartB, cardanTail_, osg::Vec3(1, 0, 0));

	// Fix cross Parts
	this->fixBodies(crossPartA, crossPartAedge1, osg::Vec3(1, 0, 0));
	this->fixBodies(crossPartA, crossPartAedge2, osg::Vec3(1, 0, 0));
	this->fixBodies(crossPartB, crossPartBedge1, osg::Vec3(1, 0, 0));
	this->fixBodies(crossPartB, crossPartBedge2, osg::Vec3(1, 0, 0));

	// Create motors
	motor1_.reset(
			new ServoMotor(joint, ServoMotor::DEFAULT_MAX_FORCE,
					ServoMotor::DEFAULT_GAIN,
					NeuralNetworkRepresentation::ioPair(this->getId(),0)));
	motor2_.reset(
			new ServoMotor(joint2, ServoMotor::DEFAULT_MAX_FORCE,
					ServoMotor::DEFAULT_GAIN,
					NeuralNetworkRepresentation::ioPair(this->getId(),1)));

	return true;

}

dBodyID ActiveCardanModel::getRoot() {
	return cardanRoot_;
}

dBodyID ActiveCardanModel::getSlot(unsigned int i) {
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

		osg::Vec3 curPos = this->getPosition(cardanRoot_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->getPosition(cardanTail_);
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

		quat = this->getAttitude(this->cardanRoot_);
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->cardanTail_);
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

		quat = this->getAttitude(this->cardanRoot_);
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->cardanTail_);
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
