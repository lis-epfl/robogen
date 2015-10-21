/*
 * @(#) CardanModel.cpp   1.0   Feb 8, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani
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
#include "model/components/CardanModel.h"

namespace robogen {

const float CardanModel::MASS_SLOT = inGrams(2);
const float CardanModel::MASS_FRAME = inGrams(3);

const float CardanModel::SLOT_WIDTH = inMm(34);
const float CardanModel::SLOT_THICKNESS = inMm(1.5);
const float CardanModel::CONNNECTION_PART_LENGTH = inMm(24);
const float CardanModel::CONNECTION_PART_HEIGHT = inMm(10);

// Computed from the left corner of the connection part
const float CardanModel::CONNECTION_PART_OFFSET = inMm(18.5);

CardanModel::CardanModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id) :
		Model(odeWorld, odeSpace, id) {

}

CardanModel::~CardanModel() {

}

bool CardanModel::initModel() {

	// Create the 4 components of the cardan
	// now created directly with calls to this->add___

	float separation = inMm(0.1);

	cardanRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_A_ID);

	dReal xPartA = SLOT_THICKNESS / 2 + separation
			+ CONNNECTION_PART_LENGTH / 2;
	boost::shared_ptr<SimpleBody> connectionPartA = this->addBox(
			MASS_FRAME, osg::Vec3(xPartA, 0, 0),
			CONNNECTION_PART_LENGTH, SLOT_WIDTH, CONNECTION_PART_HEIGHT,
			B_CONNECTION_A_ID);

	dReal xPartB = xPartA
			+ (CONNNECTION_PART_LENGTH / 2
					- (CONNNECTION_PART_LENGTH - CONNECTION_PART_OFFSET)) * 2;
	boost::shared_ptr<SimpleBody> connectionPartB = this->addBox(
			MASS_FRAME, osg::Vec3(xPartB, 0, 0),
			CONNNECTION_PART_LENGTH, CONNECTION_PART_HEIGHT, SLOT_WIDTH,
			B_CONNECTION_B_ID);

	dReal xTail = xPartB + CONNNECTION_PART_LENGTH / 2 + separation
			+ SLOT_THICKNESS / 2;
	cardanTail_ = this->addBox(MASS_SLOT, osg::Vec3(xTail, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_B_ID);

	// Create joints to hold pieces in position

	// root <-> connectionPartA
	this->fixBodies(cardanRoot_, connectionPartA);

	// connectionPartA <(universal)> connectionPArtB

	universalJoint_ = this->attachWithUniversal(connectionPartA,
			connectionPartB,
			osg::Vec3(0, 1, 0), osg::Vec3(0, 0, 1),
			osg::Vec3(
					xPartA + (CONNNECTION_PART_LENGTH / 2
						- (CONNNECTION_PART_LENGTH - CONNECTION_PART_OFFSET)),
					0, 0));

	// connectionPartB <-> tail
	this->fixBodies(connectionPartB, cardanTail_);

	return true;

}

boost::shared_ptr<SimpleBody> CardanModel::getRoot() {
	return cardanRoot_;
}

boost::shared_ptr<Joint> CardanModel::getJoint() {
	return universalJoint_;
}

boost::shared_ptr<SimpleBody> CardanModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return cardanRoot_;
	} else {
		return cardanTail_;
	}
}

osg::Vec3 CardanModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[CardanModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->cardanRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->cardanTail_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	}

	return slotPos;

}

osg::Vec3 CardanModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[CardanModel] Invalid slot: " << i << std::endl;
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

osg::Vec3 CardanModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[CardanModel] Invalid slot: " << i << std::endl;
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

}
