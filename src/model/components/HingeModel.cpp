/*
 * @(#) HingeModel.cpp   1.0   Feb 8, 2013
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
#include "model/components/HingeModel.h"

namespace robogen {

const float HingeModel::MASS_SLOT = inGrams(2);
const float HingeModel::MASS_FRAME = inGrams(1);
const float HingeModel::SLOT_WIDTH = inMm(34);
const float HingeModel::SLOT_THICKNESS = inMm(1.5);
const float HingeModel::CONNNECTION_PART_LENGTH = inMm(20.5);
const float HingeModel::CONNECTION_PART_HEIGHT = inMm(20);
const float HingeModel::CONNECTION_PART_THICKNESS = inMm(2);

// Computed from the left corner of the connection part
const float HingeModel::CONNECTION_ROTATION_OFFSET = inMm(18.5);

// Center of rotation 18.5 from the slot

HingeModel::HingeModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id) :
		Model(odeWorld, odeSpace, id) {

}

HingeModel::~HingeModel() {

}

bool HingeModel::initModel() {

	// hinge will have 4 components, with new functionality, these are created directly from
	// calls to this->add____

	float separation = 0;//inMm(0.1);

	hingeRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_A_ID);

	dReal xPartA = SLOT_THICKNESS / 2 + separation
			+ CONNNECTION_PART_LENGTH / 2;
	boost::shared_ptr<SimpleBody> connectionPartA = this->addBox(MASS_FRAME,
			osg::Vec3(xPartA, 0, 0),
			CONNNECTION_PART_LENGTH, CONNECTION_PART_THICKNESS,
			CONNECTION_PART_HEIGHT, B_CONNECTION_A_ID);

	dReal xPartB = xPartA
			+ (CONNNECTION_PART_LENGTH / 2
					- (CONNNECTION_PART_LENGTH - CONNECTION_ROTATION_OFFSET))
					* 2;
	boost::shared_ptr<SimpleBody> connectionPartB = this->addBox(MASS_FRAME,
			osg::Vec3(xPartB, 0, 0),
			CONNNECTION_PART_LENGTH, CONNECTION_PART_THICKNESS,
			CONNECTION_PART_HEIGHT, B_CONNECTION_B_ID);

	dReal xTail = xPartB + CONNNECTION_PART_LENGTH / 2 + separation
			+ SLOT_THICKNESS / 2;
	hingeTail_ = this->addBox(MASS_SLOT, osg::Vec3(xTail, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_B_ID);

	// Create joints to hold pieces in position

	// root <-> connectionPartA
	this->fixBodies(hingeRoot_, connectionPartA);

	// connectionPartA <(hinge)> connectionPArtB
	this->attachWithHinge(connectionPartA, connectionPartB,
			osg::Vec3( 0, 0, 1),
			osg::Vec3(xPartA
						+ (CONNNECTION_PART_LENGTH / 2
						- (CONNNECTION_PART_LENGTH
						- CONNECTION_ROTATION_OFFSET)), 0, 0));

	// connectionPartB <-> tail
	this->fixBodies(connectionPartB, hingeTail_);

	return true;

}

boost::shared_ptr<SimpleBody> HingeModel::getRoot() {
	return hingeRoot_;
}

boost::shared_ptr<SimpleBody> HingeModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return hingeRoot_;
	} else {
		return hingeTail_;
	}
}

osg::Vec3 HingeModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->hingeRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->hingeTail_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	}

	return slotPos;

}

osg::Vec3 HingeModel::getSlotAxis(unsigned int i) {

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

osg::Vec3 HingeModel::getSlotOrientation(unsigned int i) {

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

}
