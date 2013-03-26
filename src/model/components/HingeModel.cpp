/*
 * @(#) HingeModel.cpp   1.0   Feb 8, 2013
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

HingeModel::HingeModel(dWorldID odeWorld, dSpaceID odeSpace) :
		Model(odeWorld, odeSpace) {

}

HingeModel::~HingeModel() {

}

bool HingeModel::initModel() {

	// Create the 4 components of the hinge
	hingeRoot_ = this->createBody(B_SLOT_A_ID);
	dBodyID connectionPartA = this->createBody(B_CONNECTION_A_ID);
	dBodyID connectionPartB = this->createBody(B_CONNECTION_B_ID);
	hingeTail_ = this->createBody(B_SLOT_B_ID);

	// Create the geometries
	float separation = inMm(0.1);

	this->createBoxGeom(hingeRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	dReal xPartA = SLOT_THICKNESS / 2 + separation
			+ CONNNECTION_PART_LENGTH / 2;
	this->createBoxGeom(connectionPartA, MASS_FRAME, osg::Vec3(xPartA, 0, 0),
			CONNNECTION_PART_LENGTH, CONNECTION_PART_THICKNESS,
			CONNECTION_PART_HEIGHT);

	dReal xPartB = xPartA
			+ (CONNNECTION_PART_LENGTH / 2
					- (CONNNECTION_PART_LENGTH - CONNECTION_ROTATION_OFFSET))
					* 2;
	this->createBoxGeom(connectionPartB, MASS_FRAME, osg::Vec3(xPartB, 0, 0),
			CONNNECTION_PART_LENGTH, CONNECTION_PART_THICKNESS,
			CONNECTION_PART_HEIGHT);

	dReal xTail = xPartB + CONNNECTION_PART_LENGTH / 2 + separation
			+ SLOT_THICKNESS / 2;
	this->createBoxGeom(hingeTail_, MASS_SLOT, osg::Vec3(xTail, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	// Create joints to hold pieces in position

	// root <-> connectionPartA
	this->fixBodies(hingeRoot_, connectionPartA, osg::Vec3(1, 0, 0));

	// connectionPartA <(hinge)> connectionPArtB
	dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
	dJointAttach(joint, connectionPartA, connectionPartB);
	dJointSetHingeAxis(joint, 0, 0, 1);
	dJointSetHingeAnchor(joint,
			xPartA
					+ (CONNNECTION_PART_LENGTH / 2
							- (CONNNECTION_PART_LENGTH
									- CONNECTION_ROTATION_OFFSET)), 0, 0);

	// connectionPartB <-> tail
	this->fixBodies(connectionPartB, hingeTail_, osg::Vec3(1, 0, 0));

	return true;

}

dBodyID HingeModel::getRoot() {
	return hingeRoot_;
}

dBodyID HingeModel::getSlot(unsigned int i) {
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

		osg::Vec3 curPos = this->getPosition(hingeRoot_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->getPosition(hingeTail_);
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

		quat = this->getAttitude(this->hingeRoot_);
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->hingeTail_);
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

		quat = this->getAttitude(this->hingeRoot_);
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->hingeTail_);
		axis.set(0, 1, 0);

	}

	return quat * axis;

}

}
