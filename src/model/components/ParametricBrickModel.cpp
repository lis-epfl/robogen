/*
 * @(#) ParametricBrickModel.cpp   1.0   Feb 14, 2013
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
#include "model/components/ParametricBrickModel.h"

namespace robogen {

const float ParametricBrickModel::MASS_SLOT = inGrams(1);
const float ParametricBrickModel::MASS_CONNECTION_PER_CM = inGrams(2);
const float ParametricBrickModel::SLOT_WIDTH = inMm(34);
const float ParametricBrickModel::SLOT_THICKNESS = inMm(1.5);
const float ParametricBrickModel::CONNECTION_PART_WIDTH = inMm(20);
const float ParametricBrickModel::CONNECTION_PART_THICKNESS = inMm(2);
const float ParametricBrickModel::CAPSULE_HEIGHT = inMm(10);
const float ParametricBrickModel::CAPSULE_LENGTH = inMm(10);

ParametricBrickModel::ParametricBrickModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id, float connectionPartLength, float angleA, float angleB):
		Model(odeWorld, odeSpace, id),
		connectionPartLength_(connectionPartLength), angleA_(angleA),
		angleB_(angleB) {

}

ParametricBrickModel::~ParametricBrickModel() {

}

bool ParametricBrickModel::initModel() {

	// Create the 4 components of the hinge
	brickRoot_ = this->createBody(B_SLOT_A_ID);
	//dBodyID capsule = this->createBody(B_CAPSULE_ID);
	dBodyID connectionPart = this->createBody(B_CONNECTION_ID);
	brickTail_ = this->createBody(B_SLOT_B_ID);

	// Create the geometries
	float separation = inMm(0.1);

	this->createBoxGeom(brickRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	dReal xConnection = SLOT_THICKNESS / 2 + connectionPartLength_ / 2;
	osg::Vec3 connectionPartPos(xConnection, 0, 0);
	this->createBoxGeom(connectionPart,
			MASS_CONNECTION_PER_CM * CONNECTION_PART_WIDTH / 10,
			connectionPartPos, connectionPartLength_, CONNECTION_PART_WIDTH,
			CONNECTION_PART_THICKNESS);

	dReal xSlotB = xConnection + connectionPartLength_ / 2 + separation
			+ SLOT_THICKNESS / 2;
	osg::Vec3 posSlotB(xSlotB, 0, 0);
	this->createBoxGeom(brickTail_, MASS_SLOT, posSlotB, SLOT_THICKNESS,
			SLOT_WIDTH, SLOT_WIDTH);

	// Rotating the connection should be enough before fixing it
	osg::Quat rotationA;
	rotationA.makeRotate(osg::DegreesToRadians(angleA_), osg::Vec3(0, 1, 0));

	dQuaternion quatOde;
	quatOde[0] = rotationA.w();
	quatOde[1] = rotationA.x();
	quatOde[2] = rotationA.y();
	quatOde[3] = rotationA.z();

	connectionPartPos = rotationA * connectionPartPos;
	dBodySetQuaternion(connectionPart, quatOde);
	dBodySetPosition(connectionPart, connectionPartPos.x(),
			connectionPartPos.y(), connectionPartPos.z());

	// Create joints to hold pieces in position
	this->fixBodies(brickRoot_, connectionPart, osg::Vec3(1, 0, 0));

	// Rotate slot B
	osg::Quat rotationB;
	rotationB.makeRotate(osg::DegreesToRadians(angleB_), osg::Vec3(1, 0, 0));

	rotationB = rotationB * rotationA;

	quatOde[0] = rotationB.w();
	quatOde[1] = rotationB.x();
	quatOde[2] = rotationB.y();
	quatOde[3] = rotationB.z();
	dBodySetQuaternion(brickTail_, quatOde);

	posSlotB = rotationB * posSlotB;
	dBodySetPosition(brickTail_, posSlotB.x(), posSlotB.y(), posSlotB.z());

	// Fix slot B
	this->fixBodies(connectionPart, brickTail_, osg::Vec3(1, 0, 0));

	return true;

}

dBodyID ParametricBrickModel::getRoot() {
	return brickRoot_;
}

dBodyID ParametricBrickModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return brickRoot_;
	} else {
		return brickTail_;
	}
}

osg::Vec3 ParametricBrickModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[ParametricBrickModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->getPosition(brickRoot_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->getPosition(brickTail_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		return curPos + slotAxis * (SLOT_THICKNESS / 2);
	}

	return slotPos;

}

osg::Vec3 ParametricBrickModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[ParametricBrickModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->getAttitude(this->brickRoot_);
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->brickTail_);
		axis.set(1, 0, 0);

	}

	return quat * axis;

}

osg::Vec3 ParametricBrickModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[ParametricBrickModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->getAttitude(this->brickRoot_);
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->getAttitude(this->brickTail_);
		axis.set(0, 1, 0);

	}

	return quat * axis;

}

float ParametricBrickModel::getConnectionLength() {
	return connectionPartLength_;
}

}
