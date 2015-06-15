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
const float ParametricBrickModel::MASS_CONNECTION_PER_M = inGrams(1.37) * 100.;
const float ParametricBrickModel::SLOT_WIDTH = inMm(34);
const float ParametricBrickModel::SLOT_THICKNESS = inMm(1.5);
const float ParametricBrickModel::CONNECTION_PART_WIDTH = inMm(20);
const float ParametricBrickModel::CONNECTION_PART_THICKNESS = inMm(10);
const float ParametricBrickModel::CYLINDER_RADIUS = inMm(5);
const float ParametricBrickModel::FIXED_BAR_LENGTH = inMm(6);

ParametricBrickModel::ParametricBrickModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id, float connectionPartLength, float angleA, float angleB):
		Model(odeWorld, odeSpace, id),
		connectionPartLength_(connectionPartLength + CYLINDER_RADIUS),
		angleA_(angleA),
		angleB_(angleB) {

}

ParametricBrickModel::~ParametricBrickModel() {

}

bool ParametricBrickModel::initModel() {

	// Create the 4 components of the hinge
	brickRoot_ = this->createBody(B_SLOT_A_ID);

	this->createBoxGeom(brickRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
				SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	dBodyID fixedBar = this->createBody(B_FIXED_BAR__ID);
	osg::Vec3 fixedBarPosition(SLOT_THICKNESS / 2. + FIXED_BAR_LENGTH/2., 0, 0);
	this->createBoxGeom(fixedBar, MASS_CONNECTION_PER_M * FIXED_BAR_LENGTH,
			fixedBarPosition,
			FIXED_BAR_LENGTH, CONNECTION_PART_WIDTH, CONNECTION_PART_THICKNESS);

	this->fixBodies(brickRoot_, fixedBar,  osg::Vec3(1, 0, 0) );

	dBodyID cylinder = this->createBody(B_CYLINDER_ID);


	osg::Vec3 cylinderPosition(fixedBarPosition.x() + FIXED_BAR_LENGTH/2.,
			0, 0);
	// mass uses length of radius, since only half cylinder outside of box
	this->createCylinderGeom(cylinder, MASS_CONNECTION_PER_M * CYLINDER_RADIUS,
			cylinderPosition, 2,
			CYLINDER_RADIUS, CONNECTION_PART_WIDTH);

	this->fixBodies(fixedBar, cylinder,  osg::Vec3(1, 0, 0) );


	dBodyID connectionPart = this->createBody(B_CONNECTION_PART_ID);

	// here we just place the box at origin since we will correctly
	// position it after applying the rotation.
	this->createBoxGeom(connectionPart,
			MASS_CONNECTION_PER_M * connectionPartLength_,
			osg::Vec3(0,0,0), connectionPartLength_,
			CONNECTION_PART_WIDTH, CONNECTION_PART_THICKNESS);


	// Now create the rotation
	osg::Quat rotationA;
	rotationA.makeRotate(osg::DegreesToRadians(angleA_), osg::Vec3(0, 1, 0));

	dQuaternion quatOde;
	quatOde[0] = rotationA.w();
	quatOde[1] = rotationA.x();
	quatOde[2] = rotationA.y();
	quatOde[3] = rotationA.z();

	// the piece will be positioned at the cylinder's position
	// + the rotated offset
	osg::Vec3 connectionPartPosition = cylinderPosition + rotationA *
							osg::Vec3(connectionPartLength_ / 2., 0, 0);

	dBodySetQuaternion(connectionPart, quatOde);
	dBodySetPosition(connectionPart, connectionPartPosition.x(),
			connectionPartPosition.y(), connectionPartPosition.z());

	// Create joints to hold pieces in position
	this->fixBodies(cylinder, connectionPart, osg::Vec3(1, 0, 0));


	brickTail_ = this->createBody(B_SLOT_B_ID);

	// do something similar with the slot piece
	// first place it at origin
	this->createBoxGeom(brickTail_, MASS_SLOT, osg::Vec3(0,0,0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);


	// Create rotation, which will actually be combined rotation of the
	// two angles
	osg::Quat rotationB;
	rotationB.makeRotate(osg::DegreesToRadians(angleB_), osg::Vec3(1, 0, 0));

	rotationB = rotationB * rotationA;

	quatOde[0] = rotationB.w();
	quatOde[1] = rotationB.x();
	quatOde[2] = rotationB.y();
	quatOde[3] = rotationB.z();

	// the piece will be positioned at the connection part's position +
	// the roatated offset
	osg::Vec3 slotBPosition = connectionPartPosition + rotationB *
				osg::Vec3(connectionPartLength_ / 2 + SLOT_THICKNESS / 2, 0, 0);

	dBodySetQuaternion(brickTail_, quatOde);
	dBodySetPosition(brickTail_, slotBPosition.x(), slotBPosition.y(),
			slotBPosition.z());

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
		slotPos = curPos - slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->getPosition(brickTail_);
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos - slotAxis * (SLOT_THICKNESS / 2);

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
