/*
 * @(#) ParametricBrickModel.cpp   1.0   Feb 14, 2013
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

	// ParametricBrick is composed of 5 geometries,
	// now created directly with calls to this->add___

	brickRoot_ = this->addBox(MASS_SLOT, osg::Vec3(0, 0, 0),
				SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_A_ID);

	osg::Vec3 fixedBarPosition(SLOT_THICKNESS / 2. + FIXED_BAR_LENGTH/2., 0, 0);
	boost::shared_ptr<SimpleBody> fixedBar = this->addBox(
			MASS_CONNECTION_PER_M * FIXED_BAR_LENGTH,
			fixedBarPosition,
			FIXED_BAR_LENGTH, CONNECTION_PART_WIDTH, CONNECTION_PART_THICKNESS,
			B_FIXED_BAR__ID);

	this->fixBodies(brickRoot_, fixedBar);

	osg::Vec3 cylinderPosition(fixedBarPosition.x() + FIXED_BAR_LENGTH/2.,
			0, 0);
	// mass uses length of radius, since only half cylinder outside of box
	boost::shared_ptr<SimpleBody> cylinder = this->addCylinder(
			MASS_CONNECTION_PER_M * CYLINDER_RADIUS,
			cylinderPosition, 2,
			CYLINDER_RADIUS, CONNECTION_PART_WIDTH, B_CYLINDER_ID);

	this->fixBodies(fixedBar, cylinder);


	// here we just place the box at origin since we will correctly
	// position it after applying the rotation.
	boost::shared_ptr<SimpleBody>  connectionPart = this->addBox(
			MASS_CONNECTION_PER_M * connectionPartLength_,
			osg::Vec3(0,0,0), connectionPartLength_,
			CONNECTION_PART_WIDTH, CONNECTION_PART_THICKNESS,
			B_CONNECTION_PART_ID);


	// Now create the rotation
	osg::Quat rotationA;
	rotationA.makeRotate(osg::DegreesToRadians(angleA_), osg::Vec3(0, 1, 0));

	// the piece will be positioned at the cylinder's position
	// + the rotated offset
	osg::Vec3 connectionPartPosition = cylinderPosition + rotationA *
							osg::Vec3(connectionPartLength_ / 2., 0, 0);

	connectionPart->setAttitude(rotationA);
	connectionPart->setPosition(connectionPartPosition);

	// Create joints to hold pieces in position
	this->fixBodies(cylinder, connectionPart);


	// do something similar with the slot piece
	// first place it at origin
	brickTail_ = this->addBox(MASS_SLOT, osg::Vec3(0,0,0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH, B_SLOT_B_ID);


	// Create rotation, which will actually be combined rotation of the
	// two angles
	osg::Quat rotationB;
	rotationB.makeRotate(osg::DegreesToRadians(angleB_), osg::Vec3(1, 0, 0));

	rotationB = rotationB * rotationA;

	// the piece will be positioned at the connection part's position +
	// the rotated offset
	osg::Vec3 slotBPosition = connectionPartPosition + rotationB *
				osg::Vec3(connectionPartLength_ / 2 + SLOT_THICKNESS / 2, 0, 0);

	brickTail_->setAttitude(rotationB);
	brickTail_->setPosition(slotBPosition);

	// Fix slot B
	this->fixBodies(connectionPart, brickTail_);

	return true;

}

boost::shared_ptr<SimpleBody> ParametricBrickModel::getRoot() {
	return brickRoot_;
}

boost::shared_ptr<SimpleBody> ParametricBrickModel::getSlot(unsigned int i) {
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

		osg::Vec3 curPos = this->brickRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->brickTail_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

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

		quat = this->brickRoot_->getAttitude();
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->brickTail_->getAttitude();
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

		quat = this->brickRoot_->getAttitude();
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->brickTail_->getAttitude();
		axis.set(0, 1, 0);

	}

	return quat * axis;

}

}
