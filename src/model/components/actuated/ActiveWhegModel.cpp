/*
 * @(#) ActiveWhegModel.cpp   1.0   Feb 27, 2013
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
#include <cmath>
#include "model/components/actuated/ActiveWhegModel.h"
#include "model/motors/ServoMotor.h"

namespace robogen {

const float ActiveWhegModel::MASS_SLOT = inGrams(2);
const float ActiveWhegModel::MASS_SERVO = inGrams(7);
const float ActiveWhegModel::MASS_WHEG = inGrams(3);

const float ActiveWhegModel::SLOT_WIDTH = inMm(34);
const float ActiveWhegModel::SLOT_THICKNESS = inMm(1.5);
const float ActiveWhegModel::SERVO_Z_OFFSET = inMm(9); // zCenter shift respect to slot z-center
const float ActiveWhegModel::SERVO_WIDTH = inMm(10);
const float ActiveWhegModel::SERVO_LENGTH = inMm(29);
const float ActiveWhegModel::SERVO_HEIGHT = inMm(28);
const float ActiveWhegModel::WHEG_BASE_RADIUS = inMm(9);
const float ActiveWhegModel::WHEG_RADIUS = inMm(31);
const float ActiveWhegModel::WHEG_THICKNESS = inMm(4);
const float ActiveWhegModel::WHEG_WIDTH = inMm(4);

ActiveWhegModel::ActiveWhegModel(dWorldID odeWorld, dSpaceID odeSpace) :
		ActuatedComponent(odeWorld, odeSpace) {

}

ActiveWhegModel::~ActiveWhegModel() {

}

bool ActiveWhegModel::initModel() {

	// Create the components of the wheg
	whegRoot_ = this->createBody(B_SLOT_ID);
	dBodyID servo = this->createBody(B_SERVO_ID);
	dBodyID whegBase = this->createBody(B_WHEG_BASE);
	dBodyID spoke1 = this->createBody(B_WHEG_SPOKE_1);
	dBodyID spoke2 = this->createBody(B_WHEG_SPOKE_2);
	dBodyID spoke3 = this->createBody(B_WHEG_SPOKE_3);

	// Set the masses for the various boxes
	dMass mass;

	float separation = inMm(0.1);

	this->createBoxGeom(whegRoot_, MASS_SLOT, osg::Vec3(0, 0, 0),
			SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	dReal xServo = SLOT_THICKNESS / 2 + separation + SERVO_LENGTH / 2;
	dReal zServo = 0;
	this->createBoxGeom(servo, MASS_SERVO, osg::Vec3(xServo, 0, zServo),
			SERVO_LENGTH, SERVO_WIDTH, SERVO_HEIGHT);

	dReal xWhegBase = xServo + SERVO_LENGTH / 2 + separation
			+ WHEG_THICKNESS / 2;
	this->createCylinderGeom(whegBase, MASS_WHEG / 4,
			osg::Vec3(xWhegBase, 0, zServo), 1, WHEG_BASE_RADIUS,
			WHEG_THICKNESS);

	this->createBoxGeom(spoke1, MASS_WHEG / 4,
			osg::Vec3(xWhegBase, 0,
					zServo + WHEG_BASE_RADIUS + WHEG_RADIUS / 2),
			WHEG_THICKNESS, WHEG_WIDTH, WHEG_RADIUS);

	this->createBoxGeom(spoke2, MASS_WHEG / 4, osg::Vec3(xWhegBase, 0, zServo),
			WHEG_THICKNESS, WHEG_WIDTH, WHEG_RADIUS);

	this->createBoxGeom(spoke3, MASS_WHEG / 4, osg::Vec3(xWhegBase, 0, zServo),
			WHEG_THICKNESS, WHEG_WIDTH, WHEG_RADIUS);

	// Position spokes
	osg::Quat rotation;
	dQuaternion quatOde;

	float rotationSpoke2 = 120;
	float rotationSpoke3 = 240;

	rotation.makeRotate(osg::inDegrees(rotationSpoke2), osg::Vec3(1, 0, 0));
	quatOde[0] = rotation.w();
	quatOde[1] = rotation.x();
	quatOde[2] = rotation.y();
	quatOde[3] = rotation.z();
	dBodySetQuaternion(spoke2, quatOde);

	rotation.makeRotate(osg::inDegrees(rotationSpoke3), osg::Vec3(1, 0, 0));
	quatOde[0] = rotation.w();
	quatOde[1] = rotation.x();
	quatOde[2] = rotation.y();
	quatOde[3] = rotation.z();
	dBodySetQuaternion(spoke3, quatOde);

	// Move center of spokes
	osg::Vec3 newPosSpoke2(xWhegBase, 0, zServo);
	newPosSpoke2 += osg::Vec3(0,
			(WHEG_BASE_RADIUS + WHEG_RADIUS / 2)
					* std::cos(osg::inDegrees(90.0 + rotationSpoke2)),
			(WHEG_BASE_RADIUS + WHEG_RADIUS / 2)
					* std::sin(osg::inDegrees(90.0 + rotationSpoke2)));
	dBodySetPosition(spoke2, newPosSpoke2.x(), newPosSpoke2.y(),
			newPosSpoke2.z());

	osg::Vec3 newPosSpoke3(xWhegBase, 0, zServo);
	newPosSpoke3 += osg::Vec3(0,
			(WHEG_BASE_RADIUS + WHEG_RADIUS / 2)
					* std::cos(osg::inDegrees(90.0 + rotationSpoke3)),
			(WHEG_BASE_RADIUS + WHEG_RADIUS / 2)
					* std::sin(osg::inDegrees(90.0 + rotationSpoke3)));
	dBodySetPosition(spoke3, newPosSpoke3.x(), newPosSpoke3.y(),
			newPosSpoke3.z());

	// Create joints to hold pieces in position

	// slot <slider> servo
	this->fixBodies(whegRoot_, servo, osg::Vec3(1, 0, 0));
	this->fixBodies(whegBase, spoke1, osg::Vec3(1, 0, 0));
	this->fixBodies(whegBase, spoke2, osg::Vec3(1, 0, 0));
	this->fixBodies(whegBase, spoke3, osg::Vec3(1, 0, 0));

	// servo <(hinge)> wheg base
	dJointID joint = dJointCreateHinge(this->getPhysicsWorld(), 0);
	dJointAttach(joint, servo, whegBase);
	dJointSetHingeAxis(joint, 1, 0, 0);
	dJointSetHingeAnchor(joint, xWhegBase, 0, 0);

	// Create servo
	this->motor_.reset(
			new ServoMotor(joint, ServoMotor::DEFAULT_MAX_FORCE,
					ServoMotor::DEFAULT_GAIN));

	return true;

}

dBodyID ActiveWhegModel::getRoot() {
	return whegRoot_;
}

dBodyID ActiveWhegModel::getSlot(unsigned int i) {

	if (i > 1) {
		std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	return whegRoot_;
}

osg::Vec3 ActiveWhegModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
		std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Vec3 curPos = this->getPosition(whegRoot_);
	osg::Vec3 slotAxis = this->getSlotAxis(i);
	return curPos + slotAxis * (SLOT_THICKNESS / 2);

}

osg::Vec3 ActiveWhegModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
		std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat = this->getAttitude(this->whegRoot_);
	osg::Vec3 axis(-1, 0, 0);

	return quat * axis;

}

osg::Vec3 ActiveWhegModel::getSlotOrientation(unsigned int i) {

	if (i > 1) {
		std::cout << "[ActiveWhegModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat = this->getAttitude(this->whegRoot_);
	osg::Vec3 axis(0, 1, 0);

	return quat * axis;

}

void ActiveWhegModel::getMotors(
		std::vector<boost::shared_ptr<Motor> >& motors) {
	motors.resize(1);
	motors[0] = this->motor_;
}

}
