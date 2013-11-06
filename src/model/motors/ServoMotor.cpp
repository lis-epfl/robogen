/*
 * @(#) ServoMotor.cpp   1.0   Feb 20, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch),
 * Titus Cieslewski (dev@titus-c.ch)
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
#include <iostream>
#include "model/motors/ServoMotor.h"

namespace robogen {

const float ServoMotor::DEFAULT_GAIN = 0.5;

// Expressed in Newton-millimeters from kg-cm = ((kg-cm)*g)*10
const float ServoMotor::DEFAULT_MAX_FORCE = 25 * 9.81 * 1000;
const float ServoMotor::MIN_POS_RAD = -(M_PI / 2) * 6 / 9;
const float ServoMotor::MAX_POS_RAD = (M_PI / 2) * 6 / 9;
const float ServoMotor::MIN_VELOCITY = -3;
const float ServoMotor::MAX_VELOCITY = 3;

ServoMotor::ServoMotor(dJointID joint, float maxForce, float gain,
		NeuralNetworkRepresentation::ioPair id) : Motor(id),
		joint_(joint), maxForce_(maxForce), gain_(gain), isVelocityDriven_(
				false){
	dJointSetHingeParam(joint_, dParamFMax, maxForce_);
}

ServoMotor::ServoMotor(dJointID joint, float maxForce,
		NeuralNetworkRepresentation::ioPair id) : Motor(id),
		joint_(joint), maxForce_(maxForce), gain_(0), isVelocityDriven_(
				true) {
	dJointSetHingeParam(joint_, dParamFMax, maxForce_);
}

ServoMotor::~ServoMotor() {

}

void ServoMotor::setPosition(float position) {

	if (position > 1) {
		position = 1;
	} else if (position < 0) {
		position = 0;
	}

	position = MIN_POS_RAD + position * (MAX_POS_RAD - MIN_POS_RAD);

	dReal curPosition = dJointGetHingeAngle(joint_);
	dReal error = curPosition - position;

	dReal velocity = -gain_ * error;
	if (velocity > MAX_VELOCITY) {
		velocity = MAX_VELOCITY;
	} else if (velocity < MIN_VELOCITY) {
		velocity = MIN_VELOCITY;
	}

	if (fabs(error) > 1e-3) {
		dJointSetHingeParam(joint_, dParamVel, velocity);
	} else {
		dJointSetHingeParam(joint_, dParamVel, 0);
	}
}

void ServoMotor::setVelocity(float velocity) {

	if (velocity > 1) {
		velocity = 1;
	} else if (velocity < 0) {
		velocity = 0;
	}

	velocity = MIN_VELOCITY + velocity * (MAX_VELOCITY - MIN_VELOCITY);

	if (abs(velocity) < 1e-5) {
		dJointSetHingeParam(joint_, dParamVel, 0);
	} else {
		dJointSetHingeParam(joint_, dParamVel, velocity);
	}
}

bool ServoMotor::isVelocityDriven() {
	return isVelocityDriven_;
}

}
