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

// Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100
const float ServoMotor::DEFAULT_MAX_FORCE_ROTATIONAL = 4 * 9.81 / 100;
const float ServoMotor::DEFAULT_MAX_FORCE_SERVO = 1.8 * 9.81 / 100;

const float ServoMotor::MIN_POS_RAD = -(M_PI / 2) * 6 / 9;
const float ServoMotor::MAX_POS_RAD = (M_PI / 2) * 6 / 9;

// 100 rpm converted to rad/s
const float ServoMotor::MIN_VELOCITY = -(100.0/60.0) * 2 * M_PI;
const float ServoMotor::MAX_VELOCITY = (100.0/60.0) * 2 * M_PI;

// TODO find what this should be before burnout is likely
const int ServoMotor::MAX_DIRECTION_SHIFTS_PER_SECOND = 20;

ServoMotor::ServoMotor(dJointID joint, float maxForce, float gain,
		ioPair id) : Motor(id),
		joint_(joint), maxForce_(maxForce), gain_(gain),
		isVelocityDriven_(false),
		internalCounter_(0), isBurntOut_(false) {
	dJointSetHingeParam(joint_, dParamFMax, maxForce_);
}

ServoMotor::ServoMotor(dJointID joint, float maxForce,
		ioPair id) : Motor(id),
		joint_(joint), maxForce_(maxForce), gain_(0),
		isVelocityDriven_(true),
		internalCounter_(0), isBurntOut_(false) {
	dJointSetHingeParam(joint_, dParamFMax, maxForce_);
}

ServoMotor::~ServoMotor() {

}

void ServoMotor::testBurnout(float velocity, float step) {
	unsigned int history_size = ((unsigned int) (0.5/step));
	if(previousVelocities_.size() < history_size) {
		previousVelocities_.push_back(velocity);
	} else {
		previousVelocities_[internalCounter_ % history_size] = velocity;
	}
	internalCounter_++;

	unsigned int numDirectionFlips = 0;
	for(unsigned int i=1; i<previousVelocities_.size(); i++) {
		if (	(previousVelocities_[i % history_size] > 0.0 &&
				previousVelocities_[(i-1) % history_size] < 0.0) ||
				(previousVelocities_[i % history_size] < 0.0 &&
				previousVelocities_[(i-1) % history_size] > 0.0)
				) {
			numDirectionFlips++;
		}
	}

	// considering previous half-second of simulated time
	if ((numDirectionFlips * 2) > MAX_DIRECTION_SHIFTS_PER_SECOND) {
		std::cout << "motor burnt out!" << std::endl;
		isBurntOut_ = true;
	}
}

void ServoMotor::setPosition(float position, float step) {

	if (isBurntOut_) {
		dJointSetHingeParam(joint_, dParamVel, 0);
		return;
	}


	if (position > 1) {
		position = 1;
	} else if (position < 0) {
		position = 0;
	}

	position = MIN_POS_RAD + position * (MAX_POS_RAD - MIN_POS_RAD);

	dReal curPosition = dJointGetHingeAngle(joint_);
	dReal error = curPosition - position;

	dReal velocity = -gain_ * error / step;
	if (velocity > MAX_VELOCITY) {
		velocity = MAX_VELOCITY;
	} else if (velocity < MIN_VELOCITY) {
		velocity = MIN_VELOCITY;
	}

	if (fabs(velocity) > 1e-3) {
		dJointSetHingeParam(joint_, dParamVel, velocity);
	} else {
		dJointSetHingeParam(joint_, dParamVel, 0);
	}

	testBurnout(velocity, step);

}

void ServoMotor::setVelocity(float velocity, float step) {

	if (isBurntOut_) {
		dJointSetHingeParam(joint_, dParamVel, 0);
		return;
	}


	if (velocity > 1) {
		velocity = 1;
	} else if (velocity < 0) {
		velocity = 0;
	}

	velocity = MIN_VELOCITY + velocity * (MAX_VELOCITY - MIN_VELOCITY);

	if (fabs(velocity) < 1e-5) {
		dJointSetHingeParam(joint_, dParamVel, 0);
	} else {
		dJointSetHingeParam(joint_, dParamVel, velocity);
	}

	testBurnout(velocity, step);
}

bool ServoMotor::isVelocityDriven() {
	return isVelocityDriven_;
}

bool ServoMotor::isBurntOut() {
	return isBurntOut_;
}



}
