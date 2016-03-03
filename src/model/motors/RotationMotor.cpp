/*
 * @(#) RotationMotor.cpp   1.0   Mar 3, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
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


#include "RotationMotor.h"

namespace robogen {

// Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100
const float RotationMotor::DEFAULT_MAX_FORCE_ROTATIONAL = 4 * 9.81 / 100;

// 60 rpm converted to rad/s
const float RotationMotor::MIN_VELOCITY = -2 * M_PI;
const float RotationMotor::MAX_VELOCITY = 2 * M_PI;


RotationMotor::RotationMotor(ioPair id, boost::shared_ptr<Joint> joint,
		float maxForce, int maxDirectionShiftsPerSecond) :
				Motor(id, joint, maxForce, maxDirectionShiftsPerSecond),
				desiredVelocity_(0) {

}

RotationMotor::~RotationMotor() {

}


void RotationMotor::setDesiredVelocity(float velocity, float stepSize) {
	if (velocity > 1) {
		velocity = 1;
	} else if (velocity < 0) {
		velocity = 0;
	}

	desiredVelocity_ = MIN_VELOCITY + velocity * (MAX_VELOCITY - MIN_VELOCITY);
	if(maxDirectionShiftsPerSecond_ != -1)
		testBurnout(stepSize);

#ifdef OLD_SERVO_MODEL
	shouldStep_ = true;
	step(stepSize);
	shouldStep_ = false;
#endif
}


void RotationMotor::step(float stepSize) {

	if(!shouldStep_)
		return;

	if (isBurntOut_) {
		dJointSetHingeParam(joint_->getJoint(), dParamVel, 0);
		return;
	}

	if (fabs(desiredVelocity_) < 1e-5) {
		dJointSetHingeParam(joint_->getJoint(), dParamVel, 0);
	} else {
		dJointSetHingeParam(joint_->getJoint(), dParamVel, desiredVelocity_);
	}

}


int RotationMotor::getNumDirectionFlips() {
	int numDirectionFlips = 0;
	int previousSign = sgn(previousSignals_[0]);
	for(unsigned int i=1; i<previousSignals_.size(); i++) {
		int currentSign = sgn(previousSignals_[i]);
		if (currentSign != 0) {
			if (previousSign * currentSign < 0)
				numDirectionFlips++;
			previousSign = currentSign;
		}
	}
	return numDirectionFlips;
}


}
