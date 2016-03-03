/*
 * @(#) ServoMotor.cpp   1.0   Feb 20, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch),
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Titus Cieslewski, Joshua Auerbach
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

//#define OLD_SERVO_MODEL


namespace robogen {

//#ifdef OLD_SERVO_MODEL
const float ServoMotor::DEFAULT_GAIN = 0.5;
//#else
// reproduce above when running physics at 0.005
//const float ServoMotor::DEFAULT_GAIN = 0.0625;
//#endif

// Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100
const float ServoMotor::DEFAULT_MAX_FORCE_SERVO = 1.8 * 9.81 / 100;

const float ServoMotor::MIN_POS_RAD = -(45 * M_PI / 180);
const float ServoMotor::MAX_POS_RAD = (45 * M_PI / 180);

// 50 rpm converted to rad/s
// 	note, max velocity should really be 100 rpm, but only with 0 torque
// 	50 rpms is a compromise
const float ServoMotor::MIN_VELOCITY = -(50.0/60.0) * 2 * M_PI;
const float ServoMotor::MAX_VELOCITY = (50.0/60.0) * 2 * M_PI;



ServoMotor::ServoMotor(ioPair id, boost::shared_ptr<Joint> joint, float maxForce,
		float gain, int maxDirectionShiftsPerSecond) :
				Motor(id, joint, maxForce, maxDirectionShiftsPerSecond),
				gain_(gain), desiredPosition_(0) {
}



ServoMotor::~ServoMotor() {

}

void ServoMotor::setDesiredPosition(float position, float stepSize) {
	if (position > 1) {
		position = 1;
	} else if (position < 0) {
		position = 0;
	}

	desiredPosition_ = MIN_POS_RAD + position * (MAX_POS_RAD - MIN_POS_RAD);
	if(maxDirectionShiftsPerSecond_ != -1)
		testBurnout(stepSize);

#ifdef OLD_SERVO_MODEL
	shouldStep_ = true;
	step(stepSize);
	shouldStep_ = false;
#endif
}

void ServoMotor::step(float stepSize) {

	if(!shouldStep_)
		return;

	if (isBurntOut_) {
		dJointSetHingeParam(joint_->getJoint(), dParamVel, 0);
		return;
	}


	dReal curPosition = dJointGetHingeAngle(joint_->getJoint());
	dReal error = curPosition - desiredPosition_;

	dReal velocity = -gain_ * error / stepSize;

	if (velocity > MAX_VELOCITY) {
		velocity = MAX_VELOCITY;
	} else if (velocity < MIN_VELOCITY) {
		velocity = MIN_VELOCITY;
	}

	if (fabs(velocity) > 1e-5) {
		dJointSetHingeParam(joint_->getJoint(), dParamVel, velocity);
	} else {
		dJointSetHingeParam(joint_->getJoint(), dParamVel, 0);
	}


}

int ServoMotor::getNumDirectionFlips() {
	int numDirectionFlips = 0;
	int previousSign = sgn(previousSignals_[1] - previousSignals_[0]);
	for(unsigned int i=2; i<previousSignals_.size(); i++) {
		int currentSign = sgn(previousSignals_[i] - previousSignals_[i-1]);
		if (currentSign != 0) {
			if (previousSign * currentSign < 0)
				numDirectionFlips++;
			previousSign = currentSign;
		}
	}
	return numDirectionFlips;
}


}
