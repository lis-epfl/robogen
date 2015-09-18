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

#define OLD_SERVO_MODEL


namespace robogen {


//#ifdef OLD_SERVO_MODEL
const float ServoMotor::DEFAULT_GAIN = 0.5;
//#else
// reproduce above when running physics at 0.005
//const float ServoMotor::DEFAULT_GAIN = 0.0625;
//#endif

// Expressed in Newton*m from kg-cm = ((kg-cm)*g)/100
const float ServoMotor::DEFAULT_MAX_FORCE_ROTATIONAL = 4 * 9.81 / 100;
const float ServoMotor::DEFAULT_MAX_FORCE_SERVO = 1.8 * 9.81 / 100;

const float ServoMotor::MIN_POS_RAD = -(45 * M_PI / 180);
const float ServoMotor::MAX_POS_RAD = (45 * M_PI / 180);

// 50 rpm converted to rad/s
// 	note, max velocity should really be 100 rpm, but only with 0 torque
// 	50 rpms is a compromise
const float ServoMotor::MIN_VELOCITY = -(50.0/60.0) * 2 * M_PI;
const float ServoMotor::MAX_VELOCITY = (50.0/60.0) * 2 * M_PI;

void ServoMotor::init() {
	dJointSetHingeParam(joint_, dParamFMax, maxForce_);
	dJointSetFeedback( joint_, &fback_ );
	shouldStep_ = false;
#ifndef OLD_SERVO_MODEL
	shouldStep_ = true;
#endif
}

ServoMotor::ServoMotor(boost::shared_ptr<Joint> joint,
		float maxForce, float gain,
		ioPair id, int maxDirectionShiftsPerSecond) : Motor(id),
		joint_(joint->getJoint()), maxForce_(maxForce), gain_(gain),
		isVelocityDriven_(false),
		internalCounter_(0), isBurntOut_(false),
		maxDirectionShiftsPerSecond_(maxDirectionShiftsPerSecond) {
	init();
}

ServoMotor::ServoMotor(boost::shared_ptr<Joint> joint, float maxForce,
		ioPair id, int maxDirectionShiftsPerSecond) : Motor(id),
		joint_(joint->getJoint()), maxForce_(maxForce), gain_(0),
		isVelocityDriven_(true),
		internalCounter_(0), isBurntOut_(false),
		maxDirectionShiftsPerSecond_(maxDirectionShiftsPerSecond) {
	init();
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

void ServoMotor::setDesiredVelocity(float velocity, float stepSize) {
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

void ServoMotor::step(float stepSize) {

	if(!shouldStep_)
		return;

	if (isBurntOut_) {
		dJointSetHingeParam(joint_, dParamVel, 0);
		return;
	}

	if (isVelocityDriven_) {
		if (fabs(desiredVelocity_) < 1e-5) {
			dJointSetHingeParam(joint_, dParamVel, 0);
		} else {
			dJointSetHingeParam(joint_, dParamVel, desiredVelocity_);
		}
	} else {
		dReal curPosition = dJointGetHingeAngle(joint_);
		dReal error = curPosition - desiredPosition_;

		dReal velocity = -gain_ * error / stepSize;

		if (velocity > MAX_VELOCITY) {
			velocity = MAX_VELOCITY;
		} else if (velocity < MIN_VELOCITY) {
			velocity = MIN_VELOCITY;
		}

		if (fabs(velocity) > 1e-5) {
			dJointSetHingeParam(joint_, dParamVel, velocity);
		} else {
			dJointSetHingeParam(joint_, dParamVel, 0);
		}
	}

}

bool ServoMotor::isVelocityDriven() {
	return isVelocityDriven_;
}

bool ServoMotor::isBurntOut() {
	return isBurntOut_;
}

void ServoMotor::setMaxDirectionShiftsPerSecond(int
		maxDirectionShiftsPerSecond) {
	maxDirectionShiftsPerSecond_ = maxDirectionShiftsPerSecond;
}

dReal ServoMotor::getTorque() {
	// code from Jeff Shim

	osg::Vec3 torque1(fback_.t1[0], fback_.t1[1], fback_.t1[2] );
	osg::Vec3 torque2(fback_.t2[0], fback_.t2[1], fback_.t2[2] );
	osg::Vec3 force1(fback_.f1[0], fback_.f1[1], fback_.f1[2] );
	osg::Vec3 force2(fback_.f2[0], fback_.f2[1], fback_.f2[2] );

	const double* p1 = dBodyGetPosition( dJointGetBody(joint_,0) );
	const double* p2 = dBodyGetPosition( dJointGetBody(joint_,1) );

	osg::Vec3 pos1(p1[0], p1[1], p1[2]);
	osg::Vec3 pos2(p2[0], p2[1], p2[2]);


	dVector3 odeAnchor;
	dJointGetHingeAnchor ( joint_, odeAnchor );
	osg::Vec3 anchor(odeAnchor[0], odeAnchor[1], odeAnchor[2]);


	osg::Vec3 ftorque1 = torque1 - (force1^(pos1-anchor));// torq by motor = total torq - constraint torq
	osg::Vec3 ftorque2 = torque2 - (force2^(pos2-anchor));// opposite direction - use if this is necessary

	dVector3 odeAxis;
	dJointGetHingeAxis ( joint_, odeAxis);
	osg::Vec3 axis(odeAxis[0], odeAxis[1], odeAxis[2] );
	axis.normalize();

	double torque =  ftorque1 * axis;

	//printf ("torque: % 1.10f\n", torque);
	return torque;
}

dReal ServoMotor::getVelocity() {
	return dJointGetHingeAngleRate(joint_);

}

dReal ServoMotor::getPosition() {
	return dJointGetHingeAngle(joint_);
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void ServoMotor::testBurnout(float stepSize) {
	unsigned int historySize = ((unsigned int) (0.5/stepSize));
	double signal;
	if (isVelocityDriven()) {
		signal = desiredVelocity_;
	} else {
		signal = desiredPosition_;
	}

	if(previousSignals_.size() < historySize) {
		previousSignals_.push_back(signal);
	} else {
		previousSignals_[internalCounter_ % historySize] = signal;
	}
	internalCounter_++;

	unsigned int numDirectionFlips = 0;

	if(isVelocityDriven()) {
		int previousSign = sgn(previousSignals_[0]);
		for(unsigned int i=1; i<previousSignals_.size(); i++) {
			int currentSign = sgn(previousSignals_[i]);
			if (currentSign != 0) {
				if (previousSign * currentSign < 0)
					numDirectionFlips++;
				previousSign = currentSign;
			}
		}
	} else {
		int previousSign = sgn(previousSignals_[1] - previousSignals_[0]);
		for(unsigned int i=2; i<previousSignals_.size(); i++) {
			int currentSign = sgn(previousSignals_[i] - previousSignals_[i-1]);
			if (currentSign != 0) {
				if (previousSign * currentSign < 0)
					numDirectionFlips++;
				previousSign = currentSign;
			}
		}
	}

	// considering previous half-second of simulated time
	if ((numDirectionFlips * 2) > maxDirectionShiftsPerSecond_) {
		std::cout << "motor burnt out!" << std::endl;
		for(unsigned int i=0; i<previousSignals_.size(); i++) {
			std::cout << previousSignals_[i] << std::endl;
		}
		isBurntOut_ = true;
	}
}

}
