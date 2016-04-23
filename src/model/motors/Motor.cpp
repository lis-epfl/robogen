/*
 * @(#) Motor.cpp   1.0   Sep 10, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
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

#include "model/motors/Motor.h"

namespace robogen{

Motor::Motor(ioPair id, boost::shared_ptr<Joint> joint, float maxForce,
		int maxDirectionShiftsPerSecond) : id_(id),
		joint_(joint), internalCounter_(0), isBurntOut_(false),
		maxForce_(maxForce),
		maxDirectionShiftsPerSecond_(maxDirectionShiftsPerSecond) {

	// need to set these on our joint object so that they get
	// reset if the joint is reconnected
	joint_->setParam(dParamFMax, maxForce_);
	joint_->setFeedback( &fback_ );
	shouldStep_ = false;
#ifndef OLD_SERVO_MODEL
	shouldStep_ = true;
#endif
}



ioPair Motor::getId(){
	return id_;
}


bool Motor::isBurntOut() {
	return isBurntOut_;
}

void Motor::setMaxDirectionShiftsPerSecond(int
		maxDirectionShiftsPerSecond) {
	maxDirectionShiftsPerSecond_ = maxDirectionShiftsPerSecond;
}

dReal Motor::getTorque() {
	// code from Jeff Shim

	osg::Vec3 torque1(fback_.t1[0], fback_.t1[1], fback_.t1[2] );
	osg::Vec3 torque2(fback_.t2[0], fback_.t2[1], fback_.t2[2] );
	osg::Vec3 force1(fback_.f1[0], fback_.f1[1], fback_.f1[2] );
	osg::Vec3 force2(fback_.f2[0], fback_.f2[1], fback_.f2[2] );

	const double* p1 = dBodyGetPosition( dJointGetBody(joint_->getJoint(),0) );
	const double* p2 = dBodyGetPosition( dJointGetBody(joint_->getJoint(),1) );

	osg::Vec3 pos1(p1[0], p1[1], p1[2]);
	osg::Vec3 pos2(p2[0], p2[1], p2[2]);


	dVector3 odeAnchor;
	dJointGetHingeAnchor ( joint_->getJoint(), odeAnchor );
	osg::Vec3 anchor(odeAnchor[0], odeAnchor[1], odeAnchor[2]);


	osg::Vec3 ftorque1 = torque1 - (force1^(pos1-anchor));// torq by motor = total torq - constraint torq
	osg::Vec3 ftorque2 = torque2 - (force2^(pos2-anchor));// opposite direction - use if this is necessary

	dVector3 odeAxis;
	dJointGetHingeAxis ( joint_->getJoint(), odeAxis);
	osg::Vec3 axis(odeAxis[0], odeAxis[1], odeAxis[2] );
	axis.normalize();

	double torque =  ftorque1 * axis;

	//printf ("torque: % 1.10f\n", torque);
	return torque;
}

dReal Motor::getVelocity() {
	return dJointGetHingeAngleRate(joint_->getJoint());

}

dReal Motor::getPosition() {
	return dJointGetHingeAngle(joint_->getJoint());
}

void Motor::testBurnout(float stepSize) {
	unsigned int historySize = ((unsigned int) (0.5/stepSize));
	double signal = getSignal();

	if(previousSignals_.size() < historySize) {
		previousSignals_.push_back(signal);
	} else {
		previousSignals_[internalCounter_ % historySize] = signal;
	}
	internalCounter_++;

	if(previousSignals_.size() < 2)
		return;

	int numDirectionFlips = getNumDirectionFlips();
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
