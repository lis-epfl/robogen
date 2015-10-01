/*
 * @(#) Joint.cpp   1.0   September 16, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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

#include "Joint.h"
#include "SimpleBody.h"
#include "CompositeBody.h"

namespace robogen {


void Joint::createHinge(dWorldID world, boost::shared_ptr<SimpleBody> bodyA,
		  boost::shared_ptr<SimpleBody> bodyB,
		  osg::Vec3 axis, osg::Vec3 anchor, dJointGroupID jointGroup) {

	type_ = HINGE;

	world_ = world;
	anchor_ = anchor;
	hingeAxis_ = axis;
	jointGroup_ = jointGroup;

	// now add to body to maintain shared pointer
	bodyA_ = bodyA;
	bodyB_ = bodyB;

	bodyA_.lock()->addJoint(shared_from_this());
	bodyB_.lock()->addJoint(shared_from_this());

	this->reconnect();
}


void Joint::createFixed(dWorldID world, boost::shared_ptr<SimpleBody> bodyA,
			  boost::shared_ptr<SimpleBody> bodyB, dJointGroupID jointGroup) {

	type_ = FIXED;

	world_ = world;
	jointGroup_ = jointGroup;



	// now add to body to maintain shared pointer
	bodyA_ = bodyA;
	bodyB_ = bodyB;

	bodyA_.lock()->addJoint(shared_from_this());
	bodyB_.lock()->addJoint(shared_from_this());

	this->reconnect();
}

void Joint::createUniversal(dWorldID world,
				boost::shared_ptr<SimpleBody> bodyA,
				boost::shared_ptr<SimpleBody> bodyB,
				osg::Vec3 axis1, osg::Vec3 axis2, osg::Vec3 anchor,
				dJointGroupID jointGroup) {

	type_ = UNIVERSAL;

	world_ = world;
	anchor_ = anchor;
	universalAxis1_ = axis1;
	universalAxis2_ = axis2;
	jointGroup_ = jointGroup;

	// now add to body to maintain shared pointer
	bodyA_ = bodyA;
	bodyB_ = bodyB;

	bodyA_.lock()->addJoint(shared_from_this());
	bodyB_.lock()->addJoint(shared_from_this());

	this->reconnect();
}


void Joint::reconnect() {
	if (joint_) {
		/*
		if (type_ == HINGE) {
			dVector3 anchor;
			dJointGetHingeAnchor(joint_, anchor);
			std::cout << "-*-*-*-*-*-*-*-*-*-*- ode anchor " <<
					anchor[0] << " " << anchor[1] << " " << anchor[2]
								<< std::endl;
			dVector3 axis;
			dJointGetHingeAxis(joint_, axis);
			std::cout << "-*-*-*-*-*-*-*-*-*-*- ode axis " <<
					axis[0] << " " << axis[1] << " " << axis[2]
								<< std::endl;
		}
		*/
		dJointDestroy(joint_);
		joint_ = NULL;
	}
#ifdef DEBUG_RECONNECT
	std::cout << "-*-*-*-*-*-*-*-*-*-*- attaching " << bodyA_.lock()->getBody()
			<< " " << bodyB_.lock()->getBody() << std::endl;
#endif
	if (type_ == FIXED) {
		joint_ = dJointCreateFixed(world_, jointGroup_);
		dJointAttach(joint_, bodyA_.lock()->getBody(), bodyB_.lock()->getBody());
		dJointSetFixed(joint_);
	} else if (type_ == HINGE) {
#ifdef DEBUG_RECONNECT
		std::cout << "-*-*-*-*-*-*-*-*-*-*- axis " <<
				hingeAxis_[0] << " " << hingeAxis_[1] << " " << hingeAxis_[2]
                << std::endl;
		std::cout << "-*-*-*-*-*-*-*-*-*-*- anchor " <<
				anchor_[0] << " " << anchor_[1] << " " << anchor_[2]
				<< std::endl;
#endif
		joint_ = dJointCreateHinge(world_, jointGroup_);
		dJointAttach(joint_, bodyA_.lock()->getBody(), bodyB_.lock()->getBody());
		dJointSetHingeAxis(joint_, hingeAxis_[0], hingeAxis_[1], hingeAxis_[2]);
		dJointSetHingeAnchor(joint_, anchor_[0], anchor_[1], anchor_[2]);

	} else if(type_ == UNIVERSAL) {
#ifdef DEBUG_RECONNECT
		std::cout << "-*-*-*-*-*-*-*-*-*-*- axis1 " <<
				universalAxis1_[0] << " " << universalAxis1_[1] << " "
				<< universalAxis1_[2]
                << std::endl;
		std::cout << "-*-*-*-*-*-*-*-*-*-*- axis2 " <<
				universalAxis2_[0] << " " << universalAxis2_[1] << " "
				<< universalAxis2_[2]
                << std::endl;
		std::cout << "-*-*-*-*-*-*-*-*-*-*- anchor " <<
				anchor_[0] << " " << anchor_[1] << " " << anchor_[2]
				<< std::endl;
#endif
		joint_ = dJointCreateUniversal(world_, jointGroup_);
		dJointAttach(joint_, bodyA_.lock()->getBody(), bodyB_.lock()->getBody());
		dJointSetUniversalAxis1(joint_, universalAxis1_[0], universalAxis1_[1],
				universalAxis1_[2]);
		dJointSetUniversalAxis2(joint_, universalAxis2_[0], universalAxis2_[1],
						universalAxis2_[2]);
		dJointSetUniversalAnchor(joint_, anchor_[0], anchor_[1], anchor_[2]);

	} else {
		std::cerr << "INVALID JOINT TYPE" << std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	updateJointParams();
}

void Joint::reset() {
	if (joint_) {
		dJointDestroy(joint_);
		joint_ = NULL;
	}
	bodyA_.lock()->removeJoint(shared_from_this());
	bodyB_.lock()->removeJoint(shared_from_this());
}

void Joint::updateAxisAndAngle() {
	if (type_ == HINGE) {
		dVector3 axis, anchor;
		dJointGetHingeAxis(joint_, axis);
		dJointGetHingeAnchor(joint_, anchor);
		for(unsigned int i=0; i<3; ++i) {
			hingeAxis_[i] = axis[i];
			anchor_[i] = anchor[i];
		}
	}
}

void Joint::updateJointParams() {
	for(std::map<unsigned int, double>::iterator it = params_.begin();
			it != params_.end(); ++it) {
		if(type_ == FIXED) {
			dJointSetFixedParam(joint_, it->first, it->second);
		} else if (type_ == HINGE) {
			dJointSetHingeParam(joint_, it->first, it->second);
		} else if (type_ == UNIVERSAL) {
			dJointSetUniversalParam(joint_, it->first, it->second);
		}
	}
	if(fback_)
		dJointSetFeedback(joint_, fback_);
}

void Joint::setParam(unsigned int param, double value) {
	params_[param] = value;
	updateJointParams();
}

void Joint::setFeedback(dJointFeedback *fback) {
	fback_ = fback;
	updateJointParams();
}

Joint::~Joint() {
	//reset();
	//dJointDestroy(joint_);
}

}
