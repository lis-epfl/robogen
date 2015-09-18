/*
 * Joint.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: auerbach
 */

#include "Joint.h"
#include "PhysicalBody.h"

namespace robogen {

Joint::Joint(dWorldID world, boost::shared_ptr<PhysicalBody> bodyA,
		  boost::shared_ptr<PhysicalBody> bodyB,
		  osg::Vec3 axis, osg::Vec3 anchor) :
				  bodyA_(bodyA), bodyB_(bodyB) {

	joint_ = dJointCreateHinge(world, 0);
	dJointAttach(joint_, bodyA->getBody(), bodyB->getBody());
	dJointSetHingeAxis(joint_, axis[0], axis[1], axis[2]);
	dJointSetHingeAnchor(joint_, anchor[0], anchor[1], anchor[2]);

	// now add to body to maintain shared pointer
	bodyA->addJoint(shared_from_this());
	bodyB->addJoint(shared_from_this());
}


Joint::Joint(dWorldID world, boost::shared_ptr<PhysicalBody> bodyA,
			  boost::shared_ptr<PhysicalBody> bodyB, dJointGroupID jointGroup) :
				 bodyA_(bodyA), bodyB_(bodyB), jointGroup_(jointGroup) {
	joint_ = dJointCreateFixed(world, jointGroup);
	dJointAttach(joint_, bodyA->getBody(), bodyB->getBody());
	dJointSetFixed(joint_);

	bodyA->addJoint(shared_from_this());
	bodyB->addJoint(shared_from_this());
}

Joint::~Joint() {
	dJointDestroy(joint_);
}

}
