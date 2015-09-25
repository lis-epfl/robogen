/*
 * PhysicalBody.cpp
 *
 *  Created on: Sep 25, 2015
 *      Author: auerbach
 */


#include "PhysicalBody.h"
#include "CompositeBody.h"

namespace robogen {

boost::shared_ptr<PhysicalBody> PhysicalBody::getRoot() {
	boost::shared_ptr<PhysicalBody> root = shared_from_this();
	while(root->getParent())
		root = root->getParent();
	return root;
}

void PhysicalBody::removeJoint(boost::shared_ptr<Joint> joint) {
	joints_.erase(std::remove(joints_.begin(), joints_.end(), joint),
				joints_.end());
}

void PhysicalBody::setPosition(osg::Vec3 position) {
	dBodySetPosition(getBody(), position.x(), position.y(), position.z());
}


void PhysicalBody::setAttitude(osg::Quat attitude) {
	dQuaternion quatOde;
	quatOde[0] = attitude.w();
	quatOde[1] = attitude.x();
	quatOde[2] = attitude.y();
	quatOde[3] = attitude.z();

	dBodySetQuaternion(getBody(), quatOde);
}



}
