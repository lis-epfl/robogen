/*
 * Joint.h
 *
 *  Created on: Sep 17, 2015
 *      Author: auerbach
 */

#ifndef ROBOGEN_JOINT_H_
#define ROBOGEN_JOINT_H_

#include "Robogen.h"

#include <boost/enable_shared_from_this.hpp>


namespace robogen {

class PhysicalBody;

class Joint : public boost::enable_shared_from_this<Joint> {
public:

	enum JointType {
		HINGE = 0,
		FIXED
	};

	// error free constructor
	inline Joint() {}
	// create a hinge
	void createHinge(dWorldID world, boost::shared_ptr<PhysicalBody> bodyA,
		  boost::shared_ptr<PhysicalBody> bodyB,
		  osg::Vec3 axis, osg::Vec3 anchor);
	// create fixed
	void createFixed(dWorldID world, boost::shared_ptr<PhysicalBody> bodyA,
			  boost::shared_ptr<PhysicalBody> bodyB, dJointGroupID jointGroup=0);

	~Joint();

	inline dJointID getJoint() { return joint_; }
	inline JointType getType() { return type_; }

	inline boost::weak_ptr<PhysicalBody> getBodyA() { return bodyA_; }
	inline boost::weak_ptr<PhysicalBody> getBodyB() { return bodyB_; }

private :
	dJointID joint_;
	boost::weak_ptr<PhysicalBody> bodyA_, bodyB_;
	dJointGroupID jointGroup_;
	JointType type_;



};


}




#endif /* JOINT_H_ */
