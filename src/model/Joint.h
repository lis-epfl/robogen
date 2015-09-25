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
#include <map>

namespace robogen {

class PhysicalBody;

class Joint : public boost::enable_shared_from_this<Joint> {
public:

	enum JointType {
		HINGE = 0,
		FIXED
	};

	// error free constructor
	inline Joint() {
		joint_ = NULL;
		fback_ = NULL;
	}
	// create a hinge
	void createHinge(dWorldID world, boost::shared_ptr<PhysicalBody> bodyA,
		  boost::shared_ptr<PhysicalBody> bodyB,
		  osg::Vec3 axis, osg::Vec3 anchor, dJointGroupID jointGroup=0);
	// create fixed
	void createFixed(dWorldID world, boost::shared_ptr<PhysicalBody> bodyA,
			  boost::shared_ptr<PhysicalBody> bodyB, dJointGroupID jointGroup=0);

	void reconnect();

	void reset();

	~Joint();

	inline dJointID getJoint() { return joint_; }
	inline JointType getType() { return type_; }

	inline boost::weak_ptr<PhysicalBody> getBodyA() { return bodyA_; }
	inline boost::weak_ptr<PhysicalBody> getBodyB() { return bodyB_; }


	inline const osg::Vec3 &getAnchor() { return anchor_; }
	inline const osg::Vec3 &getHingeAxis() { return hingeAxis_; }

	void updateAxisAndAngle();

	inline dWorldID getWorld() { return world_; }

	void setParam(unsigned int param, double value);

	void setFeedback(dJointFeedback *fback);

private :
	void updateJointParams();


	dJointID joint_;
	boost::weak_ptr<PhysicalBody> bodyA_, bodyB_;
	dJointGroupID jointGroup_;
	JointType type_;
	osg::Vec3 anchor_, hingeAxis_;
	dWorldID world_;
	std::map<unsigned int, double> params_;
	dJointFeedback *fback_;

};


}




#endif /* JOINT_H_ */
