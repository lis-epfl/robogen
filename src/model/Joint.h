/*
 * @(#) Joint.h   1.0   September 16, 2015
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

#ifndef ROBOGEN_JOINT_H_
#define ROBOGEN_JOINT_H_

#include "Robogen.h"

#include <boost/enable_shared_from_this.hpp>
#include <map>

namespace robogen {

class AbstractBody;

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
	void createHinge(dWorldID world, boost::shared_ptr<AbstractBody> bodyA,
		  boost::shared_ptr<AbstractBody> bodyB,
		  osg::Vec3 axis, osg::Vec3 anchor, dJointGroupID jointGroup=0);
	// create fixed
	void createFixed(dWorldID world, boost::shared_ptr<AbstractBody> bodyA,
			  boost::shared_ptr<AbstractBody> bodyB, dJointGroupID jointGroup=0);

	void reconnect();

	void reset();

	~Joint();

	inline dJointID getJoint() { return joint_; }
	inline JointType getType() { return type_; }

	inline boost::weak_ptr<AbstractBody> getBodyA() { return bodyA_; }
	inline boost::weak_ptr<AbstractBody> getBodyB() { return bodyB_; }


	inline const osg::Vec3 &getAnchor() { return anchor_; }
	inline const osg::Vec3 &getHingeAxis() { return hingeAxis_; }

	void updateAxisAndAngle();

	inline dWorldID getWorld() { return world_; }

	void setParam(unsigned int param, double value);

	void setFeedback(dJointFeedback *fback);

private :
	void updateJointParams();


	dJointID joint_;
	boost::weak_ptr<AbstractBody> bodyA_, bodyB_;
	dJointGroupID jointGroup_;
	JointType type_;
	osg::Vec3 anchor_, hingeAxis_;
	dWorldID world_;
	std::map<unsigned int, double> params_;
	dJointFeedback *fback_;

};


}




#endif /* ROBOGEN_JOINT_H_ */
