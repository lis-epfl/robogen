/*
 * @(#) SimpleBody.cpp   1.0   September 16, 2015
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

#include "Model.h"
#include "SimpleBody.h"

namespace robogen {

SimpleBody::SimpleBody(boost::shared_ptr<Model> model, dMass mass,
		dGeomID geom, const osg::Vec3& pos, const osg::Quat& attitude) :
		model_(model),  mass_(mass), geom_(geom) {

	body_ = dBodyCreate(model->getPhysicsWorld());

	dBodySetMass(body_, &mass_);
	dGeomSetBody(geom, body_);

	for(unsigned int i=0; i<3; ++i) specifiedPosition_[i] = pos[i];
	for(unsigned int i=0; i<4; ++i) specifiedAttitude_[i] = attitude[i];

	setPosition(pos);
	setAttitude(attitude);
}

void SimpleBody::removeJoint(boost::shared_ptr<Joint> joint) {
	joints_.erase(std::remove(joints_.begin(), joints_.end(), joint),
				joints_.end());
}

void SimpleBody::addJoint(boost::shared_ptr<Joint> joint) {
	joints_.push_back(joint);
}

void SimpleBody::clearJoints() {
	joints_.clear();
}

osg::Vec3 SimpleBody::getLocalPosition() {
	  // Get local position/orientation of component within geom transform
	 const dReal* pLocalPos = dGeomGetOffsetPosition (geom_);
	 return osg::Vec3(pLocalPos[0], pLocalPos[1], pLocalPos[2]);

}

osg::Vec3 SimpleBody::getPosition() {
	 // Get world position/orientation of parent body
#if 0
	 const dReal* pBodyPos = dBodyGetPosition(body_);
	 const dReal* pBodyRotMat = dBodyGetRotation(body_);

	 // Get local position/orientation of component within geom transform
	 const dReal* pLocalPos = dGeomGetOffsetPosition (geom);
	 const dReal* pLocalRotMat = dGeomGetOffsetPosition (geom);

	 // Calculate world space position of component by using the body position/orientation
	 dVector3 worldPos;
	 dMatrix3 worldRotMat;
	 dMULTIPLY0_331( worldPos, pBodyRotMat, pLocalPos );
	 worldPos[0] += pBodyPos[0];
	 worldPos[1] += pBodyPos[1];
	 worldPos[2] += pBodyPos[2];
	 dMULTIPLY0_333( worldRotMat, pBodyRotMat, pLocalRotMat );

	 return osg::Vec3(worldPos[0], worldPos[1], worldPos[2]);

	 // And if you want a quaternion instead of a matrix...
	 //dQuaternion worldQuat;
	 //dQfromR(worldQuat, worldRotMat);
#endif
	 const dReal* pos = dGeomGetPosition(geom_);
	 //printf("pos: % 02.7f % 02.7f % 02.7f\n", pos[0], pos[1], pos[2]);
	 return osg::Vec3(pos[0], pos[1], pos[2]);

}

osg::Quat SimpleBody::getLocalAttitude() {

	 dQuaternion localQuat;
	 dGeomGetOffsetQuaternion(geom_, localQuat);
	 return (osg::Quat(localQuat[1], localQuat[2], localQuat[3], localQuat[0]));

}

osg::Quat SimpleBody::getAttitude() {
	 // Get world position/orientation of parent body
#if 0
	 const dReal* pBodyPos = dBodyGetPosition(body_);
	 const dReal* pBodyRotMat = dBodyGetRotation(body_);

	 // Get local position/orientation of component within geom transform
	 const dReal* pLocalPos = dGeomGetOffsetPosition (geom);
	 const dReal* pLocalRotMat = dGeomGetOffsetRotation (geom);

	 // Calculate world space position of component by using the body position/orientation
	 dVector3 worldPos;
	 dMatrix3 worldRotMat;
	 dMULTIPLY0_331( worldPos, pBodyRotMat, pLocalPos );
	 worldPos[0] += pBodyPos[0];
	 worldPos[1] += pBodyPos[1];
	 worldPos[2] += pBodyPos[2];
	 dMULTIPLY0_333( worldRotMat, pBodyRotMat, pLocalRotMat );


	 dQuaternion worldQuat;
	 dQfromR(worldQuat, worldRotMat);
	 return (osg::Quat(worldQuat[1], worldQuat[2], worldQuat[3], worldQuat[0]));
#endif
	 dQuaternion quat;
	 dGeomGetQuaternion(geom_, quat);
	 return (osg::Quat(quat[1], quat[2], quat[3], quat[0]));
}


const boost::weak_ptr<Model>& SimpleBody::getModel() {
		if (!model_.lock()) {
			std::cout << "bad model!" << std::endl;
			exitRobogen(EXIT_FAILURE);
		}
		return model_;
	}

}
