/*
 * @(#) BoxObstacle.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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
#include "model/objects/BoxObstacle.h"
#include "utils/RobogenUtils.h"

namespace robogen {

BoxObstacle::BoxObstacle(dWorldID odeWorld, dSpaceID odeSpace,
		const osg::Vec3& pos, const osg::Vec3& size, float density,
		const osg::Vec3& rotationAxis, float rotationAngle) :
		size_(size) {


	if (density >= RobogenUtils::OSG_EPSILON_2){
		// if not fixed, create body
		box_ = dBodyCreate(odeWorld);
		dMass massOde;
		dMassSetBox(&massOde, density, size.x(), size.y(), size.z());
		dBodySetMass(box_, &massOde);
	} else{
		// otherwise make body 0
		box_ = 0;

	}
	boxGeom_ = dCreateBox(odeSpace, size.x(), size.y(), size.z());
	dGeomSetBody(boxGeom_, box_);
	dGeomSetPosition(boxGeom_, pos.x(), pos.y(), pos.z());
	// for some reason body/geom position do not get tied together as they
	// should, so we set the body position as well, and use it when getting
	// the position on non-stationary bodies
	if (box_ != 0)
		dBodySetPosition(box_, pos.x(), pos.y(), pos.z());

	if (rotationAngle >= RobogenUtils::OSG_EPSILON_2){
		osg::Quat rotation;
		rotation.makeRotate(osg::DegreesToRadians(rotationAngle),rotationAxis);
		dQuaternion quatOde;
		quatOde[0] = rotation.w();
		quatOde[1] = rotation.x();
		quatOde[2] = rotation.y();
		quatOde[3] = rotation.z();
		dGeomSetQuaternion(boxGeom_, quatOde);

	}
}

BoxObstacle::~BoxObstacle() {
}

void BoxObstacle::remove() {
	dGeomDestroy(boxGeom_);
	if (box_ != 0) {
		for(int i=0; i< dBodyGetNumJoints(box_); i++) {
			dJointDestroy(dBodyGetJoint(box_, i));
		}
		dBodyDestroy(box_);
	}
}

const osg::Vec3 BoxObstacle::getPosition() {
	if (box_!= 0) {
		const dReal* pos = dBodyGetPosition(box_);
		return osg::Vec3(pos[0], pos[1], pos[2]);
	}
	const dReal* pos = dGeomGetPosition(boxGeom_);
	return osg::Vec3(pos[0], pos[1], pos[2]);
}

const osg::Quat BoxObstacle::getAttitude() {
	dQuaternion boxQuat;
	dGeomGetQuaternion(boxGeom_, boxQuat);
	return osg::Quat(boxQuat[1], boxQuat[2], boxQuat[3], boxQuat[0]);

}

const osg::Vec3 BoxObstacle::getSize() {
	return size_;
}

void BoxObstacle::getAABB(double& minX, double& maxX, double& minY,
		double& maxY, double& minZ, double& maxZ) {

	dReal aabb[6];
	dGeomGetAABB(boxGeom_, aabb);
	minX = aabb[0];
	maxX = aabb[1];
	minY = aabb[2];
	maxY = aabb[3];
	minZ = aabb[4];
	maxZ = aabb[5];
}

}
