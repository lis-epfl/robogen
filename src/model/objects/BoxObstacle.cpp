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

namespace robogen {

const float BoxObstacle::DENSITY = 0.00175;

BoxObstacle::BoxObstacle(dWorldID odeWorld, dSpaceID odeSpace,
		const osg::Vec3& pos, const osg::Vec3& size) :
		size_(size) {

	box_ = dBodyCreate(odeWorld);
	dMass massOde;
	dMassSetBox(&massOde, DENSITY, size.x(), size.y(), size.z());
	dBodySetMass(box_, &massOde);
	dxGeom* g = dCreateBox(odeSpace, size.x(), size.y(), size.z());
	dBodySetPosition(box_, pos.x(), pos.y(), pos.z());
	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());
	dGeomSetBody(g, box_);
}

BoxObstacle::~BoxObstacle() {

}

const osg::Vec3 BoxObstacle::getPosition() {
	const dReal* pos = dBodyGetPosition(box_);
	return osg::Vec3(pos[0], pos[1], pos[2]);
}

const osg::Quat BoxObstacle::getAttitude() {
	const dReal* boxQuat = dBodyGetQuaternion(box_);
	return (osg::Quat(boxQuat[1], boxQuat[2], boxQuat[3], boxQuat[0]));
}

const osg::Vec3 BoxObstacle::getSize() {
	return size_;
}

}
