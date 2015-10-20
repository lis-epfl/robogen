/*
 * @(#) AbstractBody.cpp   1.0   September 25, 2015
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


#include "AbstractBody.h"
#include "CompositeBody.h"

namespace robogen {

boost::shared_ptr<AbstractBody> AbstractBody::getRoot() {
	boost::shared_ptr<AbstractBody> root = shared_from_this();
	while(root->getParent())
		root = root->getParent();
	return root;
}


void AbstractBody::setPosition(osg::Vec3 position) {
	dBodySetPosition(getBody(), position.x(), position.y(), position.z());
}


void AbstractBody::setAttitude(osg::Quat attitude) {
	dQuaternion quatOde;
	quatOde[0] = attitude.w();
	quatOde[1] = attitude.x();
	quatOde[2] = attitude.y();
	quatOde[3] = attitude.z();

	dBodySetQuaternion(getBody(), quatOde);
}



}
