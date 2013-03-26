/*
 * @(#) Mesh.h   1.0   Feb 6, 2013
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
#ifndef RENDERABLE_JOINT_H_
#define RENDERABLE_JOINT_H_

namespace robogen {

class RenderableJoint {

public:

	virtual ~RenderableJoint() {}

	/**
	 * Provide one joint angle for each degree of freeedom of the joint
	 */
	virtual void operateJoint(std::vector<double> jointAngles) = 0;

};

}


#endif /* RENDERABLE_JOINT_H_ */
