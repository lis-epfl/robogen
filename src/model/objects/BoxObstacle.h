/*
 * @(#) BoxObstacle.h   1.0   Mar 13, 2013
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
#ifndef ROBOGEN_BOX_OBSTACLE_H_
#define ROBOGEN_BOX_OBSTACLE_H_

#include <osg/Quat>
#include <osg/Vec3>
#include "model/PositionObservable.h"
#include "Robogen.h"

namespace robogen {

/**
 * A simple obstacle made of a box of specified size
 */
class BoxObstacle : public PositionObservable {

public:

	/**
	 * Density of box obstacles
	 */
	static const float DENSITY;

	/**
	 * Initializes a box obstacle
	 */
	BoxObstacle(dWorldID odeWorld, dSpaceID odeSpace, const osg::Vec3& pos, const osg::Vec3& size);

	/**
	 * Destructor
	 */
	virtual ~BoxObstacle();

	/**
	 * Inherited from PositionObservable
	 */
	virtual const osg::Vec3 getPosition();
	virtual const osg::Quat getAttitude();

	/**
	 * @return the box size
	 */
	const osg::Vec3 getSize();

private:

	/**
	 * The box
	 */
	dBodyID box_;

	/**
	 * The box size
	 */
	osg::Vec3 size_;

};

}


#endif /* ROBOGEN_BOX_OBSTACLE_H_ */
