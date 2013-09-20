/*
 * @(#) ConfigurationReader.h   1.0   Mar 13, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani, Titus Cieslewski
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
#ifndef STARTPOSITION_H_
#define STARTPOSITION_H_

#include<osg/Vec2>

namespace robogen{

/**
 * Class to contain beginning modalities of robot (position, azimuth).
 */
class StartPosition{

public:
	StartPosition();

	/**
	 * Initializes start position to given position and azimuth
	 */
	bool init(osg::Vec2 position, float azimuth);

	/**
	 * @return starting position
	 */
	osg::Vec2 getPosition();

	/**
	 * @return starting azimuth
	 */
	float getAzimuth();

private:

	osg::Vec2 position_;

	float azimuth_;

};

}

#endif /* STARTPOSITION_H_ */
