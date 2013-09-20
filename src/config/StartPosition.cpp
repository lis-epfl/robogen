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

#include "config/StartPosition.h"
#include <osg/Vec2>
#include <iostream>

namespace robogen{

StartPosition::StartPosition(){
}

bool StartPosition::init(osg::Vec2 position, float azimuth){
	position_ = position;
	azimuth_ = azimuth;
	if(azimuth_<0. || azimuth>360.){
		std::cout << "Start position azimuth" << azimuth << " is not between 0"\
				" and 360 degrees" << std::endl;
		return false;
	}
	return true;
}

osg::Vec2 StartPosition::getPosition(){
	return position_;
}

float StartPosition::getAzimuth(){
	return azimuth_;
}

}
