/*
 * @(#) ObstaclesConfig.h   1.0   Mar 12, 2013
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
#ifndef ROBOGEN_OBSTACLES_CONFIG_H_
#define ROBOGEN_OBSTACLES_CONFIG_H_

#include <osg/Vec2>
#include <osg/Vec3>
#include <vector>

namespace robogen {

/**
 * Obstacles configuration parameters
 */
class ObstaclesConfig {

public:

	/**
	 * Initializes obstacles configuration
	 */
	ObstaclesConfig(const std::vector<osg::Vec2>& coordinates,
			const std::vector<osg::Vec3>& size,
			const std::vector<float> &density) :
			coordinates_(coordinates), size_(size), density_(density) {

	}

	/**
	 * Destructor
	 */
	virtual ~ObstaclesConfig() {

	}

	/**
	 * @return the coordinates of the obstacles
	 */
	const std::vector<osg::Vec2>& getCoordinates() const {
		return coordinates_;
	}

	/**
	 * @return the size of the obstacles
	 */
	const std::vector<osg::Vec3>& getSize() const {
		return size_;
	}

	/**
	 * @return the obstacle densities
	 */
	const std::vector<float>& getDensity() const{
		return density_;
	}

private:

	/**
	 * Obstacles coordinates
	 */
	std::vector<osg::Vec2> coordinates_;

	/**
	 * Obstacles size
	 */
	std::vector<osg::Vec3> size_;

	/**
	 * Obstacle density. If 0, obstacle is fixed
	 */
	std::vector<float> density_;
};

}

#endif /* ROBOGEN_OBSTACLES_CONFIG_H_ */
