/*
 * @(#) LightSourcesConfig.h   1.0   Jan 7, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
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
#ifndef ROBOGEN_LIGHTSOURCESCONFIG_H_
#define ROBOGEN_LIGHTSOURCESCONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {

/**
 * Obstacles configuration parameters
 */
class LightSourcesConfig {

public:

	/**
	 * Initializes light sources configuration
	 */

	// no light sources
	LightSourcesConfig() {}

	LightSourcesConfig(const std::vector<osg::Vec3>& coordinates,
			const std::vector<float> &intensities) :
			coordinates_(coordinates), intensities_(intensities) {

	}

	/**
	 * Destructor
	 */
	virtual ~LightSourcesConfig() {

	}

	/**
	 * @return the coordinates of the light sources
	 */
	const std::vector<osg::Vec3>& getCoordinates() const {
		return coordinates_;
	}

	/**
	 * @return the light source intensities
	 */
	const std::vector<float>& getIntensities() const{
		return intensities_;
	}

	/**
	 * Serialize obstacles into a SimulatorConf message
	 */
	void serialize(robogenMessage::SimulatorConf &message){
		for (unsigned int i=0; i<coordinates_.size(); ++i){
			robogenMessage::LightSource *curr = message.add_lightsources();
			curr->set_intensity(intensities_[i]);
			curr->set_x(coordinates_[i].x());
			curr->set_y(coordinates_[i].y());
			curr->set_z(coordinates_[i].z());
		}
	}

private:

	/**
	 * Light sources' coordinates
	 */
	std::vector<osg::Vec3> coordinates_;


	/**
	 * Light sources' intensities
	 */
	std::vector<float> intensities_;
};

}




#endif /* ROBOGEN_LIGHTSOURCESCONFIG_H_ */
