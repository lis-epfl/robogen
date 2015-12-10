/*
 * @(#) environment.h   1.0   Mar 6, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_ENVIRONMENT_H_
#define ROBOGEN_ENVIRONMENT_H_

#include "model/objects/LightSource.h"
#include "model/objects/Obstacle.h"
#include "Terrain.h"
#include "config/RobogenConfig.h"

#define DEFAULT_AMBIENT_LIGHT (0.04)

namespace robogen {


/*
 * Environment contains all info about the simulation except for the robot
 */

class Environment {

public:

	Environment(dWorldID odeWorld, dSpaceID odeSpace,
			boost::shared_ptr<RobogenConfig> robogenConfig);

	bool init();

	virtual ~Environment();

	void setTimeElapsed(float timeElapsed) {
		timeElapsed_ = timeElapsed;
	}

	float getTimeElapsed() {
		return timeElapsed_;
	}

	void setLightSources(
			const std::vector<boost::shared_ptr<LightSource> >& lightSources) {
		lightSources_ = lightSources;
	}

	const std::vector<boost::shared_ptr<LightSource> >& getLightSources() {
		return lightSources_;
	}

	void setAmbientLight(float ambientLight) {
		ambientLight_ = ambientLight;
	}

	float getAmbientLight() {
		return ambientLight_;
	}

	void addObstacle(boost::shared_ptr<Obstacle> obstacle) {
		obstacles_.push_back(obstacle);
	}

	boost::shared_ptr<Terrain> getTerrain() {
		return terrain_;
	}

	std::vector<boost::shared_ptr<Obstacle> > getObstacles() {
		return obstacles_;
	}

private:
	/**
	 * ODE physics world
	 */
	dWorldID odeWorld_;

	/**
	 * ODE collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * Robogen config
	 */
	boost::shared_ptr<RobogenConfig> robogenConfig_;

	/**
	 * Time elapsed since the last "refresh" of the environment
	 */
	float timeElapsed_;

	/**
	 * Light sources in the environment
	 */
	std::vector<boost::shared_ptr<LightSource> > lightSources_;

	/**
	 * Ambient light
	 */
	float ambientLight_;

	/**
	 * Terrain
	 */
	boost::shared_ptr<Terrain> terrain_;

	/**
	 * Obstacle in the environment
	 */

	std::vector<boost::shared_ptr<Obstacle> > obstacles_;

};

}

#endif /* ROBOGEN_ENVIRONMENT_H_ */
