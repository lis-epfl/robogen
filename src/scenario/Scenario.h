/*
 * @(#) Scenario.h   1.0   Mar 13, 2013
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
#ifndef ROBOGEN_SCENARIO_H_
#define ROBOGEN_SCENARIO_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include "Robogen.h"

namespace robogen {

class BoxObstacle;
class Environment;
class RobogenConfig;
class Robot;
class Terrain;

/**
 * A Scenario for the Robogen simulator.
 */
class Scenario {

public:

	/**
	 * A scenario is not valid until the
	 * init() method is called successfully.
	 *
	 * @param robogenConfig
	 */
	Scenario(boost::shared_ptr<RobogenConfig> robogenConfig);

	/**
	 * Destructor
	 */
	virtual ~Scenario();

	/**
	 * Initializes a scenario
	 *
	 * @param odeWorld
	 * @param odeSpace
	 * @param robot
	 */
	virtual bool init(dWorldID odeWorld, dSpaceID odeSpace,
			boost::shared_ptr<Robot> robot);

	/**
	 * Clears unused scenario (so that robot can be freed - joints need to be
	 * destroyed before the destruction of the world). Undoes what init() does
	 * and avoids memory leaks over multiple starting positions.
	 */
	void prune();

	/**
	 * @return the robot
	 */
	boost::shared_ptr<Robot> getRobot();

	/**
	 * @return the robogen configuration
	 */
	boost::shared_ptr<RobogenConfig> getRobogenConfig();

	/**
	 * @return the environment
	 */
	boost::shared_ptr<Environment> getEnvironment();

	/**
	 * @return the terrain
	 */
	boost::shared_ptr<Terrain> getTerrain();

	/**
	 * @return the obstacles
	 */
	std::vector<boost::shared_ptr<BoxObstacle> > getObstacles();

	/**
	 * Set the environment
	 */
	void setEnvironment(boost::shared_ptr<Environment> env);

	/**
	 * Sets the starting position id
	 */
	void setStartingPosition(int id);

	/**
	 * Setup the simulation scenario. Called before the physics simulation begins.
	 * @return true if the operation completed successfully, false otherwise
	 */
	virtual bool setupSimulation() = 0;

	/**
	 * Called after each simulation step
	 * @return true if the operation completed successfully, false otherwise
	 */
	virtual bool afterSimulationStep() = 0;

	/**
	 * Called at the end of the physics simulation
	 * @return true if the operation completed successfully, false otherwise
	 */
	virtual bool endSimulation() = 0;

	/**
	 * Compute the fitness
	 * @return fitness
	 */
	virtual double getFitness() = 0;

	/**
	 * @return true if another trial must be executed
	 */
	virtual bool remainingTrials() = 0;

	/**
	 * @return the current trial
	 */
	virtual int getCurTrial() const = 0;

protected:

	/**
	 * @return the current trial starting position
	 */
	boost::shared_ptr<StartPosition> getCurrentStartPosition();

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
	 * Robot
	 */
	boost::shared_ptr<Robot> robot_;

	/**
	 * Robogen config
	 */
	boost::shared_ptr<RobogenConfig> robogenConfig_;

	/**
	 * Terrain
	 */
	boost::shared_ptr<Terrain> terrain_;

	/**
	 * Obstacles
	 */
	std::vector<boost::shared_ptr<BoxObstacle> > obstacles_;

	/**
	 * The current id of starting position
	 */
	int startPositionId_;

	/**
	 * The environment
	 */
	boost::shared_ptr<Environment> environment_;

};

}

#endif /* ROBOGEN_SCENARIO_H_ */
