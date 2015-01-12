/*
 * @(#) RobogenConfig.h   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_ROBOGEN_CONFIG_H_
#define ROBOGEN_ROBOGEN_CONFIG_H_

#include <boost/shared_ptr.hpp>
#include "config/ObstaclesConfig.h"
#include "config/StartPositionConfig.h"
#include "config/TerrainConfig.h"
#include "robogen.pb.h"

namespace robogen {

/**
 * Parameters for experimental scenarios supported by the simulator
 */
class RobogenConfig {

public:

	enum SimulationScenario {
		CHASING, RACING
	};

	/**
	 * Initializes a robogen config object from configuration parameters
	 */
	RobogenConfig(SimulationScenario scenario, unsigned int timeSteps,
			float timeStepLength, int actuationPeriod,
			boost::shared_ptr<TerrainConfig> terrain,
			boost::shared_ptr<ObstaclesConfig> obstacles,
			std::string obstacleFile,
			boost::shared_ptr<StartPositionConfig> startPositions,
			std::string startPosFile, float lightSourceHeight,
			float sensorNoiseLevel, float motorNoiseLevel) :
				scenario_(scenario), timeSteps_(timeSteps),
				timeStepLength_(timeStepLength),
				actuationPeriod_(actuationPeriod),
				terrain_(terrain), obstacles_(obstacles),
				obstacleFile_(obstacleFile),
				startPositions_(startPositions),
				startPosFile_(startPosFile),
				lightSourceHeight_(lightSourceHeight),
				sensorNoiseLevel_(sensorNoiseLevel),
				motorNoiseLevel_(motorNoiseLevel) {

		simulationTime_ = timeSteps * timeStepLength;

	}

	/**
	 * Initializes a robogen config object from a message
	 */
	RobogenConfig(const robogenMessage::SimulatorConf &message);

	/**
	 * Destructor
	 */
	virtual ~RobogenConfig() {

	}

	/**
	 * @return the simulation scenario
	 */
	SimulationScenario getScenario() const {
		return scenario_;
	}

	/**
	 * @return the terrain configuration
	 */
	boost::shared_ptr<TerrainConfig> getTerrainConfig() {
		return terrain_;
	}

	/**
	 * @return the obstacles configuration
	 */
	boost::shared_ptr<ObstaclesConfig> getObstaclesConfig() {
		return obstacles_;
	}

	/**
	 * @return the number of timesteps
	 */
	unsigned int getTimeSteps() const {
		return timeSteps_;
	}

	/**
	 * @return the total simulation time
	 */
	float getSimulationTime() const {
		return simulationTime_;
	}

	/**
	 * @return the time step length
	 */
	float getTimeStepLength() const {
		return timeStepLength_;
	}

	/**
	 * @return the actuation period
	 */
	int getActuationPeriod() const {
		return actuationPeriod_;
	}

	/**
	 * @return the robot starting positions
	 */
	boost::shared_ptr<StartPositionConfig> getStartingPos() {
		return startPositions_;
	}

	/**
	 * @return the starting position configuration file
	 */
	std::string getStartPosFile(){
		return startPosFile_;
	}

	/**
	 * @return the obstacle configuration file
	 */
	std::string getObstacleFile(){
		return obstacleFile_;
	}

	/**
	 * @return the desired height of the light source
	 */
	float getLightSourceHeight(){
		return lightSourceHeight_;
	}

	/**
	 * @return sensor noise level
	 * Sensor noise is Gaussian with std dev of sensorNoiseLevel * actualValue
	 * i.e. value given to Neural Network is N(a, a * s)
	 * where a is actual value and s is sensorNoiseLevel
	 */
	float getSensorNoiseLevel() {
		return sensorNoiseLevel_;
	}

	/**
	 * @return motor noise level
	 * Motor noise is uniform in range +/- motorNoiseLevel * actualValue
	 */
	float getMotorNoiseLevel() {
		return motorNoiseLevel_;
	}

	/**
	 * Convert configuration into configuration message.
	 */
	robogenMessage::SimulatorConf serialize() const{
		robogenMessage::SimulatorConf ret;
		ret.set_lightsourceheight(lightSourceHeight_);
		ret.set_ntimesteps(timeSteps_);
		if (scenario_ == CHASING) {
			ret.set_scenario("chasing");
		} else if (scenario_ == RACING) {
			ret.set_scenario("racing");
		}
		ret.set_terrainlength(terrain_->getLength());
		ret.set_terrainwidth(terrain_->getWidth());
		ret.set_timestep(timeStepLength_);
		ret.set_actuationperiod(actuationPeriod_);
		ret.set_terrainfriction(terrain_->getFriction());
		ret.set_sensornoiselevel(sensorNoiseLevel_);
		ret.set_motornoiselevel(motorNoiseLevel_);
		obstacles_->serialize(ret);
		startPositions_->serialize(ret);
		return ret;
	}

private:

	/**
	 * The simulation scenario
	 */
	SimulationScenario scenario_;

	/**
	 * Total number of simulation timesteps
	 */
	unsigned int timeSteps_;

	/**
	 * Time step duration
	 */
	float timeStepLength_;

	/**
	 * Actuation period (in number of time steps)
	 */
	int actuationPeriod_;

	/**
	 * Terrain configuration
	 */
	boost::shared_ptr<TerrainConfig> terrain_;

	/**
	 * Obstacles configuration
	 */
	boost::shared_ptr<ObstaclesConfig> obstacles_;

	/**
	 * Obstacle configuration file location
	 */
	std::string obstacleFile_;

	/**
	 * List of robot starting positions
	 */
	boost::shared_ptr<StartPositionConfig> startPositions_;

	/**
	 * Starting positions configuration file location
	 */
	std::string startPosFile_;

	/**
	 * Height of light source in chasing scenario
	 */
	float lightSourceHeight_;

	/**
	 * Simulation time
	 */
	float simulationTime_;

	/**
	 * Sensor noise level (see getter for details)
	 */
	float sensorNoiseLevel_;

	/**
	 * Motor noise level (see getter for details)
	 */
	float motorNoiseLevel_;

};

}

#endif /* ROBOGEN_ROBOGEN_CONFIG_H_ */
