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
#include "config/LightSourcesConfig.h"
#include "robogen.pb.h"

namespace robogen {

/**
 * Parameters for experimental scenarios supported by the simulator
 */
class RobogenConfig {

public:

	/**
	 * Initializes a robogen config object from configuration parameters
	 */
	RobogenConfig(std::string scenario, std::string scenarioFile,
			unsigned int timeSteps,
			float timeStepLength, int actuationPeriod,
			boost::shared_ptr<TerrainConfig> terrain,
			boost::shared_ptr<ObstaclesConfig> obstacles,
			std::string obstacleFile,
			boost::shared_ptr<StartPositionConfig> startPositions,
			std::string startPosFile,
			boost::shared_ptr<LightSourcesConfig> lightSources,
			std::string lightSourceFile,
			float sensorNoiseLevel, float motorNoiseLevel,
			bool capAcceleration, float maxLinearAcceleration,
			float maxAngularAcceleration, int maxDirectionShiftsPerSecond,
			osg::Vec3 gravity, bool disallowObstacleCollisions) :
				scenario_(scenario), scenarioFile_(scenarioFile),
				timeSteps_(timeSteps),
				timeStepLength_(timeStepLength),
				actuationPeriod_(actuationPeriod),
				terrain_(terrain), obstacles_(obstacles),
				obstacleFile_(obstacleFile),
				startPositions_(startPositions),
				startPosFile_(startPosFile),
				lightSources_(lightSources),
				lightSourceFile_(lightSourceFile),
				sensorNoiseLevel_(sensorNoiseLevel),
				motorNoiseLevel_(motorNoiseLevel),
				capAcceleration_(capAcceleration),
				maxLinearAcceleration_(maxLinearAcceleration),
				maxAngularAcceleration_(maxAngularAcceleration),
				maxDirectionShiftsPerSecond_(maxDirectionShiftsPerSecond),
				gravity_(gravity),
				disallowObstacleCollisions_(disallowObstacleCollisions) {

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
	std::string getScenario() const {
		return scenario_;
	}

	/**
	 * @return the simulation scenario file (for scripted scenario)
	 */
	std::string getScenarioFile() const {
		return scenarioFile_;
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
	 * @return the light sources configuration
	 */
	boost::shared_ptr<LightSourcesConfig> getLightSourcesConfig() {
		return lightSources_;
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
	 * @return the light source configuration file
	 */
	std::string getLightSourceFile(){
		return lightSourceFile_;
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
	 * @return if acceleration is capped
	 */
	bool isCapAlleration() {
		return capAcceleration_;
	}

	/**
	 * @return max linear acceleration (if capped)
	 */
	float getMaxLinearAcceleration() {
		return maxLinearAcceleration_;
	}

	/**
	 * @return max angular acceleration (if capped)
	 */
	float getMaxAngularAcceleration() {
		return maxAngularAcceleration_;
	}

	/**
	 * @return max direction shifts per second for testing motor burnout
	 * 		if -1, then do not check burnout
	 */
	int getMaxDirectionShiftsPerSecond() {
		return maxDirectionShiftsPerSecond_;
	}

	/**
	 * @return gravity vector
	 */
	osg::Vec3 getGravity() {
		return gravity_;
	}

	/**
	 * return if should disallow obstacle collisions
	 */
	bool isDisallowObstacleCollisions() {
		return disallowObstacleCollisions_;
	}

	/**
	 * Convert configuration into configuration message.
	 */
	robogenMessage::SimulatorConf serialize() const{
		robogenMessage::SimulatorConf ret;

		ret.set_ntimesteps(timeSteps_);
		ret.set_scenario(scenario_);
		ret.set_timestep(timeStepLength_);
		ret.set_actuationperiod(actuationPeriod_);
		ret.set_sensornoiselevel(sensorNoiseLevel_);
		ret.set_motornoiselevel(motorNoiseLevel_);
		ret.set_capacceleration(capAcceleration_);
		ret.set_maxlinearacceleration(maxLinearAcceleration_);
		ret.set_maxangularacceleration(maxAngularAcceleration_);
		ret.set_maxdirectionshiftspersecond(maxDirectionShiftsPerSecond_);
		ret.set_gravityx(gravity_.x());
		ret.set_gravityy(gravity_.y());
		ret.set_gravityz(gravity_.z());
		ret.set_disallowobstaclecollisions(disallowObstacleCollisions_);

		terrain_->serialize(ret);

		obstacles_->serialize(ret);
		startPositions_->serialize(ret);
		lightSources_->serialize(ret);

		return ret;
	}

private:

	/**
	 * The simulation scenario
	 */
	std::string scenario_;

	/**
	 * The simulation scenario file (if using scripted scenario)
	 */
	std::string scenarioFile_;

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
	 * Light sources configuration
	 */
	boost::shared_ptr<LightSourcesConfig> lightSources_;

	/**
	 * Light sources configuration file location
	 */
	std::string lightSourceFile_;

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


	/**
	 *  Flag to enforce acceleration cap.  Useful for preventing unrealistic
	 *  behaviors / simulator exploits
	 */
	bool capAcceleration_;

	/**
	 *  Maximum allowed linear acceleration if acceleration is capped
	 */
	float maxLinearAcceleration_;

	/**
	 *  Maximum allowed angular acceleration if acceleration is capped
	 */
	float maxAngularAcceleration_;

	/**
	 *  Maximum allowed direction shifts per second (if specified)
	 */
	int maxDirectionShiftsPerSecond_;

	/**
	 * Gravity
	 */
	osg::Vec3 gravity_;

	/**
	 * flag to disallow obstacle collisions
	 */
	bool disallowObstacleCollisions_;
};

}

#endif /* ROBOGEN_ROBOGEN_CONFIG_H_ */
