/*
 * @(#) ConfigurationReader.h   1.0   Mar 13, 2013
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
#ifndef ROBOGEN_CONFIGURATION_READER_H_
#define ROBOGEN_CONFIGURATION_READER_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include "robogen.pb.h"

namespace robogen {

class ObstaclesConfig;
class RobogenConfig;
class StartPositionConfig;
class TerrainConfig;

/**
 * Read configuration files
 */
class ConfigurationReader {

public:

	/**
	 * Reads the configuration file for ROBOGEN.
	 * The file must contain the following parameterName=parameterValues pairs
	 *
	 * scenario=<"racing"|"chasing">
	 * timeStep=FLOAT
	 * nTimeSteps=FLOAT
	 * terrainType=<"flat"|"rugged">
	 * terrainLength=FLOAT
	 * terrainWidth=FLOAT
	 * terrainHeight=FLOAT [Mandatory if terrainType=="rugged", ignored otherwise]
	 * terrainHeightField=STRING [Mandatory if terrainType=="rugged", ignored otherwise]
	 * obstaclesConfigFile=STRING
	 * startPositionConfigFile=STRING
	 *
	 */
	static boost::shared_ptr<RobogenConfig> parseConfigurationFile(
			const std::string& fileName);

	static boost::shared_ptr<RobogenConfig> parseRobogenMessage(
				const robogenMessage::SimulatorConf& simulatorConf);

private:

	/**
	 * Reads the configuration file for obstacles.
	 * The file contains on each line the coordinates and sizes of the obstacles, separated by a tab (\t)
	 * X_POSITION	Y_POSITION	X_SIZE	Y_SIZE	Z_SIZE
	 */
	static boost::shared_ptr<ObstaclesConfig> parseObstaclesFile(
			const std::string& fileName);

	/**
	 * Reads the starting position file
	 * The file contains on each line the coordinates (on a 2D plane) of robot's starting positions, separated by a tab (\t)
	 * X_POSITION	Y_POSITION
	 */
	static boost::shared_ptr<StartPositionConfig> parseStartPositionFile(const std::string& fileName);

};

}

#endif /* ROBOGEN_CONFIGURATION_READER_H_ */
