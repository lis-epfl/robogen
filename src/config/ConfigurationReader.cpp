/*
 * @(#) ConfigurationReader.h   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include "config/ConfigurationReader.h"
#include "config/ObstaclesConfig.h"
#include "config/RobogenConfig.h"
#include "config/StartPosition.h"
#include "config/StartPositionConfig.h"
#include "config/TerrainConfig.h"

#define DEFAULT_LIGHT_SOURCE_HEIGHT (0.1)
#define DEFAULT_OBSTACLE_DENSITY (0.)

namespace robogen {

boost::shared_ptr<RobogenConfig> ConfigurationReader::parseConfigurationFile(
		const std::string& fileName) {

	boost::program_options::options_description desc(
			"Allowed options for Simulation Config File");
	desc.add_options()("terrainType",
			boost::program_options::value<std::string>(),
			"Terrain type: flat or rough")("terrainHeightField",
			boost::program_options::value<std::string>(),
			"Height Field for terrain generation")("terrainWidth",
			boost::program_options::value<float>(), "Terrain width")(
			"terrainHeight", boost::program_options::value<float>(),
			"Terrain height")("terrainLength",
			boost::program_options::value<float>(), "Terrain length")(
			"terrainFriction",boost::program_options::value<float>(),
			"Terrain Friction Coefficient")(
			"obstaclesConfigFile", boost::program_options::value<std::string>(),
			"Obstacles configuration file")("scenario",
			boost::program_options::value<std::string>(), "Experiment scenario")
			("lightSourceHeight", boost::program_options::value<float>(),
			"Height of light source")("timeStep",
			boost::program_options::value<float>(), "Time step duration (s)")(
			"nTimeSteps", boost::program_options::value<unsigned int>(),
			"Number of timesteps")
			("actuationFrequency",boost::program_options::value<int>(),
			"Actuation Frequency (Hz)")
			("sensorNoiseLevel",boost::program_options::value<float>(),
			"Sensor Noise Level:\n "
			"Sensor noise is Gaussian with std dev of "
			"sensorNoiseLevel * actualValue.\n"
			"i.e. value given to Neural Network is "
			"N(a, a * s)\n"
			"where a is actual value and s is sensorNoiseLevel")
			("motorNoiseLevel",boost::program_options::value<float>(),
			"Motor noise level:\n"
			"Motor noise is uniform in range +/-"
			"(motorNoiseLevel * actualValue)"
			)
			("startPositionConfigFile",
			boost::program_options::value<std::string>(),
			"Start Positions Configuration File");

	if (fileName == "help") {
		desc.print(std::cout);
		return boost::shared_ptr<RobogenConfig>();
	}

	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_config_file<char>(fileName.c_str(),
					desc, true), vm);
	boost::program_options::notify(vm);

	// Read terrain configuration
	float terrainLength;
	float terrainWidth;
	float terrainFriction;
	std::string terrainType;

	if (!vm.count("terrainType")) {
		std::cout << "Undefined 'terrainType' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	if (!vm.count("terrainLength")) {
		std::cout << "Undefined 'terrainLength' parameter in '" << fileName
				<< "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	if (!vm.count("terrainWidth")) {
		std::cout << "Undefined 'terrainWidth' parameter in '" << fileName
				<< "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	if (!vm.count("terrainFriction")) {
		std::cout << "Undefined 'terrainFriction' parameter in '" << fileName
				<< "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	terrainType = vm["terrainType"].as<std::string>();
	terrainLength = vm["terrainLength"].as<float>();
	terrainWidth = vm["terrainWidth"].as<float>();
	terrainFriction = vm["terrainFriction"].as<float>();

	boost::shared_ptr<TerrainConfig> terrain;
	if (terrainType.compare("flat") == 0) {

		terrain.reset(new TerrainConfig(terrainLength, terrainWidth,
				terrainFriction));

	} else if (terrainType.compare("rugged") == 0) {

		std::string terrainHeightField;
		float terrainHeight;

		if (!vm.count("terrainHeightField")) {
			std::cout << "Undefined 'terrainHeightField' parameter in '"
					<< fileName << "'" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}

		if (!vm.count("terrainHeight")) {
			std::cout << "Undefined 'terrainHeight' parameter in '" << fileName
					<< "'" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}

		terrainHeightField = vm["terrainHeightField"].as<std::string>();
		terrainHeight = vm["terrainHeight"].as<float>();

		terrain.reset(
				new TerrainConfig(terrainHeightField, terrainLength,
						terrainWidth, terrainHeight, terrainFriction));

	} else {
		std::cout << "Unknown value of 'terrainType' parameter in '" << fileName
				<< "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	// Read obstacles configuration
	if (!vm.count("obstaclesConfigFile")) {
		std::cout << "Undefined 'obstaclesConfigFile' parameter in '"
				<< fileName << "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}


	const boost::filesystem::path filePath(fileName);

	std::string obstaclesConfigFile =
			vm["obstaclesConfigFile"].as<std::string>();

	const boost::filesystem::path obstaclesConfigFilePath(obstaclesConfigFile);
	if (!obstaclesConfigFilePath.is_absolute()) {
		const boost::filesystem::path absolutePath =
				boost::filesystem::absolute(obstaclesConfigFilePath,
						filePath.parent_path());
		obstaclesConfigFile = absolutePath.string();
	}




	boost::shared_ptr<ObstaclesConfig> obstacles = parseObstaclesFile(
			obstaclesConfigFile);
	if (obstacles == NULL) {
		return boost::shared_ptr<RobogenConfig>();
	}

	// Read obstacles configuration
	if (!vm.count("startPositionConfigFile")) {
		std::cout << "Undefined 'startPositionConfigFile' parameter in '"
				<< fileName << "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	std::string startPositionFile =
			vm["startPositionConfigFile"].as<std::string>();

	const boost::filesystem::path startPositionFilePath(startPositionFile);
	if (!startPositionFilePath.is_absolute()) {
		const boost::filesystem::path absolutePath =
				boost::filesystem::absolute(startPositionFilePath,
						filePath.parent_path());
		startPositionFile = absolutePath.string();
	}


	boost::shared_ptr<StartPositionConfig> startPositions =
			parseStartPositionFile(startPositionFile);
	if (startPositions == NULL) {
		return boost::shared_ptr<RobogenConfig>();
	}

	// Read generic parameters
	if (!vm.count("scenario")) {
		std::cout << "Undefined 'scenario' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	std::string scenario = vm["scenario"].as<std::string>();
	RobogenConfig::SimulationScenario simulationScenario;

	if (scenario.compare("racing") == 0) {
		simulationScenario = RobogenConfig::RACING;
	} else if (scenario.compare("chasing") == 0) {
		simulationScenario = RobogenConfig::CHASING;
	} else {
		std::cout << "Undefined 'scenario' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	float lightSourceHeight = DEFAULT_LIGHT_SOURCE_HEIGHT;
	if (vm.count("lightSourceHeight")) {
		lightSourceHeight = vm["lightSourceHeight"].as<float>();
	}

	if (!vm.count("timeStep")) {
		std::cout << "Undefined 'timeStep' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	float timeStep = vm["timeStep"].as<float>();

	if (!vm.count("nTimeSteps")) {
		std::cout << "Undefined 'nTimesteps' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	unsigned int nTimesteps = vm["nTimeSteps"].as<unsigned int>();

	int actuationPeriod;
	if (!vm.count("actuationFrequency")) {
		actuationPeriod = 1;
		std::cout << "Undefined 'actuationFrequency' parameter in '"
				<< fileName << "'" << ", will actuate every timeStep."
				<< std::endl;
	} else {
		int actuationFrequencyTmp = (int) (
				(1.0/((float)vm["actuationFrequency"].as<int>())) * 100000);
		int timeStepTmp = (int) (timeStep * 100000);
		if ((actuationFrequencyTmp % timeStepTmp) != 0) {
			std::cout << "Inverse of 'actuationFrequency' must be a multiple "
					<< "of 'timeStep'" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}
		actuationPeriod = actuationFrequencyTmp / timeStepTmp;
	}

	// noise

	float sensorNoiseLevel = 0;
	float motorNoiseLevel = 0;

	if(vm.count("sensorNoiseLevel")) {
		sensorNoiseLevel = vm["sensorNoiseLevel"].as<float>();
	}

	if(vm.count("motorNoiseLevel")) {
		motorNoiseLevel = vm["motorNoiseLevel"].as<float>();
	}


	vm["terrainFriction"].as<float>();

	return boost::shared_ptr<RobogenConfig>(
			new RobogenConfig(simulationScenario, nTimesteps,
					timeStep, actuationPeriod, terrain,
					obstacles, obstaclesConfigFile, startPositions,
					startPositionFile, lightSourceHeight, sensorNoiseLevel,
					motorNoiseLevel));

}

const std::string getMatchNFloatPattern(int n) {
	std::stringstream paternSS;
	paternSS << "^";
	for ( unsigned i=0; i<n; i++) {
		paternSS << "(-?\\d*[\\d\\.]\\d*)";
		if ( i < (n-1) )
			paternSS << "\\s+";
	}
	paternSS << "$";
	return paternSS.str();
}

boost::shared_ptr<ObstaclesConfig> ConfigurationReader::parseObstaclesFile(
		const std::string& fileName) {

	std::ifstream obstaclesFile(fileName.c_str());
	if (!obstaclesFile.is_open()) {
		std::cout << "Cannot find obstacles file: '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<ObstaclesConfig>();
	}

	// old = on ground, no rotation:
	// x y xLength yLength zLength density
	static const boost::regex oldObstacleRegex(getMatchNFloatPattern(6));
	// same as above but can be above ground:
	// x y z xLength yLength zLength density
	static const boost::regex noRotationObstacleRegex(
			getMatchNFloatPattern(7));
	// full (w/ axis+angle rotation)
	// x y z xLength yLength zLength density xRot yRot zRot rotAngle
	static const boost::regex fullObstacleRegex(
				getMatchNFloatPattern(11));

	std::vector<osg::Vec3> coordinates;
	std::vector<osg::Vec3> sizes;
	std::vector<float> densities;
	std::vector<osg::Vec3> rotationAxes;
	std::vector<float> rotationAngles;

	std::string line;
	int lineNum = 0;
	while (std::getline(obstaclesFile, line)) {
		lineNum++;
		boost::cmatch match;
		float x, y, z, xSize, ySize, zSize, density, xRotation, yRotation,
				zRotation, rotationAngle;
		if(boost::regex_match(line.c_str(), match,
						fullObstacleRegex)){
			x = std::atof(match[1].str().c_str());
			y = std::atof(match[2].str().c_str());
			z = std::atof(match[3].str().c_str());
			xSize = std::atof(match[4].str().c_str());
			ySize = std::atof(match[5].str().c_str());
			zSize = std::atof(match[6].str().c_str());
			density = std::atof(match[7].str().c_str());
			xRotation = std::atof(match[8].str().c_str());
			yRotation = std::atof(match[9].str().c_str());
			zRotation = std::atof(match[10].str().c_str());
			rotationAngle = std::atof(match[11].str().c_str());
		} else {
			xRotation = yRotation = zRotation = rotationAngle = 0.0;
			if (boost::regex_match(line.c_str(), match, oldObstacleRegex)){
				x = std::atof(match[1].str().c_str());
				y = std::atof(match[2].str().c_str());
				xSize = std::atof(match[3].str().c_str());
				ySize = std::atof(match[4].str().c_str());
				zSize = std::atof(match[5].str().c_str());
				density = std::atof(match[6].str().c_str());

				z = zSize/2;
			} else if(boost::regex_match(line.c_str(), match,
					noRotationObstacleRegex)){
				x = std::atof(match[1].str().c_str());
				y = std::atof(match[2].str().c_str());
				z = std::atof(match[3].str().c_str());
				xSize = std::atof(match[4].str().c_str());
				ySize = std::atof(match[5].str().c_str());
				zSize = std::atof(match[6].str().c_str());
				density = std::atof(match[7].str().c_str());
			} else {
				std::cout << "Error parsing line " << lineNum <<
						" of obstacles file: '" << fileName << "'"
						<< std::endl;
				return boost::shared_ptr<ObstaclesConfig>();
			}
		}
		coordinates.push_back(osg::Vec3(x, y, z));
		sizes.push_back(osg::Vec3(xSize, ySize, zSize));
		densities.push_back(density);
		rotationAxes.push_back(osg::Vec3(xRotation, yRotation, zRotation));
		rotationAngles.push_back(rotationAngle);
	}

	return boost::shared_ptr<ObstaclesConfig>(
			new ObstaclesConfig(coordinates, sizes, densities,
					rotationAxes, rotationAngles));
}

boost::shared_ptr<StartPositionConfig> ConfigurationReader::parseStartPositionFile(
		const std::string& fileName) {

	std::ifstream startPosFile(fileName.c_str());
	if (!startPosFile.is_open()) {
		std::cout << "Cannot find start position file: '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<StartPositionConfig>();
	}
	std::vector<boost::shared_ptr<StartPosition> > startPositions;
	float x, y, azimuth;
	while (startPosFile >> x) {
		if (!(startPosFile >> y)) {
			std::cout << "Malformed start position file: '" << fileName << "'"
					<< std::endl;
			return boost::shared_ptr<StartPositionConfig>();
		}
		if (!(startPosFile >> azimuth)) {
			std::cout << "Malformed start position file: '" << fileName << "'"
					<< std::endl;
			return boost::shared_ptr<StartPositionConfig>();
		}
		startPositions.push_back(
				boost::shared_ptr<StartPosition>(new StartPosition()));
		if (!startPositions.back()->init(osg::Vec2(x, y), azimuth)) {
			std::cout << "Problem initializing start position!" << std::endl;
			return boost::shared_ptr<StartPositionConfig>();
		}
	}

	return boost::shared_ptr<StartPositionConfig>(
			new StartPositionConfig(startPositions));

}

boost::shared_ptr<RobogenConfig> ConfigurationReader::parseRobogenMessage(
		const robogenMessage::SimulatorConf& simulatorConf) {

	// Decode obstacles
	std::vector<osg::Vec3> obstaclesCoord;
	std::vector<osg::Vec3> obstaclesSize;
	std::vector<float> obstaclesDensity;
	std::vector<osg::Vec3> obstaclesRotationAxis;
	std::vector<float> obstaclesRotationAngle;
	for (int i = 0; i < simulatorConf.obstacles_size(); ++i) {

		const robogenMessage::Obstacle& o = simulatorConf.obstacles(i);
		obstaclesCoord.push_back(osg::Vec3(o.x(), o.y(), o.z()));
		obstaclesSize.push_back(osg::Vec3(o.xsize(), o.ysize(), o.zsize()));
		obstaclesDensity.push_back(o.density());
		obstaclesRotationAxis.push_back(osg::Vec3(o.xrotation(),
				o.yrotation(), o.zrotation()));
		obstaclesRotationAngle.push_back(o.rotationangle());

	}
	boost::shared_ptr<ObstaclesConfig> obstacles(
			new ObstaclesConfig(obstaclesCoord, obstaclesSize,
					obstaclesDensity, obstaclesRotationAxis,
					obstaclesRotationAngle));

	// Decode start positions
	std::vector<boost::shared_ptr<StartPosition> > startPositions;
	for (int i = 0; i < simulatorConf.startpositions_size(); ++i) {
		const robogenMessage::StartPosition& s = simulatorConf.startpositions(
				i);
		boost::shared_ptr<StartPosition> newStartPos(new StartPosition());
		newStartPos->init(osg::Vec2(s.x(), s.y()), s.azimuth());
		startPositions.push_back(newStartPos);
	}

	// Decode terrain configuration
	boost::shared_ptr<TerrainConfig> terrain(
			new TerrainConfig(simulatorConf.terrainlength(),
					simulatorConf.terrainwidth(),
					simulatorConf.terrainfriction()));

	// Decode simulator configuration
	RobogenConfig::SimulationScenario simulationScenario;
	std::string scenario = simulatorConf.scenario();

	if (scenario.compare("racing") == 0) {
		simulationScenario = RobogenConfig::RACING;
	} else if (scenario.compare("chasing") == 0) {
		simulationScenario = RobogenConfig::CHASING;
	} else {
		std::cout << "Undefined 'scenario' parameter" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	unsigned int timeSteps = simulatorConf.ntimesteps();
	float timeStepLength = simulatorConf.timestep();
	float lightSourceHeight = simulatorConf.lightsourceheight();
	int actuationPeriod = simulatorConf.actuationperiod();

	float sensorNoiseLevel = simulatorConf.sensornoiselevel();
	float motorNoiseLevel = simulatorConf.motornoiselevel();

	return boost::shared_ptr<RobogenConfig>(
			new RobogenConfig(simulationScenario, timeSteps, timeStepLength,
					actuationPeriod, terrain, obstacles, "",
					boost::shared_ptr<StartPositionConfig>(
							new StartPositionConfig(startPositions)), "",
					lightSourceHeight, sensorNoiseLevel, motorNoiseLevel

					));

}

}
