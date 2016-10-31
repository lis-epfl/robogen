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
#include <boost/math/special_functions/round.hpp>
#include <boost/algorithm/string.hpp>

#include "config/ConfigurationReader.h"
#include "config/ObstaclesConfig.h"
#include "config/RobogenConfig.h"
#include "config/StartPosition.h"
#include "config/StartPositionConfig.h"
#include "config/TerrainConfig.h"
#include "config/LightSourcesConfig.h"

#include "utils/RobogenUtils.h"
#include "utils/ParsingUtils.h"

#define DEFAULT_LIGHT_SOURCE_HEIGHT (0.1)
#define DEFAULT_OBSTACLE_DENSITY (0.)
#define DEFAULT_MAX_LINEAR_ACCELERATION (15.0)
#define DEFAULT_MAX_ANGULAR_ACCELERATION (25.0)

namespace robogen {

void makeAbsolute(std::string &fileName,
		const boost::filesystem::path &filePath) {
	const boost::filesystem::path thisFilePath(fileName);
	if (!thisFilePath.is_absolute()) {
		const boost::filesystem::path absolutePath =
				boost::filesystem::absolute(thisFilePath,
						filePath.parent_path());
		fileName = absolutePath.string();
	}
}


boost::shared_ptr<RobogenConfig> ConfigurationReader::parseConfigurationFile(
		const std::string& fileName) {

	boost::program_options::options_description desc(
			"Allowed options for Simulation Config File");
	desc.add_options()
			("scenario",
					boost::program_options::value<std::string>(),
					"Experiment scenario: (racing, chasing, "
					"or a provided js file)")
			("timeStep", boost::program_options::value<float>(),
					"Time step duration (s)")
			("nTimeSteps", boost::program_options::value<unsigned int>(),
					"Number of timesteps (Either this or simulationTime are required)")
			("simulationTime", boost::program_options::value<float>(),
					"Length of simulation (s)  (Either this or nTimeSteps "\
					"are required)")
			("terrainType",
					boost::program_options::value<std::string>(),
					"Terrain type: flat or rugged")
			("terrainHeightField",
					boost::program_options::value<std::string>(),
					"Height Field for terrain generation")
			("terrainWidth",
					boost::program_options::value<float>(), "Terrain width")
			("terrainHeight", boost::program_options::value<float>(),
					"Terrain height")
			("terrainLength", boost::program_options::value<float>(),
					"Terrain length")
			("terrainFriction",boost::program_options::value<float>(),
					"Terrain Friction Coefficient")
			("startPositionConfigFile",
					boost::program_options::value<std::string>(),
					"Start Positions Configuration File")
			("obstaclesConfigFile", boost::program_options::value<std::string>(),
					"Obstacles configuration file")
			("lightSourcesConfigFile", boost::program_options::value<std::string>(),
					"Light sources configuration file")
			("actuationFrequency",boost::program_options::value<int>(),
					"Actuation Frequency (Hz)")
			("sensorNoiseLevel",boost::program_options::value<float>(),
					"Sensor Noise Level:\n "\
					"Sensor noise is Gaussian with std dev of "\
					"sensorNoiseLevel * actualValue.\n"\
					"i.e. value given to Neural Network is "\
					"N(a, a * s)\n"\
					"where a is actual value and s is sensorNoiseLevel")
			("motorNoiseLevel",boost::program_options::value<float>(),
					"Motor noise level:\n"\
					"Motor noise is uniform in range +/-"\
					"(motorNoiseLevel * actualValue)")
			("capAcceleration",boost::program_options::value<bool>(),
					"Flag to enforce acceleration cap."\
					"Useful for preventing unrealistic  behaviors "\
					"/ simulator exploits")
			("maxLinearAcceleration",boost::program_options::value<float>(),
					"Maximum linear acceleration (if capAcceleration."\
					" is true")
			("maxAngularAcceleration",boost::program_options::value<float>(),
					"Maximum angular acceleration (if capAcceleration."\
					" is true")
			("maxDirectionShiftsPerSecond",boost::program_options::value<int>(),
					"Maximum number of direction shifts per second"\
					" for testing motor burnout.  If not set, then there is no"\
					" cap")
			("gravity",
					boost::program_options::value<std::string>(),
					"Gravity: either a single z-value for g=(0,0,z)"\
					" or x,y,z (comma separated) for full g vector."\
					" Specified in m/(s^2)"\
					" Defaults to (0,0,-9.81)")
			("disallowObstacleCollisions",
					boost::program_options::value<bool>(),
					"Flag to enforce no obstacle collisions.  If true then "\
					"any obstacle collision will be considered a constraint"\
					" violation. (default false).")
			("obstacleOverlapPolicy",
					boost::program_options::value<std::string>(),
					"Defines the policy for handling obstacles "
					" enclosed in the robot's initial"\
					" axis aligned bounding box (AABB).  Options are\n"\
					"\t'removeObstacles' -- obstacles will be removed,"\
					" and the simulation will proceed (default).\n"\
					"\t'constraintViolation' -- the simulation will be"\
					" terminated with a constrain violation.\n"\
					"\t'elevateRobot' -- the robot will be elevated to be"\
					" above all obstacles before the simulation begins.\n")
			;

	if (fileName == "help") {
		desc.print(std::cout);
		return boost::shared_ptr<RobogenConfig>();
	}

	boost::program_options::variables_map vm;

	try {
		boost::program_options::store(
			boost::program_options::parse_config_file<char>(fileName.c_str(),
					desc, false), vm);
		boost::program_options::notify(vm);
	} catch (std::exception &e) {
		std::cerr << "Error while processing simulator configuration: "
					<< e.what() << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	const boost::filesystem::path filePath(fileName);

	// Read terrain configuration
	float terrainLength;
	float terrainWidth;
	float terrainFriction;
	std::string terrainType;

	if (!vm.count("terrainType")) {
		std::cerr << "Undefined 'terrainType' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	terrainType = vm["terrainType"].as<std::string>();

	if (!vm.count("terrainFriction")) {
		std::cerr << "Undefined 'terrainFriction' parameter in '" << fileName
				<< "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	terrainFriction = vm["terrainFriction"].as<float>();

	boost::shared_ptr<TerrainConfig> terrain;
	if (terrainType.compare("empty") == 0) {
		terrain.reset(new TerrainConfig(terrainFriction));
	} else {
		// if have empty terrain don't need these values, and leave terrain NULL
		if (!vm.count("terrainLength")) {
			std::cerr << "Undefined 'terrainLength' parameter in '" << fileName
					<< "'" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}
		terrainLength = vm["terrainLength"].as<float>();

		if (!vm.count("terrainWidth")) {
			std::cerr << "Undefined 'terrainWidth' parameter in '" << fileName
					<< "'" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}
		terrainWidth = vm["terrainWidth"].as<float>();

		if (terrainType.compare("flat") == 0) {

			terrain.reset(new TerrainConfig(terrainLength, terrainWidth,
					terrainFriction));

		} else if (terrainType.compare("rugged") == 0) {

			std::string terrainHeightField;
			float terrainHeight;

			if (!vm.count("terrainHeightField")) {
				std::cerr << "Undefined 'terrainHeightField' parameter in '"
						<< fileName << "'" << std::endl;
				return boost::shared_ptr<RobogenConfig>();
			}

			if (!vm.count("terrainHeight")) {
				std::cerr << "Undefined 'terrainHeight' parameter in '" << fileName
						<< "'" << std::endl;
				return boost::shared_ptr<RobogenConfig>();
			}

			terrainHeightField = vm["terrainHeightField"].as<std::string>();
			makeAbsolute(terrainHeightField, filePath);



			terrainHeight = vm["terrainHeight"].as<float>();

			terrain.reset(
					new TerrainConfig(terrainHeightField, terrainLength,
							terrainWidth, terrainHeight, terrainFriction));

		} else {
			std::cerr << "Unknown value of 'terrainType' parameter in '" << fileName
					<< "'" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}
	}

	// Read obstacles configuration
	boost::shared_ptr<ObstaclesConfig> obstacles;
	std::string obstaclesConfigFile = "";
	if (!vm.count("obstaclesConfigFile")) {
		obstacles.reset(new ObstaclesConfig());
	} else {
		obstaclesConfigFile = vm["obstaclesConfigFile"].as<std::string>();

		makeAbsolute(obstaclesConfigFile, filePath);

		obstacles = parseObstaclesFile(
				obstaclesConfigFile);
		if (obstacles == NULL) {
			return boost::shared_ptr<RobogenConfig>();
		}
	}

	boost::shared_ptr<StartPositionConfig> startPositions;
	std::string startPositionFile = "";
	// Read start pos configuration
	if (!vm.count("startPositionConfigFile")) {
		std::cout << "No startPositionConfigFile provided so will use a single"
				<< " evaluation with the robot starting at the origin, and "
				<< "having 0 azimuth" << std::endl;
		std::vector<boost::shared_ptr<StartPosition> > startPositionVector;

		boost::shared_ptr<StartPosition> startPosition(new StartPosition());
		if (!startPosition->init(osg::Vec2(0,0), 0)) {
			std::cerr << "Problem initializing start position!" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}

		startPositionVector.push_back(startPosition);
		startPositions.reset(new StartPositionConfig(startPositionVector));

	} else {

		startPositionFile = vm["startPositionConfigFile"].as<std::string>();

		makeAbsolute(startPositionFile, filePath);

		startPositions = parseStartPositionFile(startPositionFile);
		if (startPositions == NULL) {
			return boost::shared_ptr<RobogenConfig>();
		}
	}

	boost::shared_ptr<LightSourcesConfig> lightSources;
	std::string lightSourcesFile = "";
	// Read light sources configuration
	if (!vm.count("lightSourcesConfigFile")) {
		lightSources.reset(new LightSourcesConfig());
	} else {
		lightSourcesFile =
			vm["lightSourcesConfigFile"].as<std::string>();

		makeAbsolute(lightSourcesFile, filePath);

		lightSources = parseLightSourcesFile(lightSourcesFile);
		if (lightSources == NULL) {
			return boost::shared_ptr<RobogenConfig>();
		}
	}




	// Read scenario
	if (!vm.count("scenario")) {
		std::cerr << "Undefined 'scenario' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	std::string scenario = vm["scenario"].as<std::string>();
	std::string scenarioFile = "";
	if (scenario.compare("racing") != 0 && scenario.compare("chasing") != 0) {
		const boost::filesystem::path scenarioFilePath(scenario);
		if (!scenarioFilePath.is_absolute()) {
			const boost::filesystem::path absolutePath =
					boost::filesystem::absolute(scenarioFilePath,
							filePath.parent_path());
			scenarioFile = absolutePath.string();
		}
		if(boost::filesystem::path(scenarioFile).extension().string().compare(".js")
				== 0) {

			//read entire js file into string buffer

			std::ifstream file(scenarioFile.c_str());
			if (!file.is_open()) {
				std::cout << "Cannot find scenario js: '" << scenarioFile << "'"
						<< std::endl;
				return boost::shared_ptr<RobogenConfig>();
			}


			std::stringstream buffer;
			buffer << file.rdbuf();
			scenario = buffer.str();
		} else {
			std::cerr << "Invalid 'scenario' parameter" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}
	}

	if (!vm.count("timeStep")) {
		std::cerr << "Undefined 'timeStep' parameter in '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	float timeStep = vm["timeStep"].as<float>();
	int timeStepTmp = boost::math::iround (timeStep * 100000);

	if (!vm.count("nTimeSteps") && !vm.count("simulationTime")) {
		std::cerr << "Either 'nTimesteps' or 'simulationTime' is required "
				<< "in '" << fileName << "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	if (vm.count("nTimeSteps") && vm.count("simulationTime")) {
		std::cerr << "Only one of 'nTimesteps' or 'simulationTime' should "
				<< "be specified in '" << fileName << "'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}
	unsigned int nTimeSteps;
	if (vm.count("nTimeSteps")){
		nTimeSteps = vm["nTimeSteps"].as<unsigned int>();
	} else {
		int simulationTimeTmp = boost::math::iround (
						vm["simulationTime"].as<float>() * 100000);
		if ((simulationTimeTmp % timeStepTmp) != 0) {
			std::cerr << "'simulationTime' must be a multiple "
					<< "of 'timeStep'" << std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}
		nTimeSteps = simulationTimeTmp / timeStepTmp;
	}

	int actuationPeriod;
	if (!vm.count("actuationFrequency")) {
		actuationPeriod = 1;
		std::cout << "Undefined 'actuationFrequency' parameter in '"
				<< fileName << "'" << ", will actuate every timeStep."
				<< std::endl;
	} else {
		int actuationFrequencyTmp = boost::math::iround (
				(1.0/((float)vm["actuationFrequency"].as<int>())) * 100000);
		if ((actuationFrequencyTmp % timeStepTmp) != 0) {
			std::cerr << "Inverse of 'actuationFrequency' must be a multiple "
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

	bool capAcceleration = false;
	float maxLinearAcceleration = DEFAULT_MAX_LINEAR_ACCELERATION;
	float maxAngularAcceleration = DEFAULT_MAX_ANGULAR_ACCELERATION;

	if(vm.count("capAcceleration")) {
		capAcceleration = vm["capAcceleration"].as<bool>();
	}

	if(vm.count("maxLinearAcceleration")) {
		maxLinearAcceleration = vm["maxLinearAcceleration"].as<float>();
	}

	if(vm.count("maxAngularAcceleration")) {
		maxAngularAcceleration = vm["maxAngularAcceleration"].as<float>();
	}

	int maxDirectionShiftsPerSecond = -1;
	if(vm.count("maxDirectionShiftsPerSecond")) {
		maxDirectionShiftsPerSecond = vm["maxDirectionShiftsPerSecond"
		                                 ].as<int>();
	}

	osg::Vec3 gravity(0,0,-9.81);
	if(vm.count("gravity")) {

		std::string gravityString = vm["gravity"].as<std::string>();
		std::vector<std::string> gravityOpts;
		boost::split(gravityOpts, gravityString, boost::is_any_of(","));
		if (gravityOpts.size() == 1) {
			gravity[2] = parse_float(gravityOpts[0]);
		} else if (gravityOpts.size() == 3) {
			for(unsigned int i=0; i<3; ++i) {
				gravity[i] = parse_float(gravityOpts[i]);
			}
		} else {
			std::cerr << "'gravity' must either be a single value for " <<
					"g=(0,0,z) or x,y,z (comma separated) for full g vector" <<
					std::endl;
			return boost::shared_ptr<RobogenConfig>();
		}
	}

	bool disallowObstacleCollisions = false;
	if(vm.count("disallowObstacleCollisions")) {
		disallowObstacleCollisions = vm["disallowObstacleCollisions"
		                                ].as<bool>();
	}

	unsigned int obstacleOverlapPolicy;

	if((!vm.count("obstacleOverlapPolicy")) ||
			(vm["obstacleOverlapPolicy"].as<std::string>() ==
					"removeObstacles")) {
		obstacleOverlapPolicy = RobogenConfig::REMOVE_OBSTACLES;
	} else if(vm["obstacleOverlapPolicy"].as<std::string>() ==
					"constraintViolation") {
		obstacleOverlapPolicy = RobogenConfig::CONSTRAINT_VIOLATION;
	} else if(vm["obstacleOverlapPolicy"].as<std::string>() ==
			"elevateRobot") {
		obstacleOverlapPolicy = RobogenConfig::ELEVATE_ROBOT;
	} else {
		std::cerr << "Invalid value: '" <<
				vm["obstacleOverlapPolicy"].as<std::string>() <<
				"' given for 'obstacleOverlapPolicy'" << std::endl;
		return boost::shared_ptr<RobogenConfig>();
	}

	return boost::shared_ptr<RobogenConfig>(
			new RobogenConfig(scenario, scenarioFile, nTimeSteps,
					timeStep, actuationPeriod, terrain,
					obstacles, obstaclesConfigFile, startPositions,
					startPositionFile, lightSources, lightSourcesFile,
					sensorNoiseLevel,
					motorNoiseLevel, capAcceleration, maxLinearAcceleration,
					maxAngularAcceleration, maxDirectionShiftsPerSecond,
					gravity, disallowObstacleCollisions,
					obstacleOverlapPolicy));

}

const std::string getMatchNFloatPattern(unsigned int n) {
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
	while (!RobogenUtils::safeGetline(obstaclesFile, line).eof()) {
		lineNum++;
		boost::cmatch match;
		float x, y, z, xSize, ySize, zSize, density, xRotation, yRotation,
				zRotation, rotationAngle;
		if(boost::regex_match(line.c_str(), match,
						fullObstacleRegex)){
			x = parse_float(match[1].str());
			y = parse_float(match[2].str());
			z = parse_float(match[3].str());
			xSize = parse_float(match[4].str());
			ySize = parse_float(match[5].str());
			zSize = parse_float(match[6].str());
			density = parse_float(match[7].str());
			xRotation = parse_float(match[8].str());
			yRotation = parse_float(match[9].str());
			zRotation = parse_float(match[10].str());
			rotationAngle = parse_float(match[11].str());
		} else {
			xRotation = yRotation = zRotation = rotationAngle = 0.0;
			if (boost::regex_match(line.c_str(), match, oldObstacleRegex)){
				x = parse_float(match[1].str());
				y = parse_float(match[2].str());
				xSize = parse_float(match[3].str());
				ySize = parse_float(match[4].str());
				zSize = parse_float(match[5].str());
				density = parse_float(match[6].str());

				z = zSize/2;
			} else if(boost::regex_match(line.c_str(), match,
					noRotationObstacleRegex)){
				x = parse_float(match[1].str());
				y = parse_float(match[2].str());
				z = parse_float(match[3].str());
				xSize = parse_float(match[4].str());
				ySize = parse_float(match[5].str());
				zSize = parse_float(match[6].str());
				density = parse_float(match[7].str());
			} else {
				std::cerr << "Error parsing line " << lineNum <<
						" of obstacles file: '" << fileName << "'"
						<< std::endl;
				std::cerr << line.c_str() << std::endl;
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

boost::shared_ptr<LightSourcesConfig> ConfigurationReader::parseLightSourcesFile(
		const std::string& fileName) {

	std::ifstream lightSourcesFile(fileName.c_str());
	if (!lightSourcesFile.is_open()) {
		std::cout << "Cannot find light sources file: '" << fileName << "'"
				<< std::endl;
		return boost::shared_ptr<LightSourcesConfig>();
	}

	// either 3 or 4 numbers (x,y,z) or (x,y,z,intensity)
	static const boost::regex noIntensityRegex(getMatchNFloatPattern(3));
	static const boost::regex fullRegex(getMatchNFloatPattern(4));

	std::vector<osg::Vec3> coordinates;
	std::vector<float> intensities;

	std::string line;
	int lineNum = 0;
	while (!RobogenUtils::safeGetline(lightSourcesFile, line).eof()) {
		lineNum++;
		boost::cmatch match;
		float x, y, z, intensity;
		if(boost::regex_match(line.c_str(), match, fullRegex)){
			intensity = parse_float(match[4].str());
		} else {
			intensity = 1.0;
			if(!boost::regex_match(line.c_str(), match, noIntensityRegex)){

				std::cerr << "Error parsing line " << lineNum <<
						" of light sources file: '" << fileName << "'"
						<< std::endl;
				std::cerr << line.c_str() << std::endl;
				return boost::shared_ptr<LightSourcesConfig>();
			}
		}
		x = parse_float(match[1].str());
		y = parse_float(match[2].str());
		z = parse_float(match[3].str());
		coordinates.push_back(osg::Vec3(x, y, z));
		intensities.push_back(intensity);

	}

	return boost::shared_ptr<LightSourcesConfig>(
			new LightSourcesConfig(coordinates, intensities));
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

	// Decode light sources
	std::vector<osg::Vec3> lightSourcesCoords;
	std::vector<float> lightSourcesIntensities;
	for (int i = 0; i < simulatorConf.lightsources_size(); ++i) {
		const robogenMessage::LightSource& ls = simulatorConf.lightsources(i);
		lightSourcesCoords.push_back(osg::Vec3(ls.x(), ls.y(), ls.z()));
		lightSourcesIntensities.push_back(ls.intensity());
	}
	boost::shared_ptr<LightSourcesConfig> lightSources(
				new LightSourcesConfig(lightSourcesCoords,
						lightSourcesIntensities));


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
	boost::shared_ptr<TerrainConfig> terrain;
	if(simulatorConf.terraintype() == TerrainConfig::EMPTY) {
		terrain.reset(new TerrainConfig(simulatorConf.terrainfriction()));
	} else if(simulatorConf.terraintype() == TerrainConfig::FLAT) {
		terrain.reset(new TerrainConfig(simulatorConf.terrainlength(),
					simulatorConf.terrainwidth(),
					simulatorConf.terrainfriction()));
	} else if(simulatorConf.terraintype() == TerrainConfig::ROUGH) {
		terrain.reset(new TerrainConfig(
							simulatorConf.terrainheightfieldfilename(),
							simulatorConf.terrainlength(),
							simulatorConf.terrainwidth(),
							simulatorConf.terrainheight(),
							simulatorConf.terrainfriction()));
	}

	// Decode simulator configuration
	std::string scenario = simulatorConf.scenario();

	//todo with js check!!

	//if (scenario.compare("racing") != 0 && scenario.compare("chasing") != 0) {
	//	std::cerr << "Undefined 'scenario' parameter" << std::endl;
	//	return boost::shared_ptr<RobogenConfig>();
	//}

	unsigned int timeSteps = simulatorConf.ntimesteps();
	float timeStepLength = simulatorConf.timestep();
	int actuationPeriod = simulatorConf.actuationperiod();

	return boost::shared_ptr<RobogenConfig>(
			new RobogenConfig(scenario, "", timeSteps, timeStepLength,
					actuationPeriod, terrain, obstacles, "",
					boost::shared_ptr<StartPositionConfig>(
							new StartPositionConfig(startPositions)), "",
					lightSources, "", simulatorConf.sensornoiselevel(),
					simulatorConf.motornoiselevel(),
					simulatorConf.capacceleration(),
					simulatorConf.maxlinearacceleration(),
					simulatorConf.maxangularacceleration(),
					simulatorConf.maxdirectionshiftspersecond(),
					osg::Vec3(simulatorConf.gravityx(),
							  simulatorConf.gravityy(),
							  simulatorConf.gravityz()),
					simulatorConf.disallowobstaclecollisions(),
					simulatorConf.obstacleoverlappolicy()
					));

}

}
