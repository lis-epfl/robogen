/*
 * @(#) FileViewer.cpp   1.0   Mar 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Titus Cieslweski, Joshua Auerbach
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
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "config/ConfigurationReader.h"
#include "config/RobogenConfig.h"
#include "evolution/representation/RobotRepresentation.h"
#include "scenario/Scenario.h"
#include "scenario/ScenarioFactory.h"
#include "utils/network/ProtobufPacket.h"
#include "utils/network/TcpSocket.h"
#include "utils/json2pb/json2pb.h"
#include "utils/RobogenCollision.h"
#include "utils/RobogenUtils.h"
#include "viewer/KeyboardHandler.h"
#include "viewer/FileViewerLog.h"
#include "Models.h"
#include "RenderModels.h"
#include "Robogen.h"
#include "Robot.h"
#include "robogen.pb.h"

#include "Simulator.h"


using namespace robogen;

// ODE World
dWorldID odeWorld;

// Container for collisions
dJointGroupID odeContactGroup;

bool interrupted;

/**
 * Decodes a robot saved on file and visualize it
 */
int main(int argc, char *argv[]) {

	if (argc < 3) {
		std::cout << "Please provide a file containing the robot description as"
				" input and the corresponding simulation configuration file. "
				<< std::endl << "For example: " << std::string(argv[0])
				<< " robot.json configuration.conf'" << std::endl << "or"
				<< std::endl << "For example: " << std::string(argv[0])
				<< " robot.txt configuration.conf'" << std::endl
				<< "You can also select the starting position by "
						"appending an integer 1..n to the command" << std::endl
				<< "To save frames to file (for video rendering), use option "
				<< "--record <N, INTEGER> <DIR, STRING>, "
				<< "to save every Nth frame in directory DIR"
				<< std::endl
				<< "To generate output files: sensor logs and Arduino files, "
				<< "use option --output <DIR_POSTFIX, STRING>"
				<< std::endl
				<< "To evaluate an individual without the visualization, "
				<< "use option --no-visualization"
				<< std::endl
				<< "Note, cannot record frames without visualization."
				<< std::endl;
		return EXIT_FAILURE;
	}

	// Decode configuration file
	boost::shared_ptr<RobogenConfig> configuration =
			ConfigurationReader::parseConfigurationFile(std::string(argv[2]));
	if (configuration == NULL) {
		std::cerr << "Problems parsing the configuration file. Quit."
				<< std::endl;
		return EXIT_FAILURE;
	}

	// verify desired start position is specified in configuration
	unsigned int desiredStart = 0;
	unsigned int recordFrequency = 0;
	bool recording = false;
	char *recordDirectoryName = NULL;

	bool writeLog = false;
	char *outputDirectoryName;


	int currentArg = 3;
	if (argc >= 4 && !boost::starts_with(argv[3], "--")) {
		std::stringstream ss(argv[3]);
		currentArg++;
		ss >> desiredStart;
		--desiredStart; // -- accounts for parameter being 1..n
		if (ss.fail()) {
			std::cerr << "Specified desired starting position \"" << argv[3]
					<< "\" is not an integer. Aborting..." << std::endl;
			return EXIT_FAILURE;
		}
		if (desiredStart
				>= configuration->getStartingPos()->getStartPosition().size()) {
			std::cout << "Specified desired starting position " << argv[3]
					<< " does not index a starting position. Aborting..."
					<< std::endl;
			return EXIT_FAILURE;
		}
	}
	bool visualize = true;
	bool startPaused = false;
	for (; currentArg<argc; currentArg++) {
		if (std::string("--record").compare(argv[currentArg]) == 0) {
			if (argc < (currentArg + 3)) {
				std::cerr << "In order to record frames, must provide frame "
						<< "frequency and target directory."
						<< std::endl;
						return EXIT_FAILURE;
			}
			recording = true;
			currentArg++;
			std::stringstream ss(argv[currentArg]);
			ss >> recordFrequency;
			if (ss.fail()) {
				std::cout << "Specified record frequency \"" << argv[currentArg]
						<< "\" is not an integer. Aborting..." << std::endl;
				return EXIT_FAILURE;
			}
			currentArg++;

			recordDirectoryName = argv[currentArg];
			boost::filesystem::path recordDirectory(recordDirectoryName);

			if (recording && !boost::filesystem::is_directory(recordDirectory) ) {
				boost::filesystem::create_directories(recordDirectory);
			}

		} else if (std::string("--output").compare(argv[currentArg]) == 0) {
			if (argc < (currentArg + 2)) {
				std::cerr << "In order to write output files, must provide "
										<< "directory postfix."
										<< std::endl;
										return EXIT_FAILURE;

			}
			writeLog = true;
			currentArg++;

			outputDirectoryName = argv[currentArg];
		} else if (std::string("--no-visualization").compare(argv[currentArg]) == 0) {
			visualize = false;
		} else if (std::string(argv[currentArg]).compare("--pause") == 0) {
			startPaused = true;
		}

	}

	if (recording && !visualize) {
		std::cerr << "Cannot record without visualization enabled!" <<
				std::endl;
		return EXIT_FAILURE;
	}

	if (startPaused && !visualize) {
		std::cerr << "Cannot start paused without visualization enabled." <<
				std::endl;
		return EXIT_FAILURE;
	}




		// ---------------------------------------
	// Robot decoding
	// ---------------------------------------
	robogenMessage::Robot robotMessage;
	std::string robotFileString(argv[1]);

	if (boost::filesystem::path(argv[1]).extension().string().compare(".dat") == 0) {

		std::ifstream robotFile(argv[1], std::ios::binary);
		if (!robotFile.is_open()) {
			std::cout << "Cannot open " << std::string(argv[1]) << ". Quit."
					<< std::endl;
			return EXIT_FAILURE;
		}

		ProtobufPacket<robogenMessage::Robot> robogenPacket;

		robotFile.seekg(0, robotFile.end);
		unsigned int packetSize = robotFile.tellg();
		robotFile.seekg(0, robotFile.beg);

		std::vector<unsigned char> packetBuffer;
		packetBuffer.resize(packetSize);
		robotFile.read((char*) &packetBuffer[0], packetSize);
		robogenPacket.decodePayload(packetBuffer);
		robotMessage = *robogenPacket.getMessage().get();

	} else if (boost::filesystem::path(argv[1]).extension().string().compare(".txt") == 0) {

		RobotRepresentation robot;
		if (!robot.init(argv[1])) {
			std::cerr << "Failed interpreting robot text file!" << std::endl;
			return EXIT_FAILURE;
		}
		robotMessage = robot.serialize();

	} else if (boost::filesystem::path(argv[1]).extension().string().compare(".json") == 0) {

		std::ifstream robotFile(argv[1], std::ios::in | std::ios::binary);
		if (!robotFile.is_open()) {
			std::cerr << "Cannot open " << std::string(argv[1]) << ". Quit."
					<< std::endl;
			return EXIT_FAILURE;
		}

		robotFile.seekg(0, robotFile.end);
		unsigned int packetSize = robotFile.tellg();
		robotFile.seekg(0, robotFile.beg);

		std::vector<unsigned char> packetBuffer;
		packetBuffer.resize(packetSize);
		robotFile.read((char*) &packetBuffer[0], packetSize);

		json2pb(robotMessage, (char*) &packetBuffer[0], packetSize);

	} else {
		std::cerr << "File extension of provided robot file could not be "
				"resolved. Use .dat or .json for robot messages and .txt for "
				"robot text files" << std::endl;
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// Setup environment
	// ---------------------------------------

	boost::shared_ptr<Scenario> scenario = ScenarioFactory::createScenario(
			configuration);
	if (scenario == NULL) {
		return EXIT_FAILURE;
	}
	scenario->setStartingPosition(desiredStart);


	// ---------------------------------------
	// Set up log files
	// ---------------------------------------

	boost::shared_ptr<FileViewerLog> log;

	if (writeLog) {
		log.reset(new FileViewerLog(std::string(argv[1]),
			std::string(argv[2]), configuration->getObstacleFile(),
			configuration->getStartPosFile(),
			std::string(outputDirectoryName)));
	}

	// ---------------------------------------
	// Run simulations
	// ---------------------------------------
	unsigned int simulationResult = runSimulations(scenario,
			configuration, robotMessage,
			visualize, startPaused, true, log, recording, recordFrequency,
			recordDirectoryName);

	if (simulationResult == SIMULATION_FAILURE) {
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// Compute fitness
	// ---------------------------------------
	double fitness;
	if (simulationResult == ACCELERATION_CAP_EXCEEDED) {
		fitness = MIN_FITNESS;
	} else {
		fitness = scenario->getFitness();
	}
	std::cout << "Fitness for the current solution: " << fitness
			<< std::endl << std::endl;

	return EXIT_SUCCESS;
}
