/*
 * @(#) BrainEvolver.cpp   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
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

#include <boost/shared_ptr.hpp>
#include "config/EvolverConfiguration.h"
#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/EvolverLog.h"
#include "evolution/engine/Population.h"
#include "evolution/engine/Selector.h"
#include "evolution/engine/Mutator.h"
#include "evolution/engine/Selectors/DeterministicTournament.h"

#include "evolution/engine/neat/NeatContainer.h"

using namespace robogen;

void printUsage(char *argv[]) {
	std::cout << std::endl << "USAGE: " << std::endl
			<< "      " << std::string(argv[0])
			<< " <SEED, INTEGER> <OUTPUT_DIRECTORY, STRING> "
			<< "<CONFIGURATION_FILE, STRING> [<OPTIONS>]"
			<< std::endl << std::endl
			<< "WHERE: " << std::endl
			<< "      <SEED> is a number to seed "
			<< "the pseudorandom number generator."
			<< std::endl << std::endl
			<< "      <OUTPUT_DIRECTORY> is the directory to write output "
			<< "files to."
			<< std::endl << std::endl
			<< "      <CONFIGURATION_FILE> is the evolution"
			<< " configuration file to use."
			<< std::endl << std::endl
			<< "OPTIONS: " << std::endl
			<< "      --help" << std::endl
			<< "          Print all configuration options and exit."
			<< std::endl << std::endl
			<< "      --overwrite" << std::endl
			<< "          Overwrite existing output file directory if it "
			<< "exists." << std::endl
			<< "          (Default is to keep creating new output "
			<< "directories with incrementing suffixes)."
			<< std::endl << std::endl
			<< "      --save-all" << std::endl
			<< "          Save all individuals instead of just the generation"
			<< "best."
			<< std::endl << std::endl;

}

void printHelp() {
	boost::shared_ptr<EvolverConfiguration> conf =
				boost::shared_ptr<EvolverConfiguration>(
						new EvolverConfiguration());
	conf->init("help");
	std::cout << std::endl;
	boost::shared_ptr<RobogenConfig> robotConf =
				ConfigurationReader::parseConfigurationFile("help");
}

int main(int argc, char *argv[]) {

	startRobogen();

	// ---------------------------------------
	// verify usage and load configuration
	// ---------------------------------------
	if (argc > 1 && std::string(argv[1]) == "--help") {
		printUsage(argv);
		printHelp();
		exitRobogen(EXIT_SUCCESS);
	}

	if ((argc < 4)) {
		printUsage(argv);
		std::cout << "RUN: " << std::endl << std::endl
				<< "      " << std::string(argv[0])
				<< " --help to see all configuration options."
				<< std::endl << std::endl;
		exitRobogen(EXIT_FAILURE);
	}


	unsigned int seed = atoi(argv[1]);
	std::string outputDirectory = std::string(argv[2]);
	std::string confFileName = std::string(argv[3]);

	bool overwrite = false;
	bool saveAll = false;
	int currentArg = 4;
	for (; currentArg<argc; currentArg++) {
		if (std::string("--help").compare(argv[currentArg]) == 0) {
			printUsage(argv);
			printHelp();
			return EXIT_SUCCESS;
		} else if (std::string("--overwrite").compare(argv[currentArg]) == 0) {
			overwrite = true;
		} else if (std::string("--save-all").compare(argv[currentArg]) == 0) {
			saveAll = true;
		} else {
			std::cerr << std::endl << "Invalid option: " << argv[currentArg]
							 << std::endl << std::endl;
			printUsage(argv);
			exitRobogen(EXIT_FAILURE);
		}


	}


	// Create random number generator
	boost::random::mt19937 rng;
	rng.seed(seed);

	boost::shared_ptr<EvolverConfiguration> conf =
			boost::shared_ptr<EvolverConfiguration>(new EvolverConfiguration());
	if (!conf->init(confFileName)) {
		std::cerr << "Problems parsing the evolution configuration file. Quit."
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	boost::shared_ptr<RobogenConfig> robotConf =
			ConfigurationReader::parseConfigurationFile(
					conf->simulatorConfFile);
	if (robotConf == NULL) {
		std::cerr << "Problems parsing the robot configuration file. Quit."
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	// ---------------------------------------
	// Set up evolution
	// ---------------------------------------

	boost::shared_ptr<Selector> selector;
	if (conf->selection == conf->DETERMINISTIC_TOURNAMENT) {
		selector.reset(new DeterministicTournament(conf->tournamentSize, rng));
	} else {
		std::cerr << "Selection type id " << conf->selection << " unknown."
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	}
	boost::shared_ptr<Mutator> mutator = boost::shared_ptr<Mutator>(
			new Mutator(conf, rng));
	boost::shared_ptr<EvolverLog> log(new EvolverLog());
	if (!log->init(conf, robotConf, outputDirectory, overwrite, saveAll)) {
		std::cout << "Error creating evolver log. Aborting." << std::endl;
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// parse robot from file & initialize population
	// ---------------------------------------

	boost::shared_ptr<RobotRepresentation> referenceBot(
			new RobotRepresentation());
	bool growBodies = false;
	if (conf->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER
			&& conf->referenceRobotFile.compare("") == 0) {
		std::cerr << "Trying to evolve brain, but no robot file provided."
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	} else if (conf->referenceRobotFile.compare("") != 0) {
		if (!referenceBot->init(conf->referenceRobotFile)) {
			std::cerr << "Failed interpreting robot from text file"
					<< std::endl;
			exitRobogen(EXIT_FAILURE);
		}
	} else { //doing body evolution and don't have a reference robot
		if (referenceBot->init()) {
			growBodies = true;
		} else {
			std::cerr << "Failed creating base robot for body evolution"
					<< std::endl;
			exitRobogen(EXIT_FAILURE);
		}
	}
	

	boost::shared_ptr<Population> population(new Population());
	boost::shared_ptr<NeatContainer> neatContainer;

	bool neat = (conf->evolutionaryAlgorithm ==
			EvolverConfiguration::HYPER_NEAT);

	if (!population->init(referenceBot, conf->mu, mutator, growBodies,
			(!(conf->useBrainSeed || neat)) ) ) {
		std::cerr << "Error when initializing population!" << std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	if (neat) {
		neatContainer.reset(new NeatContainer(conf, population, seed, rng));

	}




	// ---------------------------------------
	// open sockets for communication with simulator processes
	// ---------------------------------------

	std::vector<TcpSocket*> sockets(conf->sockets.size());
	for (unsigned int i = 0; i < conf->sockets.size(); i++) {
		sockets[i] = new TcpSocket;
#ifndef FAKEROBOTREPRESENTATION_H // do not bother with sockets when using
		// benchmark
		if (!sockets[i]->open(conf->sockets[i].first,
				conf->sockets[i].second)) {
			std::cerr << "Could not open connection to simulator" << std::endl;
			exitRobogen(EXIT_FAILURE);
		}
#endif
	}

	// ---------------------------------------
	// run evolution TODO stopping criterion
	// ---------------------------------------

	if(neat) {
		if(!neatContainer->fillPopulationWeights(population)) {
			std::cerr << "Filling weights from NEAT failed." << std::endl;
			exitRobogen(EXIT_FAILURE);
		}
	}

	population->evaluate(robotConf, sockets);
	if (!log->logGeneration(1, *population.get())) {
		exitRobogen(EXIT_FAILURE);
	}

	for (unsigned int generation = 2; generation <= conf->numGenerations;
			++generation) {
		// create children
		IndividualContainer children;
		if (neat) {
			//neatPopulation->Epoch();
			if(!neatContainer->produceNextGeneration(population)) {
				std::cerr << "Producing next generation from NEAT failed."
						<< std::endl;
				exitRobogen(EXIT_FAILURE);
			}
			population->evaluate(robotConf, sockets);
			population->sort(true);
		} else {
			selector->initPopulation(population);
			for (unsigned int i = 0; i < conf->lambda; i++) {
				std::pair<boost::shared_ptr<RobotRepresentation>,
						boost::shared_ptr<RobotRepresentation> > selection;
				if (!selector->select(selection)) {
					std::cerr << "Selector::select() failed." << std::endl;
					exitRobogen(EXIT_FAILURE);
				}
				children.push_back(
						mutator->mutate(selection.first, selection.second));
			}

			// evaluate children
			children.evaluate(robotConf, sockets);

			// comma or plus?
			if (conf->replacement == conf->PLUS_REPLACEMENT) {
				children += *population.get();
			}

			// replace
			population.reset(new Population());
			if (!population->init(children, conf->mu)) {
				std::cerr << "Error when initializing population!" << std::endl;
				exitRobogen(EXIT_FAILURE);
			}
		}


		if (!log->logGeneration(generation, *population.get())) {
			exitRobogen(EXIT_FAILURE);
		}
	}

	// Clean up sockets
	for (unsigned int i = 0; i < conf->sockets.size(); i++) {
		delete sockets[i];
	}

	exitRobogen(EXIT_SUCCESS);
}
