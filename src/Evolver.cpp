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

int exitRobogen(int exitCode) {
	google::protobuf::ShutdownProtobufLibrary();
	return exitCode;
}

int main(int argc, char *argv[]) {

	// ---------------------------------------
	// verify usage and load configuration
	// ---------------------------------------
	if (argc > 1 && std::string(argv[1]) == "--help") {
		std::cout << " Usage: robogen-evolver "
							"<seed, INTEGER>, <output directory, STRING>, "
							"<configuration file, STRING>" << std::endl;
		boost::shared_ptr<EvolverConfiguration> conf =
					boost::shared_ptr<EvolverConfiguration>(
							new EvolverConfiguration());
		conf->init("help");
		std::cout << std::endl;
		boost::shared_ptr<RobogenConfig> robotConf =
					ConfigurationReader::parseConfigurationFile("help");
		return exitRobogen(EXIT_SUCCESS);
	}

	if (argc != 4) {
		std::cout << "Bad amount of arguments. " << std::endl
				<< " Usage: robogen-evolver "
						"<seed, INTEGER>, <output directory, STRING>, "
						"<configuration file, STRING>" << std::endl;
		return exitRobogen(EXIT_FAILURE);
	}

	unsigned int seed = atoi(argv[1]);
	std::string outputDirectory = std::string(argv[2]);
	std::string confFileName = std::string(argv[3]);

	// Create random number generator
	boost::random::mt19937 rng;
	rng.seed(seed);

	boost::shared_ptr<EvolverConfiguration> conf =
			boost::shared_ptr<EvolverConfiguration>(new EvolverConfiguration());
	if (!conf->init(confFileName)) {
		std::cout << "Problems parsing the evolution configuration file. Quit."
				<< std::endl;
		return exitRobogen(EXIT_FAILURE);
	}

	boost::shared_ptr<RobogenConfig> robotConf =
			ConfigurationReader::parseConfigurationFile(
					conf->simulatorConfFile);
	if (robotConf == NULL) {
		std::cout << "Problems parsing the robot configuration file. Quit."
				<< std::endl;
		return exitRobogen(EXIT_FAILURE);
	}

	// ---------------------------------------
	// Set up evolution
	// ---------------------------------------

	boost::shared_ptr<Selector> selector;
	if (conf->selection == conf->DETERMINISTIC_TOURNAMENT) {
		selector.reset(new DeterministicTournament(conf->tournamentSize, rng));
	} else {
		std::cout << "Selection type id " << conf->selection << " unknown."
				<< std::endl;
		return exitRobogen(EXIT_FAILURE);
	}
	boost::shared_ptr<Mutator> mutator = boost::shared_ptr<Mutator>(
			new Mutator(conf, rng));
	boost::shared_ptr<EvolverLog> log(new EvolverLog());
	if (!log->init(conf, robotConf, outputDirectory)) {
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
		std::cout << "Trying to evolve brain, but no robot file provided."
				<< std::endl;
		return exitRobogen(EXIT_FAILURE);
	} else if (conf->referenceRobotFile.compare("") != 0) {
		if (!referenceBot->init(conf->referenceRobotFile)) {
			std::cout << "Failed interpreting robot from text file"
					<< std::endl;
			return exitRobogen(EXIT_FAILURE);
		}
	} else { //doing body evolution and don't have a reference robot
		if (referenceBot->init()) {
			growBodies = true;
		} else {
			std::cout << "Failed creating base robot for body evolution"
					<< std::endl;
			return exitRobogen(EXIT_FAILURE);
		}
	}
	

	boost::shared_ptr<Population> population(new Population());
	boost::shared_ptr<NeatContainer> neatContainer;

	bool hyperNEAT = (conf->evolutionaryAlgorithm ==
			EvolverConfiguration::HYPER_NEAT);

	if (!population->init(referenceBot, conf->mu, mutator, growBodies,
			(!(conf->useBrainSeed || hyperNEAT)) ) ) {
		std::cout << "Error when initializing population!" << std::endl;
		return exitRobogen(EXIT_FAILURE);
	}

	if (hyperNEAT) {
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
			std::cout << "Could not open connection to simulator" << std::endl;
			return exitRobogen(EXIT_FAILURE);
		}
#endif
	}

	// ---------------------------------------
	// run evolution TODO stopping criterion
	// ---------------------------------------

	if(hyperNEAT) {
		if(!neatContainer->fillPopulationWeights(population)) {
			std::cout << "Filling weights from NEAT failed." << std::endl;
			return exitRobogen(EXIT_FAILURE);
		}
	}

	population->evaluate(robotConf, sockets);
	if (!log->logGeneration(1, *population.get())) {
		return exitRobogen(EXIT_FAILURE);
	}

	for (unsigned int generation = 2; generation <= conf->numGenerations;
			++generation) {
		// create children
		IndividualContainer children;
		if (hyperNEAT) {
			//neatPopulation->Epoch();
			if(!neatContainer->produceNextGeneration(population)) {
				std::cout << "Producing next generation from NEAT failed."
						<< std::endl;
				return exitRobogen(EXIT_FAILURE);
			}
			population->evaluate(robotConf, sockets);
			population->sort(true);
		} else {
			selector->initPopulation(population);
			for (unsigned int i = 0; i < conf->lambda; i++) {
				std::pair<boost::shared_ptr<RobotRepresentation>,
						boost::shared_ptr<RobotRepresentation> > selection;
				if (!selector->select(selection)) {
					std::cout << "Selector::select() failed." << std::endl;
					return exitRobogen(EXIT_FAILURE);
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
				std::cout << "Error when initializing population!" << std::endl;
				return exitRobogen(EXIT_FAILURE);
			}
		}


		if (!log->logGeneration(generation, *population.get())) {
			return exitRobogen(EXIT_FAILURE);
		}
	}

	// Clean up sockets
	for (unsigned int i = 0; i < conf->sockets.size(); i++) {
		delete sockets[i];
	}

	return exitRobogen(EXIT_SUCCESS);
}
