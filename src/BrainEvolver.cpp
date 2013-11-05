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
#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/EvolverConfiguration.h"
#include "evolution/engine/EvolverLog.h"
#include "evolution/engine/Population.h"
#include "evolution/engine/Selector.h"
#include "evolution/engine/Mutator.h"
#include "evolution/engine/Selectors/DeterministicTournament.h"

using namespace robogen;

int main(int argc, char *argv[]) {

	// create random number generator
	boost::random::mt19937 rng;

	// ---------------------------------------
	// verify usage and load configuration
	// ---------------------------------------

	if (argc != 2) {
		std::cout << "Bad amount of arguments. Usage: robogen-brain-evolver "
				"<configuration file>" << std::endl;
		return EXIT_FAILURE;
	}
	struct EvolverConfiguration conf;
	if (!conf.init(std::string(argv[1]))) {
		std::cout
				<< "Problems parsing the evolution configuration file. Quit."
				<< std::endl;
		return EXIT_FAILURE;
	}

	boost::shared_ptr<RobogenConfig> robotConf =
			ConfigurationReader::parseConfigurationFile(conf.simulatorConfFile);
	if (robotConf == NULL) {
		std::cout
				<< "Problems parsing the robot configuration file. Quit."
				<< std::endl;
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// Set up evolution
	// ---------------------------------------

	boost::shared_ptr<Selector> s;
	if (conf.selection == conf.DETERMINISTIC_TOURNAMENT) {
		s.reset(new DeterministicTournament(conf.tournamentSize, rng));
	} else {
		std::cout << "Selection type id " << conf.selection << " unknown."
				<< std::endl;
		return EXIT_FAILURE;
	}
	Mutator m(conf.pBrainMutate, conf.brainSigma, conf.pBrainCrossover,
			conf.minBrainWeight, conf.maxBrainWeight, rng);
	boost::shared_ptr<EvolverLog> log(new EvolverLog());
	if (!log->init(std::string(argv[1]))) {
		std::cout << "Error creating evolver log. Aborting." << std::endl;
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// parse robot from file & initialize population
	// ---------------------------------------

	RobotRepresentation referenceBot;
	if (!referenceBot.init(conf.referenceRobotFile)) {
		std::cout << "Failed interpreting robot from text file" << std::endl;
		return EXIT_FAILURE;
	}
	boost::shared_ptr<Population> current(new Population()), previous;
	if (!current->init(referenceBot, conf.mu, rng)) {
		std::cout << "Error when intializing population!" << std::endl;
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// open sockets for communication with simulator processes
	// ---------------------------------------

	std::vector<TcpSocket*> sockets(conf.sockets.size());
	for (unsigned int i = 0; i < conf.sockets.size(); i++) {
		sockets[i] = new TcpSocket;
#ifndef FAKEROBOTREPRESENTATION_H // do not bother with sockets when using
		// benchmark
		if (!sockets[i]->open(conf.sockets[i].first, conf.sockets[i].second)) {
			std::cout << "Could not open connection to simulator" << std::endl;
			return EXIT_FAILURE;
		}
#endif
	}


	// ---------------------------------------
	// run evolution TODO stopping criterion
	// ---------------------------------------

	current->evaluate(robotConf, sockets);
	if (!log->logGeneration(1, *current.get())) {
		return EXIT_FAILURE;
	}

	for (unsigned int generation = 2; generation <= conf.numGenerations;
			++generation) {
		// create children
		IndividualContainer children;
		s->initPopulation(current);
		for (unsigned int i = 0; i < conf.lambda; i++) {
			boost::shared_ptr<
					std::pair<RobotRepresentation, RobotRepresentation> > selection;
			if (!s->select(selection)) {
				std::cout << "Selector::select() failed." << std::endl;
				return EXIT_FAILURE;
			}
			children.push_back(m.mutate(*selection.get()));
			// TODO mutate() error handling?
		}
		// evaluate children
		children.evaluate(robotConf, sockets);
		// comma or plus?
		if (conf.replacement == conf.PLUS_REPLACEMENT) {
			children += *current.get();
		}
		// replace
		current.reset(new Population());
		if (!current->init(children, conf.mu)) {
			std::cout << "Error when intializing population!" << std::endl;
			return EXIT_FAILURE;
		}
		if (!log->logGeneration(generation, *current.get()))
			return EXIT_FAILURE;
	}
}
