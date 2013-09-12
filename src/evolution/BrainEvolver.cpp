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
#include "evolution/engine/Replacer.h"

using namespace robogen;

int main(int argc, char *argv[]){
	// verify usage and load configuration
	if (argc != 2){
		std::cout << "Bad amount of arguments. Usage: robogen-brain-evolver "\
				"<configuration file>" << std::endl;
		return EXIT_FAILURE;
	}
	struct EvolverConfiguration conf;
	conf.init(std::string(argv[1]));

	// create random number generator
	boost::random::mt19937 rng;

	// set up evolution
	Selector s(conf.numSelect,rng);
	Mutator m(conf.pBrainMutate, conf.brainSigma, conf.pBrainCrossover, rng);
	Replacer r(conf.numReplace);
	boost::shared_ptr<EvolverLog>log(new EvolverLog(std::string(argv[1])));

	// parse robot from file & initialize population
	RobotRepresentation referenceBot(conf.referenceRobotFile);
	boost::shared_ptr<Population> current(new Population(
			referenceBot,conf.populationSize,rng)),	previous;

	// open sockets for communication with simulator processes
	std::vector<TcpSocket*> sockets(conf.sockets.size());
	for (unsigned int i=0; i<conf.sockets.size(); i++){
		sockets[i] = new TcpSocket;
#ifndef FAKEROBOTREPRESENTATION_H // do not bother with sockets when using
		// benchmark
		if(!sockets[i]->open(conf.sockets[i].first, conf.sockets[i].second)){
			std::cout << "Could not open connection to simulator" << std::endl;
			return EXIT_FAILURE;
		}
#endif
	}

	// run evolution TODO stopping criterion
	current->evaluate(conf.simulatorConfFile,sockets);
	log->logGeneration(1,*current.get());
	for (unsigned int generation=2; generation<=conf.numGenerations;
			++generation){
		// create children
		IndividualContainer children;
		for (unsigned int i = 0; i<conf.lambda; i++){
			// don't forget to copy construct!!!
			children.push_back(m.mutate(s.select()));
		}
		// evaluate children
		children.evaluate(conf.simulatorConfFile, sockets);
		// comma or plus?
		if (conf.replacement == conf.PLUS_REPLACEMENT){
			children += *current.get();
		}
		// replace
		current.reset(new Population(children, conf.mu));
		log->logGeneration(i,*current.get());
	}
}
