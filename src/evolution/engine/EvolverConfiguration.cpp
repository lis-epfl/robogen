/*
 * @(#) EvolverConfiguration.cpp   1.0   Sep 2, 2013
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

#include "evolution/engine/EvolverConfiguration.h"
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

namespace robogen {

/**
 * Parses options from given conf file.
 * @todo default values
 */
void EvolverConfiguration::init(std::string confFileName) {
	// boost-parse options
	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("referenceRobotFile",
				boost::program_options::value<std::string>(&referenceRobotFile),
				"Path to the reference robot file")
		("populationSize",
				boost::program_options::value<unsigned int>(&populationSize)
				->required(), "Size of population")
		("numGenerations",
				boost::program_options::value<unsigned int>(&numGenerations)
				->required(), "Amount of generations to be evaluated")
		("numSelect",
				boost::program_options::value<unsigned int>(&numSelect),
				"Amount of best individuals to be selected by rank")
		("pBrainMutate", boost::program_options::value<double>(&pBrainMutate),
				"Probability of mutation for any single brain parameter")
		("brainSigma", boost::program_options::value<double>(&brainSigma),
				"Sigma of brain parameter mutation")
		("pBrainCrossover",
				boost::program_options::value<double>(&pBrainCrossover),
				"Probability of crossover among brains")
		("socket", boost::program_options::value<std::vector<std::string> >(),
				"Sockets to be used to connect to the server");
	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_config_file<char>(
					confFileName.c_str(), desc, true), vm);
	boost::program_options::notify(vm);

	// parse sockets. The used regex is not super-restrictive, but we count
	// on the TcpSocket to throw... else:
	// http://www.regular-expressions.info/examples.html
	static const boost::regex socketRegex("^([\\d\\.]*):(\\d*)$");
	if (!vm.count("socket")){
		// TODO exception
	}
	std::vector<std::string> encSocket =
			vm["socket"].as<std::vector<std::string> >();
	sockets.clear();
	for (unsigned int i = 0; i<encSocket.size(); i++){
		boost::cmatch match;
		// match[0]:whole string, match[1]:IP, match[2]:port
		if (!boost::regex_match(encSocket[i].c_str(), match, socketRegex)){
			// TODO exception
		}
		sockets.push_back(std::pair<std::string, int>(std::string(match[1]),
				std::atoi(match[2].first)));
	}

	// TODO verify configuration validity
}

}
