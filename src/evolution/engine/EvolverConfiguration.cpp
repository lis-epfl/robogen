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
#include <sstream>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

namespace robogen {

EvolverConfigurationException::EvolverConfigurationException(
		const std::string& w) : std::runtime_error(w){}

/**
 * Parses options from given conf file.
 * @todo default values
 */
void EvolverConfiguration::init(std::string confFileName) {
	// boost-parse options
	boost::program_options::options_description desc("Allowed options");
	// DON'T AUTO-INDENT THE FOLLOWING ON ECLIPSE:
	desc.add_options()
		("referenceRobotFile",
				boost::program_options::value<std::string>(&referenceRobotFile),
				"Path to the reference robot file")
		("simulatorConfFile",
				boost::program_options::value<std::string>(&simulatorConfFile)
				->required(), "Path to simulator configuration file")
		("mu",
				boost::program_options::value<unsigned int>(&mu)
				->required(), "Size of population")
		("lambda",
				boost::program_options::value<unsigned int>(&lambda)
				->required(), "Size of offspring")
		("numGenerations",
				boost::program_options::value<unsigned int>(&numGenerations)
				->required(), "Amount of generations to be evaluated")
		("numSelect",
				boost::program_options::value<unsigned int>(&numSelect),
				"Amount of participants in deterministic Tournament")
		("replacement",
				boost::program_options::value<std::string>()->required(),
				"Type of replacement strategy: comma or plus")
		("pBrainMutate", boost::program_options::value<double>
				(&pBrainMutate),"Probability of mutation for any single brain "\
				"parameter")
		("brainSigma", boost::program_options::value<double>(
				&brainSigma), "Sigma of brain parameter mutation")
		("brainBounds", boost::program_options::value<std::string>()
				->required(), "Bounds of brain weights. Format: min:max")
		("pBrainCrossover",
				boost::program_options::value<double>(
				&pBrainCrossover), "Probability of crossover among brains")
		("socket", boost::program_options::value<std::vector<std::string> >()
				->required(),	"Sockets to be used to connect to the server");
	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_config_file<char>(
					confFileName.c_str(), desc, true), vm);
	boost::program_options::notify(vm);

	// parse replacement type
	if (vm["replacement"].as<std::string>() == "comma"){
		replacement = COMMA_REPLACEMENT;
	}
	else if (vm["replacement"].as<std::string>() == "plus"){
		replacement = PLUS_REPLACEMENT;
	}
	else {
		std::stringstream ss;
		ss << "Specified replacement strategy \"" <<
				vm["replacement"].as<std::string>() <<
				"\" unknown. Options are \"comma\" or \"plus\"";
		throw EvolverConfigurationException(ss.str());
	}

	// parse brain bounds.
	static const boost::regex boundsRegex(
			"^(-?\\d*[\\d\\.]\\d*):(-?\\d*[\\d\\.]\\d*)$");
	boost::cmatch match;
	// match[0]:whole string, match[1]:min, match[2]:max
	if (!boost::regex_match(vm["brainBounds"].as<std::string>().c_str(),
			match, boundsRegex)){
		std::stringstream ss;
		ss << "Supplied bounds argument \"" <<
				vm["brainBounds"].as<std::string>() <<
				"\" does not match pattern <min>:<max>";
		throw EvolverConfigurationException(ss.str());
	}
	minBrainWeight = std::atof(match[1].first);
	maxBrainWeight = std::atof(match[2].first);

	// parse sockets. The used regex is not super-restrictive, but we count
	// on the TcpSocket to throw... else:
	// http://www.regular-expressions.info/examples.html
	static const boost::regex socketRegex("^([\\d\\.]*):(\\d*)$");
	std::vector<std::string> encSocket =
			vm["socket"].as<std::vector<std::string> >();
	sockets.clear();
	for (unsigned int i = 0; i<encSocket.size(); i++){
		// match[0]:whole string, match[1]:IP, match[2]:port
		if (!boost::regex_match(encSocket[i].c_str(), match, socketRegex)){
			std::stringstream ss;
			ss << "Supplied socket argument \"" << encSocket[i] <<
					"\" does not match pattern <ip address>:<port>";
			throw EvolverConfigurationException(ss.str());
		}
		sockets.push_back(std::pair<std::string, int>(std::string(match[1]),
				std::atoi(match[2].first)));
	}

	// now that everything is parsed, we verify configuration validity
	// ===================================

	// - if replacement is comma, lambda must exceed mu
	if (replacement == COMMA_REPLACEMENT && lambda < mu){
		std::stringstream ss;
		ss << "If replacement is comma, lambda must be bigger than mu, but "\
				"lambda is " << lambda << " and mu is " << mu;
		throw EvolverConfigurationException(ss.str());
	}

	// - brain bounds max must > min
	if (maxBrainWeight < minBrainWeight){
		std::stringstream ss;
		ss << "Minimum brain bound " << minBrainWeight << " exceeds maximum "
				<< maxBrainWeight;
		throw EvolverConfigurationException(ss.str());
	}

	// - 0. <= probabilities <= 1.
	if (pBrainMutate > 1. || pBrainMutate < 0.){
		std::stringstream ss;
		ss << "Brain mutation probability parameter " << pBrainMutate <<
				" not between 0 and 1!";
		throw EvolverConfigurationException(ss.str());
	}
	if (pBrainCrossover > 1. || pBrainCrossover < 0.){
		std::stringstream ss;
		ss << "Brain crossover probability parameter " << pBrainCrossover <<
				" not between 0 and 1!";
		throw EvolverConfigurationException(ss.str());
	}

	// - sigma needs to be positive
	if (brainSigma < 0.){
		std::stringstream ss;
		ss << "Brain sigma (" << brainSigma << ") must be positive";
		throw EvolverConfigurationException(ss.str());
	}
}

}
