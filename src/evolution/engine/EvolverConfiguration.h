/*
 * @(#) EvolverConfiguration.h 1.0   Sep 2, 2013
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

#ifndef EVOLVERCONFIGURATION_H_
#define EVOLVERCONFIGURATION_H_

#include <string>
#include <vector>
#include <utility>
#include <stdexcept>

namespace robogen {

class EvolverConfigurationException : public std::runtime_error {
public:
	EvolverConfigurationException(const std::string& w);
};

/**
 * The choice is made to use a struct as we want to easily write access to new
 * parameters of the configuration class
 */
struct EvolverConfiguration {
	/**
	 * Types of replacement strategies
	 */
	enum ReplacementTypes{
		PLUS_REPLACEMENT,
		COMMA_REPLACEMENT
	};

	/**
	 * Parses a configuration from a proper configuration file
	 */
	void init(std::string confFileName);

	/**
	 * File for reference robot
	 */
	std::string referenceRobotFile;

	/**
	 * Configuration file for simulator
	 */
	std::string simulatorConfFile;

	/**
	 * Population size
	 */
	unsigned int mu;

	/**
	 * Offspring size
	 */
	unsigned int lambda;

	/**
	 * Amount of generations to be simulated
	 */
	unsigned int numGenerations;

	/**
	 * Amount of robots to be selected
	 */
	unsigned int numSelect;

	/**
	 * Employed replacement strategy
	 */
	unsigned int replacement;

	/**
	 * Probability of mutation for any single brain parameter
	 */
	double pBrainMutate;

	/**
	 * Sigma of brain parameter mutation
	 */
	double brainSigma;

	/**
	 * Lower bound for brain weights and biases
	 * @todo treat weights and biases separately
	 */
	double minBrainWeight;

	/**
	 * Upper bound for brain weights and biases
	 * @todo treat weights and biases separately
	 */
	double maxBrainWeight;

	/**
	 * Probability of crossover among brains
	 */
	double pBrainCrossover;

	/**
	 * Sockets to be used to connect to the server
	 */
	std::vector<std::pair<std::string, int> > sockets;
};

} /* namespace robogen */
#endif /* EVOLVERCONFIGURATION_H_ */
