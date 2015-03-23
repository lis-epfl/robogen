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
#include "evolution/neat/Parameters.h"

namespace robogen {

/**
 * The choice is made to use a struct as we want to easily write access to new
 * parameters of the configuration class
 */
typedef struct EvolverConfiguration {
	/**
	 * Types of replacement strategies
	 */
	enum ReplacementTypes{
		PLUS_REPLACEMENT, COMMA_REPLACEMENT
	};

	/**
	 * Types of selection strategies
	 */
	enum SelectionTypes{
		DETERMINISTIC_TOURNAMENT
	};

	/**
	 * Modes of evolution
	 */
	enum EvolutionModes {
		BRAIN_EVOLVER, FULL_EVOLVER
	};

	/**
	 * Evolationary Algorithm
	 * TODO add other possibilities (e.g. CMA-ES)
	 */
	enum EvolutionaryAlgorithms {
		BASIC, HYPER_NEAT
	};


	/**
	 * Types of body mutation.
	 * Last entry is used to get the amount of operators
	 */
	enum BodyMutationOperators{
		SUBTREE_REMOVAL, SUBTREE_DUPLICATION, SUBTREE_SWAPPING,
		NODE_INSERTION, NODE_REMOVAL, PARAMETER_MODIFICATION,
		ORIENTATION_CHANGE, SENSOR_SWAP, LINK_CHANGE, ACTIVE_PASSIVE,
		NUM_BODY_OPERATORS
	};

	static const std::string BodyMutationOperatorsProbabilityCodes[];

	/**
	 * File name of this configuration
	 */
	std::string confFileName;

	/**
	 * Parses a configuration from a proper configuration file
	 */
	bool init(std::string configFileName);

	// GENERAL EVOLUTION PARAMS
	// ========================================================================

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
	 * Employed selection strategy
	 */
	int selection;

	/**
	 * Tournament size for tournaments
	 */
	unsigned int tournamentSize;

	/**
	 * Employed replacement strategy
	 */
	int replacement;

	/**
	 * Employed replacement strategy
	 */
	unsigned int evolutionMode;

	/**
	 *  Flag to continue evolving from provided brain instead of re-initialing
	 *  params randomly
	 */
	bool useBrainSeed;

	/**
	 * Sockets to be used to connect to the server
	 */
	std::vector<std::pair<std::string, int> > sockets;

	// BRAIN EVOLUTION PARAMS
	// ========================================================================

	/**
	 * Probability of mutation for any single brain parameter
	 */
	double pBrainMutate;

	/**
	 * Sigma of brain weight mutation
	 */
	double brainWeightSigma;

	/**
	 * Sigma of brain bias mutation
	 */
	double brainBiasSigma;

	/**
	 * Lower bound for brain weights
	 */
	double minBrainWeight;

	/**
	 * Upper bound for brain weights
	 */
	double maxBrainWeight;

	/**
	 * Lower bound for biases
	 */
	double minBrainBias;

	/**
	 * Upper bound for biases
	 */
	double maxBrainBias;

	/**
	 * PARAMS FOR OTHER BRAIN PARAMETERS  --
	 * 	required if using relevant neuron types  TODO -- validate these
	 */

	double brainTauSigma;
	double brainPeriodSigma;
	double brainPhaseOffsetSigma;
	double brainAmplitudeSigma;

	double minBrainTau;
	double maxBrainTau;
	double minBrainPeriod;
	double maxBrainPeriod;

	// defaults to -1, 1
	double minBrainPhaseOffset;
	double maxBrainPhaseOffset;

	// defaults to 0, 1
	double minBrainAmplitude;
	double maxBrainAmplitude;


	/**
	 * Probability of crossover among brains
	 */
	double pBrainCrossover;

	// BODY EVOLUTION PARAMS
	// ========================================================================

	/**
	 * Allowed body part types
	 */
	std::vector<char> allowedBodyPartTypes;

	/**
	 * Probabilities for the various body operators
	 */
	double bodyOperatorProbability[NUM_BODY_OPERATORS];


	/**
	 * Max body mutation attempts per reproduction events:
	 * 	Mutator will give up trying to mutate the body tree after
	 * 	this many failed attempts
	 */
	unsigned int maxBodyMutationAttempts;

	/**
	 * Maximum number of allowed body parts
	 */
	unsigned int maxBodyParts;

	/**
	 * Minimum number of body parts in individuals in the initial population
	 */
	unsigned int minNumInitialParts;

	/**
	 * Maximum number of body parts in individuals in the initial population
	 */
	unsigned int maxNumInitialParts;

	/**
	 * Std dev of body param mutations
	 */
	double bodyParamSigma;

	/**
	 * Evolutionary algorithm
	 */
	unsigned int evolutionaryAlgorithm;

	/**
	 * NEAT/HyperNEAT parameters
	 */
	NEAT::Parameters neatParams;

	/**
	 * File for NEAT/HyperNEAT parameters
	 */
	std::string neatParamsFile;



} EvolverConfiguration;

} /* namespace robogen */
#endif /* EVOLVERCONFIGURATION_H_ */
