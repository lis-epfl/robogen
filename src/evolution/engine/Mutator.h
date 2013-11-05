/*
 * @(#) Mutator.h   1.0   Sep 2, 2013
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

#ifndef MUTATOR_H_
#define MUTATOR_H_

// enable the following to perform body mutation:
// #define BODY_MUTATION

#include <boost/random/normal_distribution.hpp>
#include <boost/random/bernoulli_distribution.hpp>
#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/BodyVerifier.h"
#include "evolution/engine/EvolverConfiguration.h"

namespace robogen {

class Mutator {
public:
	/**
	 * Types of mutators
	 */
	enum types{
		BRAIN_MUTATOR,
		BRAIN_BODY_PARAM_MUTATOR, 	// just putting the idea out there
		FULL_MUTATOR 				// the holy grail of the robogen developer
	};

	/**
	 * Creates a Robogen brain mutator with the specified settings
	 * @param pBrainMutate probability for a weight or bias to mutate
	 * @param brainMuteSigma sigma of normal distribution for brain mutation
	 * @param pBrainCrossover probability for crossover among brains
	 */
	Mutator::Mutator(EvolverConfiguration &conf, boost::random::mt19937 &rng);

	virtual ~Mutator();

	/**
	 * Performs mutation and crossover on a pair of robots
	 */
	RobotRepresentation mutate(std::pair<RobotRepresentation,
			RobotRepresentation> parents);

	/**
	 * Mutates a single robot
	 * @return true if robot has been modified
	 * @todo specify bounds in the Neural Network, not here, e.g. with
	 * a MutableDouble class
	 */
	bool mutate(RobotRepresentation &robot);

	/**
	 * Performs crossover between two robots
	 * @return true if some crossover has been performed
	 * @todo enable asymmetric crossover
	 */
	bool crossover(RobotRepresentation &a, RobotRepresentation &b);

private:
	/**
	 * Type of mutator behavior from types enum
	 */
	int type_;

	/**
	 * Diverse distributions to be used for mutation
	 */
	boost::random::bernoulli_distribution<double> weightMutate_;
	boost::random::normal_distribution<double> weightDistribution_;
	boost::random::bernoulli_distribution<double> weightCrossover_;
	double brainMin_;
	double brainMax_;

	boost::random::bernoulli_distribution<double> subtreeRemoval_;
	boost::random::bernoulli_distribution<double> subtreeDuplication_;
	boost::random::bernoulli_distribution<double> subtreeSwap_;
	boost::random::bernoulli_distribution<double> nodeInsert_;
	boost::random::bernoulli_distribution<double> nodeRemoval_;
	boost::random::bernoulli_distribution<double> paramMutate_;

	/**
	 * Random number generator
	 */
	boost::random::mt19937 &rng_;


	boost::shared_ptr<RobotRepresentation> removeSubtree( boost::shared_ptr<RobotRepresentation> robot );
	boost::shared_ptr<RobotRepresentation> duplicateSubtree( boost::shared_ptr<RobotRepresentation> robot );
	boost::shared_ptr<RobotRepresentation> swapSubtrees( boost::shared_ptr<RobotRepresentation> robot );
	boost::shared_ptr<RobotRepresentation> insertNode( boost::shared_ptr<RobotRepresentation> robot );
	boost::shared_ptr<RobotRepresentation> removeNode( boost::shared_ptr<RobotRepresentation> robot );
	boost::shared_ptr<RobotRepresentation> mutateParams( boost::shared_ptr<RobotRepresentation> robot );

};

} /* namespace robogen */
#endif /* MUTATOR_H_ */
