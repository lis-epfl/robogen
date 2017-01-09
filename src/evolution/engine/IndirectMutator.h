/*
 * @(#) Mutator.h   1.0   Sep 2, 2013
 *
 * Carlos Malanche (carlos.malancheflores@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2016 Titus Cieslewski, Joshua Auerbach
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

#ifndef INDIRECT_MUTATOR_H
#define INDIRECT_MUTATOR_H  

#include <boost/random/normal_distribution.hpp>
#include <boost/random/bernoulli_distribution.hpp>

#include "evolution/engine/Mutator.h"

#include "config/EvolverConfiguration.h"
#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/BodyVerifier.h"

namespace robogen{

class IndirectMutator: public Mutator{
public:
	/**
	 * Creates a Robogen brain mutator with the specified settings
	 * @param pBrainMutate probability for a weight or bias to mutate
	 * @param brainMuteSigma sigma of normal distribution for brain mutation
	 * @param pBrainCrossover probability for crossover among brains
	 */
	IndirectMutator(boost::shared_ptr<EvolverConfiguration> conf,
			boost::random::mt19937 &rng);

	virtual ~IndirectMutator();

	/**
	 * Performs mutation and crossover on a pair of robots
	 */
	std::vector<boost::shared_ptr<RobotRepresentation> > createOffspring(
			boost::shared_ptr<RobotRepresentation> parent1,
			boost::shared_ptr<RobotRepresentation> parent2 =
					boost::shared_ptr<RobotRepresentation>());

	void growBodyRandomly(boost::shared_ptr<RobotRepresentation>& robot);
	void randomizeBrain(boost::shared_ptr<RobotRepresentation>& robot);

private:
	/**
	 * Evolver Configuration
	 */
	boost::shared_ptr<EvolverConfiguration> conf_;

	/**
	 * Random number generator
	 */
	boost::random::mt19937 &rng_;

	/**
	 * Operators to create a predecessor
	 */
	boost::shared_ptr<SubRobotRepresentation> generateRandomPredecessor();
	bool insertNode(boost::shared_ptr<SubRobotRepresentation>& robot, bool isAxiom);

	/**
	 * Master Mutator
	 */
	bool mutate(boost::shared_ptr<RobotRepresentation>& robot);

	bool mutateBrain(boost::shared_ptr<RobotRepresentation>& robot);
	bool mutateBody(boost::shared_ptr<RobotRepresentation>& robot);

	/**
	 * Mutation for the Axiom and the rules 
	 */
	
	bool createRule(boost::shared_ptr<RobotRepresentation> &robot);
	bool swapRules(boost::shared_ptr<RobotRepresentation> &robot);
	bool suppressRule(boost::shared_ptr<RobotRepresentation> &robot);
	bool mutateRule(boost::shared_ptr<RobotRepresentation> &robot);

	bool mutateAxiom(boost::shared_ptr<RobotRepresentation> &robot);

	bool insertNode(boost::shared_ptr<RobotRepresentation>& robot);
	bool removeNode(boost::shared_ptr<RobotRepresentation>& robot);

	/**
	 * Probability distribution for calling mutation operators
	 */
	boost::random::bernoulli_distribution<double> suppressRuleDist_;
	boost::random::bernoulli_distribution<double> createRuleDist_;
	boost::random::bernoulli_distribution<double> swapRulesDist_;
	boost::random::bernoulli_distribution<double> mutateRuleDist_;
	boost::random::bernoulli_distribution<double> mutateAxiomDist_;

	boost::random::bernoulli_distribution<double> oscillatorNeuronDist_;
	boost::random::bernoulli_distribution<double> addHiddenNeuronDist_;

	boost::random::bernoulli_distribution<double> brainMutate_;
	boost::random::normal_distribution<double> normalDistribution_;
};

}

#endif //INDIRECT_MUTATOR_H