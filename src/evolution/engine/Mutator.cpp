/*
 * @(#) Mutator.cpp   1.0   Sep 2, 2013
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

#include "evolution/engine/Mutator.h"

namespace robogen {

Mutator::Mutator(double pBrainMutate, double brainMuteSigma,
		double pBrainCrossover, boost::random::mt19937 &rng) :
		type_(BRAIN_MUTATOR), weightMutate_(pBrainMutate),
		weightCrossover_(pBrainCrossover),
		weightDistribution_(0.,brainMuteSigma), rng_(rng){
}

Mutator::~Mutator() {
}

void Mutator::mutateCrossover(Population &pop){
	// get individuals TODO not orderedEvaluatedRobots!!!
	std::vector<Individual> &individuals = pop.orderedEvaluatedRobots();

	// individual mutations
	for (unsigned int i=0; i<individuals.size(); ++i){
		mutate(*individuals[i].robot.get());
	}

	// TODO crossover

}

bool Mutator::mutate(RobotRepresentation &robot){
	bool mutated = false;
	// mutate brain TODO conf bits?
	if (type_ == BRAIN_MUTATOR || type_ == BRAIN_BODY_PARAM_MUTATOR ||
			type_ == FULL_MUTATOR){
		std::vector<double*> weights;
		std::vector<double*> biases;
		robot.getBrainGenome(weights,biases);
		// mutate weights
		for (unsigned int i=0; i<weights.size(); ++i){
			if (weightMutate_(rng_)){
				mutated = true;
				*weights[i] += weightDistribution_(rng_);
			}
			// normalize
			if (*weights[i]>1.) *weights[i] = 1.;
			if (*weights[i]<0.) *weights[i] = 0.;
		}
		// mutate biases
		for (unsigned int i=0; i<weights.size(); ++i){
			if (weightMutate_(rng_)){
				mutated = true;
				*weights[i] += weightDistribution_(rng_);
			}
			// normalize
			if (*weights[i]>1.) *weights[i] = 1.;
			if (*weights[i]<-1.) *weights[i] = -1.;
		}

	}
	return mutated;
}

} /* namespace robogen */
