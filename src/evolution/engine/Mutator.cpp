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
#include <boost/random/uniform_int_distribution.hpp>

namespace robogen {

Mutator::Mutator(double pBrainMutate, double brainMuteSigma,
		double pBrainCrossover, double brainMin, double brainMax,
		boost::random::mt19937 &rng) :
		type_(BRAIN_MUTATOR), weightMutate_(pBrainMutate),
		weightDistribution_(0.,brainMuteSigma),
		weightCrossover_(pBrainCrossover),
		brainMin_(brainMin), brainMax_(brainMax),
		rng_(rng){
}

Mutator::~Mutator() {
}

RobotRepresentation Mutator::mutate(std::pair<RobotRepresentation,
		RobotRepresentation> parents){
	// TODO copy first!
	this->crossover(parents.first,parents.second);
	this->mutate(parents.first);
	return parents.first;
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
			if (*weights[i]>brainMax_) *weights[i] = brainMax_;
			if (*weights[i]<brainMin_) *weights[i] = brainMin_;
		}
		// mutate biases
		for (unsigned int i=0; i<biases.size(); ++i){
			if (weightMutate_(rng_)){
				mutated = true;
				*biases[i] += weightDistribution_(rng_);
			}
			// normalize
			if (*biases[i]>brainMax_) *biases[i] = brainMax_;
			if (*biases[i]<brainMin_) *biases[i] = brainMin_;
		}
		if (mutated){
			robot.setDirty();
		}
	}

#ifdef BODY_MUTATION
	while (true){
		// check velidity of body
		int errorCode;
		std::vector<std::pair<std::string, std::string> > offenders;
		if (!verifier_.verify(robot, errorCode, offenders)){
			if (errorCode == BodyVerifier::SELF_INTERSECTION){
				std::cout << "Robot body has following intersection pairs:"
						<< std::endl;
				for (unsigned int i=0; i<offenders.size(); ++i){
					// get robots IdPartMap: Volatile, so needs update
					const RobotRepresentation::IdPartMap idPartMap =
							robot.getBody();
					std::cout << offenders[i].first << " with " <<
							offenders[i].second << std::endl;

					// check if offending body part hasn't been removed yet
					if (idPartMap.find(offenders[i].first) == idPartMap.end() ||
							idPartMap.find(offenders[i].second) ==
									idPartMap.end()){
						continue;
					}
					// will remove body part with less descendants (i.e. this
					// covers the case where one part descends from the other)
					int numDesc[] = {idPartMap.find(offenders[i].first)->
							second.lock()->numDescendants(),
							idPartMap.find(offenders[i].second)->
							second.lock()->numDescendants()
					};
					std::cout << offenders[i].first << " has " <<
							numDesc[0] << " descendants" << std::endl;
					std::cout << offenders[i].second << " has " <<
							numDesc[1] << " descendants" << std::endl;
					if (numDesc[0]>numDesc[1]){
						robot.trimBodyAt(offenders[i].second);
						std::cout << "Removing latter" << std::endl;
					}
					else{
						robot.trimBodyAt(offenders[i].first);
						std::cout << "Removing former" << std::endl;
					}
				}
			}
		}
		else{
			break;
		}
	}
#endif


	return mutated;
}

bool Mutator::crossover(RobotRepresentation &a, RobotRepresentation &b){
	if (!weightCrossover_(rng_)) return false;
	// at first, only one-point TODO two-point

	// 1. get genomes
	std::vector<double*> weights[2];
	std::vector<double*> biases[2];
	a.getBrainGenome(weights[0],biases[0]);
	b.getBrainGenome(weights[1],biases[1]);

	// 2. select crossover point
	unsigned int maxpoint = weights[0].size() + biases[0].size() - 1;
	if (maxpoint != weights[1].size() + biases[1].size() - 1){
		//TODO exception, TODO what if sum same, but not parts?
		std::cout << "Genomes not of same size!" << std::endl;
	}
	boost::random::uniform_int_distribution<unsigned int> pointSel(1,maxpoint);
	int selectedPoint = pointSel(rng_);

	// 3. perform crossover
	for (unsigned int i=selectedPoint; i<=maxpoint; i++){
		if (i<weights[0].size()){
			std::swap(*weights[0][i],*weights[1][i]);
		}
		else{
			int j = i-weights[0].size();
			std::swap(*biases[0][j],*biases[1][j]);
		}
	}

	a.setDirty(); b.setDirty();
	return true;
}

} /* namespace robogen */
