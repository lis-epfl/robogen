/*
 * @(#) Mutator.cpp   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2016 Titus Cieslewski, Andrea Maesani, Joshua Auerbach
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

#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include "evolution/engine/Mutator.h"
#include "PartList.h"

//#define DEBUG_MUTATE

namespace robogen {

Mutator::Mutator(boost::shared_ptr<EvolverConfiguration> conf,
		boost::random::mt19937 &rng) :
		conf_(conf), rng_(rng), brainMutate_(conf->pBrainMutate),
		weightCrossover_(conf->pBrainCrossover) {

	if (conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		subtreeRemovalDist_ =
				boost::random::bernoulli_distribution<double>(
						conf->bodyOperatorProbability
						[EvolverConfiguration::SUBTREE_REMOVAL]);
		subtreeDuplicationDist_ =
				boost::random::bernoulli_distribution<double>(
						conf->bodyOperatorProbability
						[EvolverConfiguration::SUBTREE_DUPLICATION]);
		subtreeSwapDist_ =
				boost::random::bernoulli_distribution<double>(
						conf->bodyOperatorProbability
						[EvolverConfiguration::SUBTREE_SWAPPING]);
		nodeInsertDist_ =
				boost::random::bernoulli_distribution<double>(
						conf->bodyOperatorProbability
						[EvolverConfiguration::NODE_INSERTION]);
		nodeRemovalDist_ =
				boost::random::bernoulli_distribution<double>(
						conf->bodyOperatorProbability
						[EvolverConfiguration::NODE_REMOVAL]);
		paramMutateDist_ =
				boost::random::bernoulli_distribution<double>(
						conf->bodyOperatorProbability
						[EvolverConfiguration::PARAMETER_MODIFICATION]);

		addHiddenNeuronDist_ = boost::random::bernoulli_distribution<double>(
				conf->pAddHiddenNeuron);

		oscillatorNeuronDist_ = boost::random::bernoulli_distribution<double>(
				conf->pOscillatorNeuron);
	}

}

Mutator::~Mutator() {
}

std::vector<boost::shared_ptr<RobotRepresentation> > Mutator::createOffspring(
			boost::shared_ptr<RobotRepresentation> parent1,
			boost::shared_ptr<RobotRepresentation> parent2) {

	std::vector<boost::shared_ptr<RobotRepresentation> > offspring;

	offspring.push_back(boost::shared_ptr<RobotRepresentation>(new
			RobotRepresentation(*parent1.get())));


	// only allow crossover if doing just brain mutation
	if (conf_->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER
			&& parent2) {
		offspring.push_back(boost::shared_ptr<RobotRepresentation>(new
				RobotRepresentation(*parent2.get())));
		this->crossover(offspring[0], offspring[1]);
	}

	// Mutate
	for(size_t i = 0; i < offspring.size(); ++i) {
		this->mutate(offspring[i]);
	}

	return offspring;
}

void Mutator::growBodyRandomly(boost::shared_ptr<RobotRepresentation>& robot) {

	boost::random::uniform_int_distribution<> dist(conf_->minNumInitialParts,
			conf_->maxNumInitialParts);
	unsigned int numPartsToAdd = dist(rng_);

	for (unsigned int i = 0; i < numPartsToAdd; i++) {
		bool success = false;

		for (unsigned int attempt = 0;
				(attempt < conf_->maxBodyMutationAttempts); ++attempt) {

			boost::shared_ptr<RobotRepresentation> newBot = boost::shared_ptr<
					RobotRepresentation>(new RobotRepresentation(*robot.get()));
			success = this->insertNode(newBot);
			int errorCode;

			std::vector<std::pair<std::string, std::string> > affectedBodyParts;
			if (success
					&& BodyVerifier::verify(*newBot.get(), errorCode,
							affectedBodyParts)) {
				robot = newBot;
				robot->setDirty();
				break;
			}
		}
	}
}

void Mutator::randomizeBrain(boost::shared_ptr<RobotRepresentation>& robot) {

	// randomize weights and biases randomly in valid range
	boost::random::uniform_01<double> distrib;

	std::vector<double*> weights;
	std::vector<double*> params;
	std::vector<unsigned int> types;
	robot->getBrainGenome(weights, types, params);

	// set weights
	for (unsigned int i = 0; i < weights.size(); ++i) {
		*weights[i] = distrib(rng_) * (conf_->maxBrainWeight -
				conf_->minBrainWeight) + conf_->minBrainWeight;
	}
	// set params (biases, etc)
	unsigned int paramCounter = 0;
	for (unsigned int i = 0; i < types.size(); ++i) {
		if(types[i] == NeuronRepresentation::SIGMOID) {
			*params[paramCounter] = distrib(rng_) * (conf_->maxBrainBias -
					conf_->minBrainBias) + conf_->minBrainBias;
			paramCounter+=1;
		} else if(types[i] == NeuronRepresentation::CTRNN_SIGMOID) {
			*params[paramCounter] = distrib(rng_) * (conf_->maxBrainBias -
					conf_->minBrainBias) + conf_->minBrainBias;
			*params[paramCounter + 1] = distrib(rng_) * (conf_->maxBrainTau -
					conf_->minBrainTau) + conf_->minBrainTau;
			paramCounter += 2;
		} else if(types[i] == NeuronRepresentation::OSCILLATOR) {
			*params[paramCounter] = distrib(rng_) * (conf_->maxBrainPeriod -
					conf_->minBrainPeriod) + conf_->minBrainPeriod;
			*params[paramCounter + 1] = distrib(rng_) * (conf_->maxBrainPhaseOffset -
					conf_->minBrainPhaseOffset) + conf_->minBrainPhaseOffset;
			*params[paramCounter + 2] = distrib(rng_) * (conf_->maxBrainAmplitude -
					conf_->minBrainAmplitude) + conf_->minBrainAmplitude;
			paramCounter += 3;
		} else {
			std::cout << "INVALID TYPE ENCOUNTERED " << types[i] << std::endl;
		}

	}
	robot->setDirty();


}

bool Mutator::mutate(boost::shared_ptr<RobotRepresentation>& robot) {

	bool mutated = false;

	// mutate brain TODO conf bits?
	if (conf_->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER
			|| conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		mutated = (this->mutateBrain(robot) || mutated);
	}

	if (conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		mutated = (this->mutateBody(robot) || mutated);
	}

	return mutated;
}

//helper function for mutations

double clip(double value, double min, double max) {
	if ( value < min )
		return min;
	if (value > max )
		return max;
	return value;
}

bool Mutator::mutateBrain(boost::shared_ptr<RobotRepresentation>& robot) {
	bool mutated = false;

	// first potentially add hidden neurons
	if ((robot->getBrain()->getNumHidden() < MAX_HIDDEN_NEURONS) &&
			addHiddenNeuronDist_(rng_)) {
		unsigned int neuronType = NeuronRepresentation::SIGMOID;
		if (oscillatorNeuronDist_(rng_)) {
			neuronType = NeuronRepresentation::OSCILLATOR;
		}
		std::string neuronId = robot->getBrain()->insertNeuron(
				ioPair(robot->getBodyRootId(),
										robot->getBrain()->getBodyPartNeurons(
												robot->getBodyRootId()).size()),
							NeuronRepresentation::HIDDEN, neuronType);
		mutated = true;
	}

	// TODO allow removing hidden neurons???


	std::vector<double*> weights;
	std::vector<double*> params;
	std::vector<unsigned int> types;
	robot->getBrainGenome(weights, types, params);

	// mutate weights
	for (unsigned int i = 0; i < weights.size(); ++i) {
		if (brainMutate_(rng_)) {
			mutated = true;
			*weights[i] += (normalDistribution_(rng_) *
					conf_->brainWeightSigma);
			*weights[i] = clip(*weights[i], conf_->minBrainWeight,
					conf_->maxBrainWeight);

		}
	}
	// mutate params (biases, etc)
	unsigned int paramCounter = 0;
	for (unsigned int i = 0; i < types.size(); ++i) {
		if(types[i] == NeuronRepresentation::SIGMOID) {
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter] += (normalDistribution_(rng_) *
						conf_->brainBiasSigma);
				*params[paramCounter] = clip(*params[paramCounter],
						conf_->minBrainBias, conf_->maxBrainBias);
			}
			paramCounter+=1;
		} else if(types[i] == NeuronRepresentation::CTRNN_SIGMOID) {
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter] += (normalDistribution_(rng_) *
						conf_->brainBiasSigma);
				*params[paramCounter] = clip(*params[paramCounter],
						conf_->minBrainBias, conf_->maxBrainBias);
			}
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter+1] += (normalDistribution_(rng_) *
						conf_->brainTauSigma);
				*params[paramCounter+1] = clip(*params[paramCounter+1],
						conf_->minBrainTau, conf_->maxBrainTau);
			}
			paramCounter += 2;
		} else if(types[i] == NeuronRepresentation::OSCILLATOR) {
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter] += (normalDistribution_(rng_) *
						conf_->brainPeriodSigma);
				*params[paramCounter] = clip(*params[paramCounter],
						conf_->minBrainPeriod, conf_->maxBrainPeriod);
			}
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter+1] += (normalDistribution_(rng_) *
						conf_->brainPhaseOffsetSigma);
				*params[paramCounter+1] = clip(*params[paramCounter+1],
						conf_->minBrainPhaseOffset, conf_->maxBrainPhaseOffset);
			}
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter+2] += (normalDistribution_(rng_) *
						conf_->brainAmplitudeSigma);
				*params[paramCounter+2] = clip(*params[paramCounter+2],
						conf_->minBrainAmplitude, conf_->maxBrainAmplitude);
			}
			paramCounter += 3;
		} else {
			std::cout << "INVALID TYPE ENCOUNTERED " << types[i] << std::endl;
		}

	}

	if (mutated) {
		robot->setDirty();
	}
	return mutated;
}

bool Mutator::crossover(boost::shared_ptr<RobotRepresentation>& a,
		boost::shared_ptr<RobotRepresentation>& b) {

	if (!weightCrossover_(rng_)) {
		return false;
	}

	// 1. get genomes
	std::vector<double*> weights[2];
	std::vector<double*> params[2];
	std::vector<unsigned int> types[2];
	a->getBrainGenome(weights[0], types[0], params[0]);
	b->getBrainGenome(weights[1], types[1], params[1]);

	// 2. select crossover point
	unsigned int genomeSizeA = weights[0].size() + params[0].size();
	unsigned int genomeSizeB = weights[1].size() + params[1].size();
	if (genomeSizeA != genomeSizeB) {
		//TODO error handling, TODO what if sum same, but not parts?
		std::cout << "Genomes not of same size! " << genomeSizeA << " " <<
				genomeSizeB << std::endl;

		std::cout << a->getBrain()->toString() << std::endl << std::endl;
		std::cout << b->getBrain()->toString() << std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	if (genomeSizeA < 2) {
		//nothing to crossover
		return false;
	}

	unsigned int maxpoint = genomeSizeA - 1;


	boost::random::uniform_int_distribution<unsigned int> pointSel(1, maxpoint);
	int selectedPoint = pointSel(rng_);

	// 3. perform crossover
	for (unsigned int i = selectedPoint; i <= maxpoint; i++) {
		if (i < weights[0].size()) {
			std::swap(*weights[0][i], *weights[1][i]);
		} else {
			int j = i - weights[0].size();
			std::swap(*params[0][j], *params[1][j]);
		}
	}

	a->setDirty();
	b->setDirty();
	return true;
}

typedef bool (Mutator::*MutationOperator)(
		boost::shared_ptr<RobotRepresentation>&);

typedef std::pair<MutationOperator,
		boost::random::bernoulli_distribution<double> > MutOpPair;

bool Mutator::mutateBody(boost::shared_ptr<RobotRepresentation>& robot) {
#ifdef DEBUG_MUTATE
	std::cout << "mutating body" << std::endl;
#endif
	bool mutated = false;
	MutOpPair mutOpPairs[] = { std::make_pair(&Mutator::removeSubtree,
			subtreeRemovalDist_), std::make_pair(&Mutator::duplicateSubtree,
			subtreeDuplicationDist_), std::make_pair(&Mutator::swapSubtrees,
			subtreeSwapDist_), std::make_pair(&Mutator::insertNode,
			nodeInsertDist_), std::make_pair(&Mutator::removeNode,
			nodeRemovalDist_), std::make_pair(&Mutator::mutateParams,
			paramMutateDist_) };

	int numOperators = sizeof(mutOpPairs) / sizeof(MutOpPair);
	for (int i = 0; i < numOperators; ++i) {

		MutationOperator mutOp = mutOpPairs[i].first;
		boost::random::bernoulli_distribution<double> dist =
				mutOpPairs[i].second;

		if (dist(rng_)) {

			for (unsigned int attempt = 0;
					attempt < conf_->maxBodyMutationAttempts; ++attempt) {

#ifdef DEBUG_MUTATE
				std::cout << "Robot will be mutated using mutation " << i << std::endl;
							std::cout << "OldBot: " << std::endl;
							std::cout << robot->toString() << std::endl;
#endif

				boost::shared_ptr<RobotRepresentation> newBot =
						boost::shared_ptr<RobotRepresentation>(
								new RobotRepresentation(*robot.get()));

				bool mutationSuccess = (this->*mutOp)(newBot);

#ifdef DEBUG_MUTATE
				std::cout << "mutationSuccess " << mutationSuccess << std::endl;
				std::cout << "NewBot: " << std::endl;
				std::cout << newBot->toString() << std::endl;
#endif

				int errorCode;
				std::vector<std::pair<std::string, std::string> > affectedBodyParts;
				if (mutationSuccess
						&& BodyVerifier::verify(*newBot.get(), errorCode,
								affectedBodyParts)) {

					if (!newBot->check()) {
						std::cout << "Consistency check failed in mutation operator " << i << std::endl;
					}

					robot = newBot;
					robot->setDirty();
					mutated = true;
					break;

				}

			}
		}
	}
#ifdef DEBUG_MUTATE
	std::cout << "body mutated " << mutated << std::endl;
#endif
	return mutated;
}

bool Mutator::removeSubtree(boost::shared_ptr<RobotRepresentation>& robot) {

	// Get a random body node
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator subtreeRootPart =
			idPartMap.begin();
	std::advance(subtreeRootPart, dist(rng_));

	// Trim the body tree at the selected random body node
	bool success = robot->trimBodyAt(subtreeRootPart->first);

#ifdef DEBUG_MUTATE
	std::cout << "Removing subtree at" << subtreeRootPart->first  << " " << success << std::endl;
#endif

	return success;
}

bool Mutator::duplicateSubtree(boost::shared_ptr<RobotRepresentation>& robot) {

	// Get a random root of the tree to duplicate
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	unsigned int totalNodes = idPartMap.size();

	RobotRepresentation::IdPartMap::const_iterator subtreeRootPart =
			idPartMap.begin();
	std::advance(subtreeRootPart, dist(rng_));

	boost::shared_ptr<PartRepresentation> srcPart =
			subtreeRootPart->second.lock();

	unsigned int subTreeSize = 1 + srcPart->numDescendants();

	if ( totalNodes + subTreeSize >  conf_->maxBodyParts )
		return false;

	// Select another node
	RobotRepresentation::IdPartMap::const_iterator subtreeDestPart =
			idPartMap.begin();
	std::advance(subtreeDestPart, dist(rng_));

	// Check if the destination node has free slots
	boost::shared_ptr<PartRepresentation> destPart =
			subtreeDestPart->second.lock();
	std::vector<unsigned int> freeSlots = destPart->getFreeSlots();
	if (freeSlots.size() > 0) {

		boost::random::uniform_int_distribution<> freeSlotsDist(0,
				freeSlots.size() - 1);
		unsigned int selectedSlotId = freeSlots[freeSlotsDist(rng_)];

#ifdef DEBUG_MUTATE
		std::cout << "duplicating subtree rooted at " << subtreeRootPart->first
				<< " to " << subtreeDestPart->first << " at slot " <<
				selectedSlotId << std::endl;
#endif

		return robot->duplicateSubTree(subtreeRootPart->first,
				subtreeDestPart->first, selectedSlotId);

	} else {

		return false;

	}

	return true;

}

bool Mutator::swapSubtrees(boost::shared_ptr<RobotRepresentation>& robot) {

	// Get a random root of the tree to duplicate
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	RobotRepresentation::IdPartMap::const_iterator rootPartIt1 =
			idPartMap.begin();
	std::advance(rootPartIt1, dist(rng_));

	boost::shared_ptr<PartRepresentation> rootPart1 =
			rootPartIt1->second.lock();

	// Get IDs of nodes to be considered for selection of second root
	std::vector<std::string> ancestorIDs = rootPart1->getAncestorsIds();
	std::vector<std::string> descendantIds = rootPart1->getDescendantsIds();

	std::vector<std::string> invalidIds;
	invalidIds.push_back(rootPart1->getId());
	invalidIds.insert(invalidIds.end(), ancestorIDs.begin(), ancestorIDs.end());
	invalidIds.insert(invalidIds.end(), descendantIds.begin(),
			descendantIds.end());

	// Retrieve all the body part ids
	std::vector<std::string> robotPartIds;
	for (RobotRepresentation::IdPartMap::const_iterator it = idPartMap.begin();
			it != idPartMap.end(); ++it) {
		robotPartIds.push_back(it->first);
	}

	// Sort IDS
	std::sort(invalidIds.begin(), invalidIds.end());
	std::sort(robotPartIds.begin(), robotPartIds.end());

	std::vector<std::string> validIds(invalidIds.size() + robotPartIds.size());
	std::vector<std::string>::iterator resIt = std::set_difference(
			robotPartIds.begin(), robotPartIds.end(), invalidIds.begin(),
			invalidIds.end(), validIds.begin());
	validIds.resize(resIt - validIds.begin());

	// No more valid ids?
	if (validIds.size() == 0) {
		return false;
	}

	// Select another node
	boost::random::uniform_int_distribution<> distRootPart2(0,
			validIds.size() - 1);
	std::string rootPartId2 = validIds[distRootPart2(rng_)];
	boost::shared_ptr<PartRepresentation> rootPart2 =
			idPartMap.at(rootPartId2).lock();

	return robot->swapSubTrees(rootPart1->getId(), rootPart2->getId());

}

bool Mutator::insertNode(boost::shared_ptr<RobotRepresentation>& robot) {

	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();

	if ( idPartMap.size() >= conf_->maxBodyParts )
		return false;

	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	RobotRepresentation::IdPartMap::const_iterator parent = idPartMap.begin();
	std::advance(parent, dist(rng_));

	boost::shared_ptr<PartRepresentation> parentPart = parent->second.lock();

	std::vector<unsigned int> freeSlots = parentPart->getFreeSlots();

	// Sample a random slot
	if (freeSlots.size() == 0) {
		return false;
	}

	boost::random::uniform_int_distribution<> freeSlotsDist(0,
					freeSlots.size() - 1);
	unsigned int parentSlot = freeSlots[freeSlotsDist(rng_)];


	// Select node type
	boost::random::uniform_int_distribution<> distType(0,
			conf_->allowedBodyPartTypes.size() - 1);
	char type = conf_->allowedBodyPartTypes[distType(rng_)];

	// Randomly generate node orientation
	boost::random::uniform_int_distribution<> orientationDist(0, 3);
	unsigned int curOrientation = orientationDist(rng_);

	// Randomly generate parameters
	unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type));
	std::vector<double> parameters;
	boost::random::uniform_01<double> paramDist;
	for (unsigned int i = 0; i < nParams; ++i) {
		parameters.push_back(paramDist(rng_));
	}

	// Create the new part
	boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
			type, "", curOrientation, parameters);

	unsigned int newPartSlot = 0;

	if (newPart->getArity() > 0) {
		// Generate a random slot in the new node, if it has arity > 0
		boost::random::uniform_int_distribution<> distNewPartSlot(0,
				newPart->getArity() - 1);
		newPartSlot = distNewPartSlot(rng_);
	}
	// otherwise just keep it at 0... inserting part will fail if arity is 0 and
	// there were previously parts attached to the parent's chosen slot

	return robot->insertPart(parent->first, parentSlot, newPart, newPartSlot,
			oscillatorNeuronDist_(rng_) ? NeuronRepresentation::OSCILLATOR :
					NeuronRepresentation::SIGMOID); //todo other types?

}

bool Mutator::removeNode(boost::shared_ptr<RobotRepresentation>& robot) {

	// Select node for removal
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator partToRemove =
			idPartMap.begin();
	std::advance(partToRemove, dist(rng_));

	bool success= robot->removePart(partToRemove->first);
#ifdef DEBUG_MUTATE
	std::cout << "Removed " << partToRemove->first << " " << success << std::endl;
#endif
	return success;

}

bool Mutator::mutateParams(boost::shared_ptr<RobotRepresentation>& robot) {

	// Select node for mutation
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator partToMutate =
			idPartMap.begin();
	std::advance(partToMutate, dist(rng_));

	std::vector<double> params = partToMutate->second.lock()->getParams();
	// Select a random parameter/or orientation to mutate
	boost::random::uniform_int_distribution<> distMutation(0, params.size());
	unsigned int paramToMutate = distMutation(rng_);

	if (paramToMutate == params.size()) { //mutate orientation
		boost::random::uniform_int_distribution<> orientationDist(0, 3);
		unsigned int newOrientation = orientationDist(rng_);
		partToMutate->second.lock()->setOrientation(newOrientation);
	} else {
		params[paramToMutate] += (normalDistribution_(rng_) *
									conf_->bodyParamSigma);
		params[paramToMutate] = clip(params[paramToMutate], 0., 1.);
	}

	return true;

}

}
