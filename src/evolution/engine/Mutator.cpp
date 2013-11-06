/*
 * @(#) Mutator.cpp   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Andrea Maesani (andrea.maesani@epfl.ch)
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

#include <boost/random/uniform_int_distribution.hpp>
#include "evolution/engine/Mutator.h"
#include "PartList.h"

namespace robogen {

Mutator::Mutator(boost::shared_ptr<EvolverConfiguration> conf,
		boost::random::mt19937 &rng) :
		conf_(conf),
		rng_(rng),
		weightMutate_(conf->pBrainMutate),
		weightDistribution_(0., conf->brainSigma),
		weightCrossover_(conf->pBrainCrossover),
		brainMin_(conf->minBrainWeight),
		brainMax_(conf->maxBrainWeight) {
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
	}

}

Mutator::~Mutator() {
}

boost::shared_ptr<RobotRepresentation> Mutator::mutate(
		boost::shared_ptr<RobotRepresentation> parent1,
		boost::shared_ptr<RobotRepresentation> parent2) {

	boost::shared_ptr<RobotRepresentation> offspring1 = boost::shared_ptr<
			RobotRepresentation>(new RobotRepresentation(*parent1.get()));
	boost::shared_ptr<RobotRepresentation> offspring2 = boost::shared_ptr<
			RobotRepresentation>(new RobotRepresentation(*parent2.get()));

	// only allow crossover if doing just brain mutation
	if (conf_->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER) {
		this->crossover(offspring1, offspring2);
	}

	// Mutate
	this->mutate(offspring1);

	return offspring1;
}

bool Mutator::growBodyRandomly(boost::shared_ptr<RobotRepresentation> robot) {

	boost::random::uniform_int_distribution<> dist(conf_->minNumInitialParts,
			conf_->minNumInitialParts);
	unsigned int numPartsToAdd = dist(rng_);

	bool success = false;
	for (unsigned int attempt = 0;
			(!success) && (attempt < conf_->maxBodyMutationAttemps);
			++attempt) {
		for (unsigned int i = 0; i < numPartsToAdd; i++) {
			this->insertNode(robot);
		}
		int errorCode;
		std::vector<std::pair<std::string, std::string> > affectedBodyParts;
		success = BodyVerifier::verify(*robot.get(), errorCode,
				affectedBodyParts);
	}

	return success;
}

bool Mutator::mutate(boost::shared_ptr<RobotRepresentation>& robot) {

	bool mutated = false;

	// mutate brain TODO conf bits?
	if (conf_->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER
			|| conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		std::vector<double*> weights;
		std::vector<double*> biases;
		robot->getBrainGenome(weights, biases);

		// mutate weights
		for (unsigned int i = 0; i < weights.size(); ++i) {
			if (weightMutate_(rng_)) {
				mutated = true;
				*weights[i] += weightDistribution_(rng_);
			}
			// normalize
			if (*weights[i] > brainMax_)
				*weights[i] = brainMax_;
			if (*weights[i] < brainMin_)
				*weights[i] = brainMin_;
		}
		// mutate biases
		for (unsigned int i = 0; i < biases.size(); ++i) {
			if (weightMutate_(rng_)) {
				mutated = true;
				*biases[i] += weightDistribution_(rng_);
			}
			// normalize
			if (*biases[i] > brainMax_)
				*biases[i] = brainMax_;
			if (*biases[i] < brainMin_)
				*biases[i] = brainMin_;
		}
		if (mutated) {
			robot->setDirty();
		}
	}

	if (conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		this->mutateBody(robot);
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
	std::vector<double*> biases[2];
	a->getBrainGenome(weights[0], biases[0]);
	b->getBrainGenome(weights[1], biases[1]);

	// 2. select crossover point
	unsigned int maxpoint = weights[0].size() + biases[0].size() - 1;
	if (maxpoint != weights[1].size() + biases[1].size() - 1) {
		//TODO error handling, TODO what if sum same, but not parts?
		std::cout << "Genomes not of same size!" << std::endl;
	}
	boost::random::uniform_int_distribution<unsigned int> pointSel(1, maxpoint);
	int selectedPoint = pointSel(rng_);

	// 3. perform crossover
	for (unsigned int i = selectedPoint; i <= maxpoint; i++) {
		if (i < weights[0].size()) {
			std::swap(*weights[0][i], *weights[1][i]);
		} else {
			int j = i - weights[0].size();
			std::swap(*biases[0][j], *biases[1][j]);
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

void Mutator::mutateBody(boost::shared_ptr<RobotRepresentation>& robot) {

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
					attempt < conf_->maxBodyMutationAttemps; ++attempt) {

				boost::shared_ptr<RobotRepresentation> newBot =
						boost::shared_ptr<RobotRepresentation>(
								new RobotRepresentation(*robot.get()));

				bool mutationSuccess = (this->*mutOp)(newBot);

				int errorCode;
				std::vector<std::pair<std::string, std::string> > affectedBodyParts;
				if (mutationSuccess
						&& BodyVerifier::verify(*newBot.get(), errorCode,
								affectedBodyParts)) {
					robot = newBot;
					robot->setDirty();
					break;
				}
			}
		}
	}
}

bool Mutator::removeSubtree(boost::shared_ptr<RobotRepresentation>& robot) {

	// Get a random body node
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator subtreeRootPart =
			idPartMap.begin();
	std::advance(subtreeRootPart, dist(rng_));

	// Trim the body tree at the selected random body node
	robot->trimBodyAt(subtreeRootPart->first);

	return true;
}

bool Mutator::duplicateSubtree(boost::shared_ptr<RobotRepresentation>& robot) {

	// Get a random root of the tree to duplicate
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	RobotRepresentation::IdPartMap::const_iterator subtreeRootPart =
			idPartMap.begin();
	std::advance(subtreeRootPart, dist(rng_));

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
		unsigned int selectedSlotId = freeSlotsDist(rng_);

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
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	RobotRepresentation::IdPartMap::const_iterator parent = idPartMap.begin();
	std::advance(parent, dist(rng_));

	boost::shared_ptr<PartRepresentation> parentPart = parent->second.lock();

	// Sample a random slot
	boost::random::uniform_int_distribution<> distSlot(0,
			parentPart->getArity() - 1);
	unsigned int parentSlot = distSlot(rng_);

	// Select node type
	boost::random::uniform_int_distribution<> distType(0,
			conf_->allowedBodyPartTypes.size() - 1);
	char type = conf_->allowedBodyPartTypes[distType(rng_)];

	// Randomly generate node orientation
	boost::random::uniform_int_distribution<> orientationDist(0, 3);
	unsigned int curOrientation = orientationDist(rng_);

	// Randomly generate parameters
	unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP[PART_TYPE_MAP[type]];
	std::vector<double> parameters;
	boost::random::uniform_01<double> paramDist;
	for (unsigned int i = 0; i < nParams; ++i) {
		parameters.push_back(paramDist(rng_));
	}

	// Create the new part
	boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
			type, "", curOrientation, parameters);

	// Generate a random slot in the new node
	boost::random::uniform_int_distribution<> distNewPartSlot(0,
			newPart->getArity() - 1);

	return robot->insertPart(parent->first, parentSlot, newPart,
			distNewPartSlot(rng_));

}

bool Mutator::removeNode(boost::shared_ptr<RobotRepresentation>& robot) {

	// Select node for removal
	const RobotRepresentation::IdPartMap& idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator partToRemove =
			idPartMap.begin();
	std::advance(partToRemove, dist(rng_));

	return robot->removePart(partToRemove->first);

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
		params[paramToMutate] += paramDistribution_(rng_);
		if (params[paramToMutate] < 0) {
			params[paramToMutate] = 0;
		} else if (params[paramToMutate] > 1) {
			params[paramToMutate] = 1;
		}
	}

	return true;

}

}
