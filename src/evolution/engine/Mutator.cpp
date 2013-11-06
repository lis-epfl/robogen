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

#include <boost/random/uniform_int_distribution.hpp>
#include "evolution/engine/Mutator.h"
#include "PartList.h"

namespace robogen {

Mutator::Mutator(boost::shared_ptr<EvolverConfiguration> conf,
		boost::random::mt19937 &rng) :
		conf_(conf), type_(BRAIN_MUTATOR), rng_(rng) {
	/*weightMutate_(conf.pBrainMutate), weightDistribution_(
	 0., conf.brainSigma), weightCrossover_(conf.pBrainCrossover), brainMin_(
	 conf.minBrainWeight), brainMax_(conf.maxBrainWeight),*/

	//TODO move type to conf, init other distributions if needed
}

Mutator::~Mutator() {
}

RobotRepresentation Mutator::mutate(
		std::pair<RobotRepresentation, RobotRepresentation> parents) {
	// TODO copy first!
	//this->crossover(parents.first, parents.second);
	//this->mutate(parents.first);
	boost::shared_ptr<RobotRepresentation> offspring = boost::shared_ptr<
			RobotRepresentation>(new RobotRepresentation(parents.first));

	this->mutateBody(offspring);

	return RobotRepresentation(*offspring.get());
	//return parents.first;
}

bool Mutator::mutate(RobotRepresentation &robot) {
	bool mutated = false;
	// mutate brain TODO conf bits?
	if (type_ == BRAIN_MUTATOR || type_ == BRAIN_BODY_PARAM_MUTATOR
			|| type_ == FULL_MUTATOR) {
		std::vector<double*> weights;
		std::vector<double*> biases;
		robot.getBrainGenome(weights, biases);
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
			robot.setDirty();
		}
	}

#ifdef BODY_MUTATION
	// let's work with hard coded mutation probability as long as this is
	// experimental. Later, make it an option of the Mutator or even create
	// a derived mutator, much like the selector is implemented.
	// 1. Hard mutation: body tree mutation
	double pBodyMutate = 0.3;
	boost::random::bernoulli_distribution<double> bodyMutate(pBodyMutate);
	if (bodyMutate(rng_)) {
		// a) Add or remove a body part
		// slight bias to adding, as remove may take away more than one bpart
		boost::random::bernoulli_distribution<double> addNotRemove(0.6);
		if (addNotRemove(rng_)) {
			// robot.addRandomBodyPart(rng_);
		}
		else {
			// robot.popRandomBodyPart(rng_);
		}
		// b) Change orientation of a body part
		double pRotate = 0.2;
		boost::random::bernoulli_distribution<double> rotate(pRotate);
		if (rotate(rng_)) {
			// robot.rotateRandomBodyPart(rng_);
		}
	}
	// 2. Soft mutation: body parameter mutation
	// TODO continue here
	// currently, let's fix body anyways for demo purposes. Later, we can do
	// this only whenever necessary.
	BodyVerifier::fixRobotBody(robot);
#endif

	return mutated;
}

bool Mutator::crossover(RobotRepresentation &a, RobotRepresentation &b) {
	if (!weightCrossover_(rng_))
		return false;

	// 1. get genomes
	std::vector<double*> weights[2];
	std::vector<double*> biases[2];
	a.getBrainGenome(weights[0], biases[0]);
	b.getBrainGenome(weights[1], biases[1]);

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

	a.setDirty();
	b.setDirty();
	return true;
}

typedef bool (Mutator::*MutationOperator)(
		boost::shared_ptr<RobotRepresentation>&);

typedef std::pair<MutationOperator,
		boost::random::bernoulli_distribution<double> > MutOpPair;

void Mutator::mutateBody(boost::shared_ptr<RobotRepresentation> &robot) {

	MutOpPair mutOpPairs[] = { std::make_pair(&Mutator::removeSubtree,
			subtreeRemoval_), std::make_pair(&Mutator::duplicateSubtree,
			subtreeDuplication_), std::make_pair(&Mutator::swapSubtrees,
			subtreeSwap_), std::make_pair(&Mutator::insertNode, nodeInsert_),
			std::make_pair(&Mutator::removeNode, nodeRemoval_), std::make_pair(
					&Mutator::mutateParams, paramMutate_) };

	int numOperators = sizeof(mutOpPairs) / sizeof(MutOpPair);
	for (int i = 0; i < numOperators; ++i) {

		MutationOperator mutOp = mutOpPairs[i].first;
		boost::random::bernoulli_distribution<double> dist =
				mutOpPairs[i].second;

		if (dist(rng_)) {
			for (int attempt = 0; attempt < MAX_MUTATION_ATTEMPTS; ++attempt) {

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

	// Get a random body node (TODO: excluding the root node)
	const RobotRepresentation::IdPartMap idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator node = idPartMap.begin();
	std::advance(node, dist(rng_));

	// Trim the body tree at the selected random body node
	robot->trimBodyAt(node->first);

	return true;
}

bool Mutator::duplicateSubtree(boost::shared_ptr<RobotRepresentation>& robot) {

	// Get a random root of the tree to duplicate (TODO: excluding the root node)
	const RobotRepresentation::IdPartMap idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	RobotRepresentation::IdPartMap::const_iterator rootNode = idPartMap.begin();
	std::advance(rootNode, dist(rng_));

	// Select another node
	RobotRepresentation::IdPartMap::const_iterator destNode = idPartMap.begin();
	std::advance(destNode, dist(rng_));

	// Check if the destination node has free slots
	boost::shared_ptr<PartRepresentation> destPart = destNode->second.lock();
	std::vector<unsigned int> freeSlots = destPart->getFreeChildSlots();
	if (freeSlots.size() > 0) {

		boost::random::uniform_int_distribution<> freeSlotsDist(0,
				freeSlots.size() - 1);
		unsigned int selectedSlotId = freeSlotsDist(rng_);

		return robot->duplicateSubTree(rootNode->first, destNode->first,
				selectedSlotId);

	} else {

		return false;

	}

	return true;

}

bool Mutator::swapSubtrees(boost::shared_ptr<RobotRepresentation>& robot) {

	// Get a random root of the tree to duplicate (TODO: excluding the root node)
	const RobotRepresentation::IdPartMap idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	RobotRepresentation::IdPartMap::const_iterator rootNode1 =
			idPartMap.begin();
	std::advance(rootNode1, dist(rng_));

	boost::shared_ptr<PartRepresentation> rootPart1 = rootNode1->second.lock();

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
	boost::random::uniform_int_distribution<> distRootNode2(0,
			validIds.size() - 1);

	std::string rootPartId2 = validIds[distRootNode2(rng_)];
	boost::weak_ptr<PartRepresentation> rootPartPtr2 = idPartMap.at(
			rootPartId2);

	boost::shared_ptr<PartRepresentation> rootPart2 = rootPartPtr2.lock();

	// TODO Implement return robot->swapSubtrees(rootPart1, rootPart2);
	return true;

}

bool Mutator::insertNode(boost::shared_ptr<RobotRepresentation>& robot) {

	const RobotRepresentation::IdPartMap idPartMap = robot->getBody();
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
	float orientations[4] = { 0, 90, 180, 270 };
	boost::random::uniform_int_distribution<> orientationDist(0, 3);
	float curOrientation = orientations[orientationDist(rng_)];

	// Randomly generate parameters
	unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP[PART_TYPE_MAP[type]];
	std::vector<double> parameters;
	boost::random::uniform_01<double> paramDist;
	for (unsigned int i = 0; i < nParams; ++i) {
		parameters.push_back(paramDist(rng_));
	}

	// Create the new part
	boost::shared_ptr<PartRepresentation> newNode = PartRepresentation::create(
			type, "", curOrientation, parameters);

	// Generate a random slot in the new node
	boost::random::uniform_int_distribution<> distNewNodeSlot(0,
			newNode->getArity() - 1);

	return robot->insertNode(parent->first, parentSlot, newNode, distNewNodeSlot(rng_));

}

bool Mutator::removeNode(boost::shared_ptr<RobotRepresentation>& robot) {

	// Select node for removal (TODO: Exclude the root)
	const RobotRepresentation::IdPartMap idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator nodeToRemove =
			idPartMap.begin();
	std::advance(nodeToRemove, dist(rng_));

	return robot->removeNode(nodeToRemove->first);

}

bool Mutator::mutateParams(boost::shared_ptr<RobotRepresentation>& robot) {

	// Select node for mutation
	const RobotRepresentation::IdPartMap idPartMap = robot->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	RobotRepresentation::IdPartMap::const_iterator nodeToMutate =
			idPartMap.begin();
	std::advance(nodeToMutate, dist(rng_));

	std::vector<double> params = nodeToMutate->second.lock()->getParams();
	if (params.size() > 0) {

		// Select a random parameter to mutate
		boost::random::uniform_int_distribution<> distMutation(0,
				params.size() - 1);
		unsigned int paramToMutate = distMutation(rng_);

		params[paramToMutate] += paramDistribution_(rng_);
		if (params[paramToMutate] < 0) {
			params[paramToMutate] = 0;
		} else if (params[paramToMutate] > 1) {
			params[paramToMutate] = 1;
		}

	}

	return true;

}

} /* namespace robogen */
