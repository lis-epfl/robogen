/*
 * @(#) RobotRepresentation.cpp   1.0   Aug 28, 2013
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

#include "evolution/representation/RobotRepresentation.h"
#include "Robogen.h"

#ifndef FAKEROBOTREPRESENTATION_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include "evolution/representation/PartRepresentation.h"
#include "utils/network/ProtobufPacket.h"
#include "PartList.h"
#include "utils/json2pb/json2pb.h"
#include "utils/RobogenUtils.h"
#include "brain/NeuralNetwork.h"

#define VERIFY_ON_LOAD_TXT
#ifdef VERIFY_ON_LOAD_TXT
#include "evolution/engine/BodyVerifier.h"
#endif

namespace robogen {

RobotRepresentation::RobotRepresentation() :
		evaluated_(false) {
			//I should initialize maxId to 1000
}

RobotRepresentation::RobotRepresentation(const RobotRepresentation &r) {

	this->robotMorph_.reset(new SubRobotRepresentation(*(r.robotMorph_.get())));

/*
	// we need to handle bodyTree_, neuralNetwork_ and reservedIds_
	// for the brainevolver, we could theoretically keep the bodyTree_ pointing
	// to the same body, but that would be easy to miss when resuming to body
	// evolution, so we'll just do proper copying right away

	// special treatment for base-pointed instances of derived classes as are
	// our body parts
	bodyTree_ = r.bodyTree_->cloneSubtree();
	// neural network pointer needs to be reset to a copy-constructed instance
	neuralNetwork_.reset(
			new NeuralNetworkRepresentation(*(r.neuralNetwork_.get())));
	// rebuild ID to part map
	idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(bodyTree_);
	while (!q.empty()) {
		boost::shared_ptr<PartRepresentation> cur = q.front();
		q.pop();
		idToPart_[cur->getId()] = boost::weak_ptr<PartRepresentation>(cur);
		for (unsigned int i = 0; i < cur->getArity(); ++i) {
			if (cur->getChild(i)) {
				q.push(cur->getChild(i));
			}
		}
	}
	// fitness and associated flag are same
	fitness_ = r.fitness_;
	evaluated_ = r.evaluated_;
	maxid_ = r.maxid_;
*/

	// Similar treatment for the grammar
	this->grammar_.reset(new Grammar(this->robotMorph_));
}

RobotRepresentation &RobotRepresentation::operator=(
		const RobotRepresentation &r) {
	// same as copy constructor, see there for explanations
	/*
	bodyTree_ = r.bodyTree_->cloneSubtree();
	neuralNetwork_.reset(
			new NeuralNetworkRepresentation(*(r.neuralNetwork_.get())));
	// rebuild ID to part map
	idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(bodyTree_);
	while (!q.empty()) {
		boost::shared_ptr<PartRepresentation> cur = q.front();
		q.pop();
		idToPart_[cur->getId()] = boost::weak_ptr<PartRepresentation>(cur);
		for (unsigned int i = 0; i < cur->getArity(); ++i) {
			if (cur->getChild(i)) {
				q.push(cur->getChild(i));
			}
		}
	}
	*/
	*this->robotMorph_ = *(r.robotMorph_);
	fitness_ = r.fitness_;
	evaluated_ = r.evaluated_;
	return *this;
}

void RobotRepresentation::asyncEvaluateResult(double fitness) {
	fitness_ = fitness;
	evaluated_ = true;
}

bool RobotRepresentation::init() {

	/*
	// Generate a core component
	boost::shared_ptr<PartRepresentation> corePart = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_CORE_COMPONENT),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());
	if (!corePart) {
		std::cout << "Failed to create root node" << std::endl;
		return false;
	}
	bodyTree_ = corePart;
	//this->grammar_.reset(new Grammar(bodyTree_->cloneSubtree()));
	idToPart_[PART_TYPE_CORE_COMPONENT] = boost::weak_ptr<PartRepresentation>(
			corePart);

	// TODO abstract this to a different function so it doesn't
	// duplicate what we have below

	// process brain
	std::string from, to;
	// create neural network: create map from body id to ioId for all sensor and
	// motor body parts
	std::map<std::string, int> sensorMap, motorMap;
	for (std::map<std::string,
			boost::weak_ptr<PartRepresentation> >::iterator it =
			idToPart_.begin(); it != idToPart_.end(); it++) {

		// omitting weak pointer checks, as this really shouldn't go wrong here!
		if (it->second.lock()->getMotors().size()) {
			motorMap[it->first] = it->second.lock()->getMotors().size();
		}
		if (it->second.lock()->getSensors().size()) {
			sensorMap[it->first] = it->second.lock()->getSensors().size();
		}

	}

	neuralNetwork_.reset(new NeuralNetworkRepresentation(sensorMap, motorMap));
	*/

	this->robotMorph_->init();

	this->grammar_.reset(new Grammar(this->robotMorph_));

	return true;
}

bool RobotRepresentation::init(std::string robotTextFile) {

	/*
	// open file
	std::ifstream file;
	file.open(robotTextFile.c_str());
	if (!file.is_open()) {
		std::cout << "Could not open robot text file " << robotTextFile
				<< std::endl;
		return false;
	}

	// prepare body processing
	boost::shared_ptr<PartRepresentation> current;
	std::stack<boost::shared_ptr<PartRepresentation> > parentStack;
	unsigned int slot, orientation, indent;
	char type;
	std::string line, id;
	std::vector<double> params;

	// process root node

	if (!robotTextFileReadPartLine(file, indent, slot, type, id, orientation,
			params) || indent) {
		std::cout << "Robot text file contains no or"
				" poorly formatted root node" << std::endl;
		return false;
	}

	current = PartRepresentation::create(type, id, orientation, params);
	if (!current) {
		std::cout << "Failed to create root node" << std::endl;
		return false;
	}
	bodyTree_ = current;
	idToPart_[id] = boost::weak_ptr<PartRepresentation>(current);

	// process other body parts

	while (robotTextFileReadPartLine(file, indent, slot, type, id,
			orientation, params)) {
		if (!indent) {
			std::cout << "Attempt to create multiple root nodes!"
					<< std::endl;
			return false;
		}
		// indentation: Adding children to current
		if (indent > (parentStack.size())) {
			parentStack.push(current);
		}
		// indentation: done adding children to top of parent stack
		for (; indent < (parentStack.size());) {
			parentStack.pop();
		}
		current = PartRepresentation::create(type, id, orientation, params);
		if (!current) {
			std::cout << "Failed to create node." << std::endl;
			return false;
		}
		if (parentStack.top()->getChild(slot)) {
			std::cout << "Attempt to overwrite child "
					<< parentStack.top()->getChild(slot)->getId() << " of "
					<< parentStack.top()->getId() << " with "
					<< current->getId() << std::endl;
			return false;
		}
		if (!parentStack.top()->setChild(slot, current)) {
			std::cout << "Failed to set child." << std::endl;
			return false;
		}
		idToPart_[id] = boost::weak_ptr<PartRepresentation>(current);
	}

	// process brain
	std::string from, to;
	int fromIoId, toIoId;
	double value;
	// create neural network: create map from body id to ioId for all sensor and
	// motor body parts
	std::map<std::string, int> sensorMap, motorMap;
	for (std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator
			it = idToPart_.begin(); it != idToPart_.end(); it++) {

		// omitting weak pointer checks, as this really shouldn't go wrong here!
		if (it->second.lock()->getMotors().size()) {
			motorMap[it->first] = it->second.lock()->getMotors().size();
		}
		if (it->second.lock()->getSensors().size()) {
			sensorMap[it->first] = it->second.lock()->getSensors().size();
		}

	}

	neuralNetwork_.reset(new NeuralNetworkRepresentation(sensorMap, motorMap));
	unsigned int neuronType;
	// add new neurons

	while (robotTextFileReadAddNeuronLine(file, id, neuronType)) {
		if (neuralNetwork_->getNumHidden() < MAX_HIDDEN_NEURONS) {
			std::string neuronId = neuralNetwork_->insertNeuron(ioPair(id,
					neuralNetwork_->getBodyPartNeurons(id).size()),
					NeuronRepresentation::HIDDEN, neuronType);
			std::cout << "added hidden neuron "  << neuronId << " with type "
					<< neuronType << std::endl;
		} else {
			std::cerr << "The number of specified hidden neurons is more than "
					<< MAX_HIDDEN_NEURONS << ", which is the maximuma allowed."
					<< std::endl;
			return false;
		}
	}

	// weights
	while (robotTextFileReadWeightLine(file, from, fromIoId, to, toIoId,
			value)) {
		if (!neuralNetwork_->setWeight(from, fromIoId, to, toIoId, value)) {
			std::cerr << "Failed to set weight" << std::endl;
			return false;
		}
	}

	// params
	params.clear();

	while (robotTextFileReadParamsLine(file, to, toIoId, neuronType, params)) {
		if (!neuralNetwork_->setParams(to, toIoId, neuronType, params)) {
			std::cerr << "Failed to set neuron params" << std::endl;
			return false;
		}
		params.clear();
	}


	while(!file.eof()) {
		RobogenUtils::safeGetline(file, line);
		if(!isLineEmpty(line)) {
			std::cerr << std::endl << std::endl
					<< "The robot text file has non-empty lines after all "
					<< "body and brain lines have been parsed!" << std::endl
					<< "Are you sure the file is properly formatted??"
					<< std::endl << std::endl;
			exitRobogen(EXIT_FAILURE);
		}
	}


	file.close();

	maxid_ = 1000;

	// loop through existing ids to find what new maxid should be.
	// this is necessary when trying to seed evolution with a previously
	// evolved morphology
	for(IdPartMap::iterator i = idToPart_.begin(); i!= idToPart_.end(); ++i) {
		if(i->first.substr(0,4).compare("myid") == 0) {
			int idVal = atoi(i->first.substr(4).c_str());
			if (idVal >= maxid_) {
				maxid_ = idVal + 1;
			}
		}
	}
	*/

	this->robotMorph_.reset(new SubRobotRepresentation());
	this->robotMorph_->init(robotTextFile);

	this->grammar_.reset(new Grammar(this->robotMorph_));

	return true;
}

robogenMessage::Robot RobotRepresentation::serialize() const {
	/*
	robogenMessage::Robot message;
	// id - this can probably be removed
	message.set_id(1);
	// body
	bodyTree_->addSubtreeToBodyMessage(message.mutable_body(), true);
	// brain
	*(message.mutable_brain()) = neuralNetwork_->serialize();
	return message;
	*/
	return this->robotMorph_->serialize();
}


void RobotRepresentation::getBrainGenome(std::vector<double*> &weights,
		std::vector<unsigned int> &types,
		std::vector<double*> &params) {
	//neuralNetwork_->getGenome(weights, types, params);
	this->robotMorph_->getBrainGenome(weights,types,params);
}

boost::shared_ptr<NeuralNetworkRepresentation> RobotRepresentation::getBrain(
		) const {
	//return neuralNetwork_;
	return this->robotMorph_->getBrain();
}

const SubRobotRepresentation::IdPartMap& RobotRepresentation::getBody() const {
	//return idToPart_;
	return this->robotMorph_->getBody();
}

void RobotRepresentation::rebuildBodyMap(){
	// rebuild ID to part map
	/*
	this->idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(this->bodyTree_);
	while (!q.empty()) {
		boost::shared_ptr<PartRepresentation> cur = q.front();
		q.pop();
		//Using at instead of []
		this->idToPart_[cur->getId()] = boost::weak_ptr<PartRepresentation>(cur);
		for (unsigned int i = 0; i < cur->getArity(); ++i) {
			if (cur->getChild(i)) {
				q.push(cur->getChild(i));
			}
		}
	}
	*/
}

boost::shared_ptr<Grammar> RobotRepresentation::getGrammar(void){
	return this->grammar_;
}

const std::string& RobotRepresentation::getBodyRootId() {
	//return bodyTree_->getId();
	return this->robotMorph_->getTree()->getId();
}

void RobotRepresentation::evaluate(Socket *socket,
		boost::shared_ptr<RobogenConfig> robotConf) {

	// 1. Prepare message to simulator
	boost::shared_ptr<robogenMessage::EvaluationRequest> evalReq(
			new robogenMessage::EvaluationRequest());
	robogenMessage::Robot* evalRobot = evalReq->mutable_robot();
	robogenMessage::SimulatorConf* evalConf = evalReq->mutable_configuration();
	*evalRobot = serialize();
	*evalConf = robotConf->serialize();

	ProtobufPacket<robogenMessage::EvaluationRequest> robotPacket(evalReq);
	std::vector<unsigned char> forgedMessagePacket;
	robotPacket.forge(forgedMessagePacket);

	// 2. send message to simulator
	socket->write(forgedMessagePacket);

#ifndef EMSCRIPTEN // we will do it later with javascript
	// 3. receive message from simulator
	ProtobufPacket<robogenMessage::EvaluationResult> resultPacket(
			boost::shared_ptr<robogenMessage::EvaluationResult>(
					new robogenMessage::EvaluationResult()));
	std::vector<unsigned char> responseMessage;
	socket->read(responseMessage,
			ProtobufPacket<robogenMessage::EvaluationResult>::HEADER_SIZE);

	// Decode the Header and read the payload-message-size
	size_t msgLen = resultPacket.decodeHeader(responseMessage);
	responseMessage.clear();

	// Read the fitness payload message
	socket->read(responseMessage, msgLen);

	// Decode the packet
	resultPacket.decodePayload(responseMessage);

	// 4. write fitness to individual TODO exception
	if (!resultPacket.getMessage()->has_fitness()) {
		std::cerr << "Fitness field not set by Simulator!!!" << std::endl;
		exit(EXIT_FAILURE);
	} else {
		fitness_ = resultPacket.getMessage()->fitness();
		evaluated_ = true;
	}
#endif

}

double RobotRepresentation::getFitness() const {
	return fitness_;
}

bool RobotRepresentation::isEvaluated() const {
	return evaluated_;
}

void RobotRepresentation::setDirty() {
	evaluated_ = false;
}

bool RobotRepresentation::trimBodyAt(const std::string& id, bool printErrors) {
	/*
	// kill all neurons and their weights
	recurseNeuronRemoval(idToPart_[id].lock());

	// thanks to shared pointer magic, we only need to reset the shared pointer
	// to the indicated body part
	PartRepresentation *parent = idToPart_[id].lock()->getParent();
	int position = idToPart_[id].lock()->getPosition();
	if (!parent) {
		if (printErrors) {
			std::cerr << "Trying to remove root body part!" << std::endl;
		}
		return false;
	}
	//std::cout << "Has references: " << idToPart_[id].lock().use_count()
	//		<< std::endl;
	if (!parent->setChild(position, boost::shared_ptr<PartRepresentation>())) {
		if (printErrors) {
			std::cerr << "Failed trimming robot body!" << std::endl;
		}
		return false;
	}
	if (!parent->getChild(position)) {
		//std::cout << "Successfully removed" << std::endl;
	}
	// need to update the id to body part map! Easily done with weak pointers
	for (IdPartMap::iterator it = idToPart_.begin(); it != idToPart_.end();) {
		if (!it->second.lock()) {
			idToPart_.erase(it++);
			//std::cout << "Had a part to erase! " << parent << std::endl;
		} else {
			++it;
		}
	}
	return true;
	*/

	return this->robotMorph_->trimBodyAt(id,printErrors);
}

std::string RobotRepresentation::generateUniqueIdFromSomeId() {

	/*
	std::stringstream ss;
	ss << "myid" << maxid_;

	std::string newUniqueId = ss.str();
	maxid_++;
	return newUniqueId;
	*/

}

bool RobotRepresentation::duplicateSubTree(const std::string& subtreeRootPartId,
		const std::string& subtreeDestPartId, unsigned int slotId,
		bool printErrors) {

	/*
	// find src part and dest part by id
	boost::shared_ptr<PartRepresentation> src =
			idToPart_[subtreeRootPartId].lock();
	boost::shared_ptr<PartRepresentation> dst =
			idToPart_[subtreeDestPartId].lock();

	// If source is root node, then return
	if (src->getId().compare(bodyTree_->getId()) == 0) {
		if (printErrors) {
			std::cerr << "Cannot duplicate root node!" << std::endl;
		}
		return false;
	}

	boost::shared_ptr<PartRepresentation> clone = src->cloneSubtree();
	dst->setChild(slotId, clone);

	std::map<std::string, std::string> neuronReMapping;

	// insert clones into part map and generate neurons
	this->addClonesToMap(clone, neuronReMapping);

	// correctly generate the weights
	neuralNetwork_->generateCloneWeights(neuronReMapping);

	return true;
	*/

	return this->robotMorph_->duplicateSubTree(subtreeRootPartId, subtreeDestPartId, slotId, printErrors);

}

bool RobotRepresentation::swapSubTrees(const std::string& subtreeRoot1,
		const std::string& subtreeRoot2, bool printErrors) {

			/*
	// Get roots of the subtrees
	boost::shared_ptr<PartRepresentation> root1 =
			idToPart_[subtreeRoot1].lock();
	boost::shared_ptr<PartRepresentation> root2 =
			idToPart_[subtreeRoot2].lock();

	// Check none of them is the root node
	if (root1->getId().compare(bodyTree_->getId()) == 0
			|| root2->getId().compare(bodyTree_->getId()) == 0) {
		if (printErrors) {
			std::cerr << "Cannot swap root subtree" << std::endl;
		}
		return false;
	}

	// Get parents and slots of each subtree
	PartRepresentation* parentRoot1 = root1->getParent();
	PartRepresentation* parentRoot2 = root2->getParent();

	// Get the slots to which this nodes are connected
	unsigned int slotParentRoot1 = 1000000;

	for (unsigned int i = 0; i < parentRoot1->getArity(); ++i) {
		if (parentRoot1->getChild(i) != NULL) {
			if (parentRoot1->getChild(i)->getId().compare(root1->getId())
					== 0) {
				slotParentRoot1 = i;
				break;
			}
		}
	}

	unsigned int slotParentRoot2 = 0;
	for (unsigned int i = 0; i < parentRoot2->getArity(); ++i) {
		if (parentRoot2->getChild(i) != NULL) {
			if (parentRoot2->getChild(i)->getId().compare(root2->getId())
					== 0) {
				slotParentRoot2 = i;
				break;
			}
		}
	}

	// Swap the subtrees
	parentRoot2->setChild(slotParentRoot2, root1);
	parentRoot1->setChild(slotParentRoot1, root2);

	return true;
	*/

	return this->robotMorph_->swapSubTrees(subtreeRoot1, subtreeRoot2, printErrors);

}

bool RobotRepresentation::insertPart(const std::string& parentPartId,
		unsigned int parentPartSlot,
		boost::shared_ptr<PartRepresentation> newPart,
		unsigned int newPartSlot,
		unsigned int motorNeuronType, bool printErrors) {

	/*

	// Set new ID for the inserted node
	std::string newUniqueId = this->generateUniqueIdFromSomeId();
	newPart->setId(newUniqueId);

	// create Neurons in NeuralNetwork
	std::vector<std::string> sensors = newPart->getSensors();
	for (unsigned int i = 0; i < sensors.size(); ++i) {
		neuralNetwork_->insertNeuron(ioPair(newPart->getId(), i),
				NeuronRepresentation::INPUT, NeuronRepresentation::SIMPLE);
	}
	std::vector<std::string> motors = newPart->getMotors();
	for (unsigned int i = 0; i < motors.size(); ++i) {
		neuralNetwork_->insertNeuron(
				ioPair(newPart->getId(), sensors.size() + i),
				NeuronRepresentation::OUTPUT, motorNeuronType);
	}

	// find dst part by id
	boost::shared_ptr<PartRepresentation> parentPart =
			idToPart_[parentPartId].lock();
	boost::shared_ptr<PartRepresentation> childPart = parentPart->getChild(
			parentPartSlot);

	// Check the arity of the new part
	if (childPart != NULL && newPart->getArity() < 1) {
		if (printErrors) {
			std::cerr << "Cannot insert 0-arity part when there is already a"
					" part present at its location" << std::endl;
		}
		return false;
	}

	parentPart->setChild(parentPartSlot, newPart);

	if (childPart != NULL)
		newPart->setChild(newPartSlot, childPart);

	// Add to the map
	idToPart_[newUniqueId] = boost::weak_ptr<PartRepresentation>(newPart);

	return true;
	*/

	return this->robotMorph_->insertPart(parentPartId, parentPartSlot, newPart, newPartSlot, motorNeuronType, printErrors);

}

bool RobotRepresentation::removePart(const std::string& partId,
		bool printErrors) {

/*
	boost::shared_ptr<PartRepresentation> nodeToRemove =
			idToPart_[partId].lock();

	// If root node, return
	if (nodeToRemove->getId().compare(bodyTree_->getId()) == 0) {
		if (printErrors) {
			std::cerr << "Cannot remove root" << std::endl;
		}
		return false;
	}

	// Get parent of node to be removed
	PartRepresentation* parent = nodeToRemove->getParent();

	// this should be handled by check above, but just to be safe
	if (parent == NULL) {
		if (printErrors) {
				std::cerr << "Cannot remove root" << std::endl;
			}
		return false;
	}

	// Add one since will be freeing a slot when this node is removed
	unsigned int nFreeSlots = parent->getFreeSlots().size() + 1;
	if (nFreeSlots < nodeToRemove->getChildrenCount()) {
		if (printErrors) {
			std::cerr << "Not enough free slots on parent to remove part."
					<< "Need " << nodeToRemove->numDescendants()
					<< ", but only have " << nFreeSlots
					<< std::endl;
		}
		return false;
	}

	// remove neurons and all their connections
	neuralNetwork_->removeNeurons(partId);

	// Find child part that will be removed
	for (unsigned int i = 0; i < parent->getArity(); i++) {
		if (parent->getChild(i) != NULL &&
				parent->getChild(i)->getId().compare(nodeToRemove->getId())
				== 0) {
			parent->setChild(i, boost::shared_ptr<PartRepresentation>());
			break;
		}
	}
	idToPart_.erase(nodeToRemove->getId());

	if ( nodeToRemove->getArity() > 0 ) {
		unsigned int indx = 0;
		for (unsigned int i = 0; i < parent->getArity(); i++) {

			if (parent->getChild(i) == NULL) {

				while (nodeToRemove->getChild(indx) == NULL) {
					indx++;
					if (indx >= nodeToRemove->getArity()) {
						return true;
					}
				}

				parent->setChild(i, nodeToRemove->getChild(indx));

				// Increment current index
				indx++;

				if (indx >= nodeToRemove->getArity()) {
					return true;
				}

			}
		}
	}
	return true;
	*/

	return this->robotMorph_->removePart(partId, printErrors);
}

bool RobotRepresentation::check() {

/*
	// 1. Check that every body part in the body tree is in the idBodyPart map
	// and there are no dangling references
	std::vector<std::string> bodyPartIdsFromMap;
	for (std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator
			it = idToPart_.begin(); it != idToPart_.end(); ++it) {
		bodyPartIdsFromMap.push_back(it->first);
	}

	std::vector<std::string> bodyIds = bodyTree_->getDescendantsIds();
	bodyIds.push_back(bodyTree_->getId());

	std::sort(bodyPartIdsFromMap.begin(), bodyPartIdsFromMap.end());
	std::sort(bodyIds.begin(), bodyIds.end());

	bool idsMatch = true;
	if (bodyPartIdsFromMap.size() != bodyIds.size()) {
		std::cout << "Error: bodyPartIdsFromMap has " <<
				bodyPartIdsFromMap.size() << " elements, but bodyIds has " <<
				bodyIds.size() << " elements\n";
		idsMatch = false;
	} else {
		for (unsigned int i = 0; i < bodyPartIdsFromMap.size(); ++i) {
			if (bodyPartIdsFromMap[i].compare(bodyIds[i]) != 0) {
				std::cerr << "Error: bodyPartIdsFromMap does not match bodyIds "
						<< "at "
						<< "position " << i << " " << bodyPartIdsFromMap[i]
						<< " " << bodyIds[i] << "\n";
				idsMatch = false;
			}
		}
	}

	if (!idsMatch) {
		std::cerr << "bodyPartIdsFromMap:";
		for (unsigned int i = 0; i < bodyPartIdsFromMap.size(); ++i) {
			std::cerr << " " << bodyPartIdsFromMap[i] ;
		}
		std::cerr << "\nbodyIds:";
		for (unsigned int i = 0; i < bodyIds.size(); ++i) {
			std::cerr << " " << bodyIds[i] ;
		}
		std::cerr << "\n";

		return false;
	}

	// 2. Check that each neuron has an associated body part
	std::map<std::string, int> netInputs;
	std::map<std::string, int> netOutputs;
	for (unsigned int i = 0; i < bodyIds.size(); ++i) {

		bool hasNeurons = true;
		boost::weak_ptr<PartRepresentation> part = idToPart_[bodyIds[i]];
		if (part.lock()->getSensors().size() > 0) {
			netInputs[bodyIds[i]] = part.lock()->getSensors().size();
		}

		if (part.lock()->getMotors().size() > 0) {
			netOutputs[bodyIds[i]] = part.lock()->getMotors().size();
		}

		if (hasNeurons) {

			std::vector<boost::weak_ptr<NeuronRepresentation> > neurons =
					neuralNetwork_->getBodyPartNeurons(bodyIds[i]);
			int totInputs = 0;
			int totOutputs = 0;
			for (unsigned int j = 0; j < neurons.size(); ++j) {
				unsigned int layer = neurons[j].lock()->getLayer();
				if (layer == NeuronRepresentation::INPUT) {
					totInputs++;
				} else if (layer == NeuronRepresentation::OUTPUT) {
					totOutputs++;
				}
			}

			if (totInputs != netInputs[bodyIds[i]] ||
					totOutputs != netOutputs[bodyIds[i]]) {
				return false;
			}
		}

	}

	// TODO Consistency check is not complete, neural representation is
	// only partially checked
	return true;
	*/

	return this->robotMorph_->check();

}

std::string RobotRepresentation::toString() {

	std::stringstream str;
	str << "[" << this->robotMorph_->getTree()->getId() << " | " << this->robotMorph_->getTree()->getType() << "]"
			<< std::endl;
	this->robotMorph_->getTree()->toString(str, 0);
	str << "Network:" << std::endl;
	str << this->robotMorph_->getBrain()->toString();
	return str.str();

}


bool RobotRepresentation::createRobotMessageFromFile(robogenMessage::Robot
		&robotMessage, std::string robotFileString) {

	if (boost::filesystem::path(robotFileString).extension().string().compare(
			".dat") == 0) {

		std::ifstream robotFile(robotFileString.c_str(), std::ios::binary);
		if (!robotFile.is_open()) {
			std::cerr << "Cannot open " << robotFileString << ". Quit."
					<< std::endl;
			return false;
		}

		ProtobufPacket<robogenMessage::Robot> robogenPacket;

		robotFile.seekg(0, robotFile.end);
		unsigned int packetSize = robotFile.tellg();
		robotFile.seekg(0, robotFile.beg);

		std::vector<unsigned char> packetBuffer;
		packetBuffer.resize(packetSize);
		robotFile.read((char*) &packetBuffer[0], packetSize);
		robogenPacket.decodePayload(packetBuffer);
		robotMessage = *robogenPacket.getMessage().get();

	} else if (boost::filesystem::path(robotFileString
			).extension().string().compare(".txt") == 0) {

		RobotRepresentation robot;
		if (!robot.init(robotFileString)) {
			std::cerr << "Failed interpreting robot text file!" << std::endl;
			return false;
		}

#ifdef VERIFY_ON_LOAD_TXT
		int errorCode;

		std::vector<std::pair<std::string, std::string> > affectedBodyParts;
		if (!BodyVerifier::verify(robot, errorCode,
						affectedBodyParts, true)) {
			std::cerr << std::endl
				<< "*************** Body does not verify!! *****************"
				<< std::endl << std::endl;
			return false;
		}
#endif

		robotMessage = robot.serialize();

	} else if (boost::filesystem::path(robotFileString
				).extension().string().compare(".json") == 0) {
		std::ifstream robotFile(robotFileString.c_str(),
								std::ios::in | std::ios::binary);

		if (!robotFile.is_open()) {
			std::cerr << "Cannot open " << robotFileString << ". Quit."
					<< std::endl;
			return false;
		}

		robotFile.seekg(0, robotFile.end);
		unsigned int packetSize = robotFile.tellg();
		robotFile.seekg(0, robotFile.beg);

		std::vector<unsigned char> packetBuffer;
		packetBuffer.resize(packetSize);
		robotFile.read((char*) &packetBuffer[0], packetSize);

		json2pb(robotMessage, (char*) &packetBuffer[0], packetSize);

	} else {
		std::cerr << "File extension of provided robot file could not be "
				"resolved. Use .dat or .json for robot messages and .txt for "
				"robot text files" << std::endl;
		return false;
	}
	return true;

}

}

#endif /* usage of fake robot representation */
