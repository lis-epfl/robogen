/*
 * @(#) RobotRepresentation.cpp   1.0   Aug 28, 2013
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

#include "evolution/representation/RobotRepresentation.h"
#ifndef FAKEROBOTREPRESENTATION_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>
#include <boost/regex.hpp>
#include "evolution/representation/PartRepresentation.h"
#include "utils/network/ProtobufPacket.h"

namespace robogen {

RobotRepresentation::RobotRepresentation() :
		maxid_(1000) {

}

RobotRepresentation::RobotRepresentation(const RobotRepresentation &r) {

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
}

/**
 * Helper function for decoding a part line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadPartLine(std::ifstream &file, int &indent, int &slot,
		char &type, std::string &id, int &orientation,
		std::vector<double> &params) {
	// match (0 or more tabs)(digit) (type) (id) (orientation) (parameters)
	static const boost::regex rx(
			"^(\\t*)(\\d) ([A-Z]) ([^\\s]+) (\\d)([ \\d\\.-]*)$");
	boost::cmatch match;
	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)) {
		// match[0]:whole string, match[1]:tabs, match[2]:slot, match[3]:type,
		// match[4]:id, match[5]:orientation, match[6]:parameters
		indent = match[1].length();
		slot = std::atoi(match[2].first);
		type = match[3].first[0];
		id = std::string(match[4]);
		orientation = std::atoi(match[5].first);
		double param;
		std::stringstream ss(match[6]);
		params.clear();

		std::vector<double> rawParams;
		while (ss >> param) {
			rawParams.push_back(param);
		}
		if (rawParams.size()
				!= PART_TYPE_PARAM_COUNT_MAP[PART_TYPE_MAP[type]]) {
			std::cout << "Error reading body part from text file.\n"
					<< PART_TYPE_MAP[type] << " requires "
					<< PART_TYPE_PARAM_COUNT_MAP[PART_TYPE_MAP[type]]
					<< " params, but " << rawParams.size()
					<< " were received\n";
			return false;
		}
		for (unsigned int i = 0; i < rawParams.size(); i++) {
			std::pair<double, double> ranges =
					PART_TYPE_PARAM_RANGE_MAP[std::make_pair(
							PART_TYPE_MAP[type], i)];
			double param = rawParams[i];
			if (param < ranges.first || param > ranges.second) {
				std::cout << "Error reading body part from text file.\n"
						<< PART_TYPE_MAP[type] << " requires param " << i
						<< " to be in [" << ranges.first << ", "
						<< ranges.second << "], but " << param
						<< " was received\n";
				return false;
			}
			//add param in [0,1]
			params.push_back(
					(param - ranges.first) / (ranges.second - ranges.first) );
		}

		return true;
	} else {
		// additional info if poor formatting, i.e. line not empty
		static const boost::regex spacex("^\\s*$");
		if (!boost::regex_match(line.c_str(), spacex)) {
			std::cout << "Error reading body part from text file. Received:\n"
					<< line << "\nbut expected format:\n"
					<< "<0 or more tabs><slot index digit> <part type character> "
							"<part id string> <orientation digit> <evt. parameters>"
					<< std::endl;
		}
		return false;
	}
}

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadWeightLine(std::ifstream &file, std::string &from,
		int &fromIoId, std::string &to, int &toIoId, double &value) {
	// TODO with this regex, not sure if weight value is read
	static const boost::regex rx(
			"^([^\\s]+) (\\d+) ([^\\s]+) (\\d+) (-?\\d*\\.?\\d*)$");
	boost::cmatch match;
	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)) {
		// match[0]:whole string, match[1]:from, match[2]:from IO id,
		// match[3]:to, match[4]:to IO id, match[5]:value
		from.assign(match[1]);
		fromIoId = std::atoi(match[2].first);
		to.assign(match[3]);
		toIoId = std::atoi(match[4].first);
		value = std::atof(match[5].first);
		return true;
	} else {
		// additional info if poor formatting, i.e. line not empty
		static const boost::regex spacex("^\\s*$");
		if (!boost::regex_match(line.c_str(), spacex)) {
			std::cout << "Error reading weight from text file. Received:\n"
					<< line << "\nbut expected format:\n"
					<< "<source part id string> <source part io id> "
							"<destination part id string> <destination part io id> "
							"<weight>" << std::endl;
		}
		return false;
	}
}

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadBiasLine(std::ifstream &file, std::string &node,
		int &ioId, double &value) {
	// TODO with this regex, not sure if bias value is read
	static const boost::regex rx("^([^\\s]+) (\\d+) (-?\\d*\\.?\\d*)$");
	boost::cmatch match;
	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)) {
		// match[0]:whole string, match[1]:node, match[2]:ioId, match[3]:value
		node.assign(match[1]);
		ioId = std::atoi(match[2].first);
		value = std::atof(match[3].first);
		return true;
	} else {
		// additional info if poor formatting, i.e. line not empty
		static const boost::regex spacex("^\\s*$");
		if (!boost::regex_match(line.c_str(), spacex)) {
			std::cout << "Error reading brain bias from text file. Received:\n"
					<< line << "\nbut expected format:\n"
					<< "<part id string> <part io id> "
							"<bias>" << std::endl;
		}
		return false;
	}
}

RobotRepresentation &RobotRepresentation::operator=(
		const RobotRepresentation &r) {
	// same as copy constructor, see there for explanations
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
	fitness_ = r.fitness_;
	evaluated_ = r.evaluated_;
	maxid_ = r.maxid_;
	return *this;
}

bool RobotRepresentation::init(std::string robotTextFile) {

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
	int slot, orientation, indent;
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
	while (robotTextFileReadPartLine(file, indent, slot, type, id, orientation,
			params)) {
		if (!indent) {
			std::cout << "Attempt to create multiple root nodes!" << std::endl;
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
	for (std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator it =
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

	// weights
	while (robotTextFileReadWeightLine(file, from, fromIoId, to, toIoId, value)) {
		if (!neuralNetwork_->setWeight(from, fromIoId, to, toIoId, value)) {
			std::cout << "Failed to set weight" << std::endl;
			return false;
		}
	}

	// biases
	while (robotTextFileReadBiasLine(file, to, toIoId, value)) {
		if (!neuralNetwork_->setBias(to, toIoId, value)) {
			std::cout << "Failed to set bias" << std::endl;
			return false;
		}
	}
	file.close();

	maxid_ = 1000;
	return true;
}

robogenMessage::Robot RobotRepresentation::serialize() const {
	robogenMessage::Robot message;
	// id - this can probably be removed
	message.set_id(1);
	// body
	bodyTree_->addSubtreeToBodyMessage(message.mutable_body(), true);
	// brain
	*(message.mutable_brain()) = neuralNetwork_->serialize();
	return message;
}

void RobotRepresentation::randomizeBrain(boost::random::mt19937 &rng) {
	neuralNetwork_->initializeRandomly(rng);
}

void RobotRepresentation::getBrainGenome(std::vector<double*> &weights,
		std::vector<double*> &biases) {
	neuralNetwork_->getGenome(weights, biases);
}

boost::shared_ptr<NeuralNetworkRepresentation> RobotRepresentation::getBrain() const {
	return neuralNetwork_;
}

const RobotRepresentation::IdPartMap& RobotRepresentation::getBody() const {
	return idToPart_;
}

const std::string& RobotRepresentation::getBodyRootId() {
	return bodyTree_->getId();
}

void RobotRepresentation::evaluate(TcpSocket *socket,
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

bool RobotRepresentation::trimBodyAt(const std::string& id) {

	// thanks to shared pointer magic, we only need to reset the shared pointer
	// to the indicated body part
	PartRepresentation *parent = idToPart_[id].lock()->getParent();
	int position = idToPart_[id].lock()->getPosition();
	if (!parent) {
		std::cout << "Trying to remove root body part!" << std::endl;
		return false;
	}
	std::cout << "Has references: " << idToPart_[id].lock().use_count()
			<< std::endl;
	if (!parent->setChild(position, boost::shared_ptr<PartRepresentation>())) {
		std::cout << "Failed trimming robot body!" << std::endl;
		return false;
	}
	if (!parent->getChild(position)) {
		std::cout << "Successfully removed" << std::endl;
	}
	// need to update the id to body part map! Easily done with weak pointers
	for (IdPartMap::iterator it = idToPart_.begin(); it != idToPart_.end();) {
		if (!it->second.lock()) {
			idToPart_.erase(it++);
			std::cout << "Had a part to erase! " << parent << std::endl;
		} else
			++it;
	}
	// TODO complete this: Cf. constructor from robot text file. sensorMap and
	// motorMap could be probably implemented as a RobotRep. property

	// neuralNetwork_->adoptBody(sensorMap, motorMap);

	// TODO then, complete adoptBody()

	return true;

}

std::string RobotRepresentation::generateUniqueIdFromSomeId() {

	std::stringstream ss;
	ss << "myid" << maxid_;

	std::string newUniqueId = ss.str();
	maxid_++;
	return newUniqueId;

}

bool RobotRepresentation::addPartsToMap(
		boost::shared_ptr<PartRepresentation> part) {

	std::string newUniqueId = this->generateUniqueIdFromSomeId();
	part->setId(newUniqueId);
	idToPart_[newUniqueId] = boost::weak_ptr<PartRepresentation>(part);

	for (unsigned int i = 0; i < part->getArity(); i++) {

		if (part->getChild(i)) {
			this->addPartsToMap(part->getChild(i));
		}

	}

	return true;

}

bool RobotRepresentation::duplicateSubTree(const std::string& subtreeRootPartId,
		const std::string& subtreeDestPartId, unsigned int slotId) {

	// find src part and dest part by id
	boost::shared_ptr<PartRepresentation> src =
			idToPart_[subtreeRootPartId].lock();
	boost::shared_ptr<PartRepresentation> dst =
			idToPart_[subtreeDestPartId].lock();

	// If source is root node, then return
	if (src->getId().compare(bodyTree_->getId()) == 0) {
		return false;
	}

	boost::shared_ptr<PartRepresentation> clone = src->cloneSubtree();
	dst->setChild(slotId, clone);

	this->addPartsToMap(clone);

	return true;

}

bool RobotRepresentation::swapSubTrees(const std::string& subtreeRoot1,
		const std::string& subtreeRoot2) {

	// Get roots of the subtrees
	boost::shared_ptr<PartRepresentation> root1 =
			idToPart_[subtreeRoot1].lock();
	boost::shared_ptr<PartRepresentation> root2 =
			idToPart_[subtreeRoot2].lock();

	// Check none of them is the root node
	if (root1->getId().compare(bodyTree_->getId()) == 0
			|| root2->getId().compare(bodyTree_->getId()) == 0) {
		return false;
	}

	// Get parents and slots of each subtree
	PartRepresentation* parentRoot1 = root1->getParent();
	PartRepresentation* parentRoot2 = root2->getParent();

	// Get the slots to which this nodes are connected
	unsigned int slotParentRoot1 = 0;

	for (unsigned int i = 0; i < parentRoot1->getArity(); ++i) {
		if (parentRoot1->getChild(i) != NULL) {
			if (parentRoot1->getChild(i)->getId().compare(parentRoot1->getId())
					== 0) {
				slotParentRoot1 = i;
				break;
			}
		}
	}

	unsigned int slotParentRoot2 = 0;
	for (unsigned int i = 0; i < parentRoot2->getArity(); ++i) {
		if (parentRoot2->getChild(i) != NULL) {
			if (parentRoot2->getChild(i)->getId().compare(parentRoot2->getId())
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

}

bool RobotRepresentation::insertPart(const std::string& parentPartId,
		unsigned int parentPartSlot,
		boost::shared_ptr<PartRepresentation> newPart,
		unsigned int newPartSlot) {

	// Set new ID for the inserted node
	std::string newUniqueId = this->generateUniqueIdFromSomeId();
	newPart->setId(newUniqueId);

	// find dst part by id
	boost::shared_ptr<PartRepresentation> parentPart =
			idToPart_[parentPartId].lock();
	boost::shared_ptr<PartRepresentation> childPart = parentPart->getChild(
			parentPartSlot);

	// Check the arity of the new part
	if (parentPart->getChild(parentPartSlot) != NULL
			&& newPart->getArity() < 1) {
		return false;
	}

	parentPart->setChild(parentPartSlot, newPart);
	newPart->setChild(newPartSlot, childPart);

	// Add to the map
	idToPart_[newUniqueId] = boost::weak_ptr<PartRepresentation>(newPart);

	return true;

}

bool RobotRepresentation::removePart(const std::string& partId) {

	boost::shared_ptr<PartRepresentation> nodeToRemove =
			idToPart_[partId].lock();

	// If root node, return
	if (nodeToRemove->getId().compare(bodyTree_->getId()) == 0) {
		return false;
	}

	// Get parent of node to be removed
	PartRepresentation* parent = nodeToRemove->getParent();

	if (parent == NULL) {
		return false;
	}

	unsigned int nFreeSlots = parent->getFreeSlots().size();
	if (nFreeSlots < nodeToRemove->numDescendants()) {
		return false;
	}

	unsigned int indx = 0;
	for (unsigned int i = 0; i < parent->getArity(); i++) {

		if (parent->getChild(i) == NULL) {

			// this never will be infinite cycle given that nFreeSlots were computed correctly
			while (nodeToRemove->getChild(indx) == NULL) {
				indx = indx + 1;
			}

			parent->setChild(i, nodeToRemove->getChild(indx));

			if (indx > nodeToRemove->numDescendants()) {
				break;
			}
		}
	}

	return true;
}

bool operator >(const RobotRepresentation &a, const RobotRepresentation &b) {
	return a.getFitness() > b.getFitness();
}

}

#endif /* usage of fake robot representation */
