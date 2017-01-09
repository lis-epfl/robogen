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
		evaluated_(false), fitness_(0) {
			//I should initialize maxId to 1000
}

RobotRepresentation::RobotRepresentation(const RobotRepresentation &r) {

	this->robotMorph_.reset(new SubRobotRepresentation(*(r.robotMorph_.get())));

	//This was leading to issues, copy also fitness values
	this->fitness_ = r.fitness_;
	this->evaluated_ = r.evaluated_;

	// We copy the AXIOM to the grammar
	this->grammar_.reset(new Grammar(r.grammar_->getAxiom(), r.grammar_->getAllRules()));
}

RobotRepresentation &RobotRepresentation::operator=(
		const RobotRepresentation &r) {
	*this->robotMorph_ = *(r.robotMorph_);
	*this->grammar_ = *(r.grammar_);
	fitness_ = r.fitness_;
	evaluated_ = r.evaluated_;
	return *this;
}

void RobotRepresentation::asyncEvaluateResult(double fitness) {
	fitness_ = fitness;
	evaluated_ = true;
}

bool RobotRepresentation::init() {
	this->robotMorph_.reset(new SubRobotRepresentation());
	this->robotMorph_->init();

	this->grammar_.reset(new Grammar(this->robotMorph_));
	return true;
}

bool RobotRepresentation::init(std::string robotTextFile) {

	this->robotMorph_.reset(new SubRobotRepresentation());

	this->robotMorph_->init(robotTextFile);

	//In this case, we make the axiom the content of the textfile
	this->grammar_.reset(new Grammar(this->robotMorph_));

	return true;
}

robogenMessage::Robot RobotRepresentation::serialize() const {
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
	this->robotMorph_->rebuildBodyMap();
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
	return this->robotMorph_->trimBodyAt(id,printErrors);
}

bool RobotRepresentation::duplicateSubTree(const std::string& subtreeRootPartId,
		const std::string& subtreeDestPartId, unsigned int slotId,
		bool printErrors) {

	return this->robotMorph_->duplicateSubTree(subtreeRootPartId, subtreeDestPartId, slotId, printErrors);
}

bool RobotRepresentation::swapSubTrees(const std::string& subtreeRoot1,
		const std::string& subtreeRoot2, bool printErrors) {

	return this->robotMorph_->swapSubTrees(subtreeRoot1, subtreeRoot2, printErrors);
}

bool RobotRepresentation::insertPart(const std::string& parentPartId,
		unsigned int parentPartSlot,
		boost::shared_ptr<PartRepresentation> newPart,
		unsigned int newPartSlot,
		unsigned int motorNeuronType, bool printErrors) {

	return this->robotMorph_->insertPart(parentPartId, parentPartSlot, newPart, newPartSlot, motorNeuronType, printErrors);

}

bool RobotRepresentation::removePart(const std::string& partId,
		bool printErrors) {

	return this->robotMorph_->removePart(partId, printErrors);
}

bool RobotRepresentation::setChildPosition(const std::string& partId,  
			std::vector<boost::shared_ptr<PartRepresentation>> children,
			bool printErrors){

	return this->robotMorph_->setChildPosition(partId, children, printErrors);
}

bool RobotRepresentation::check() {

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

bool RobotRepresentation::buildFromGrammar(void){
	this->robotMorph_ = this->grammar_->buildTree();
	return !this->grammar_->lastBuildFailed();
}

}

#endif /* usage of fake robot representation */
