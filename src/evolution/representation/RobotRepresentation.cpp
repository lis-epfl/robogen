/*
 * @(#) RobotRepresentation.cpp   1.0   Aug 28, 2013
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

#include "evolution/representation/RobotRepresentation.h"
#ifndef FAKEROBOTREPRESENTATION_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>
#include <boost/regex.hpp>
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/parts/CoreComponentRepresentation.h"
#include "evolution/representation/parts/LightSensorRepresentation.h"
#include "evolution/representation/parts/TouchSensorRepresentation.h"
#include "utils/network/ProtobufPacket.h"

namespace robogen{

/**
 * Helper function for decoding a part line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadPartLine(std::ifstream &file, int &indent, int &slot,
		char &type, std::string &id, int &orientation,
		std::vector<double> &params){
	// match (0 or more tabs)(digit) (type) (id) (orientation) (parameters)
	static const boost::regex rx(
			"^(\\t*)(\\d) ([A-Z]) ([^\\s]+) (\\d)([ \\d\\.-]*)$");
	boost::cmatch match;
	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)){
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
		while (ss >> param){
			params.push_back(param);
		}
		return true;
	}
	else{
		// throw exception if poor formatting, i.e. line not empty
		static const boost::regex spacex("^\\s*$");
		if (!boost::regex_match(line.c_str(),spacex)){
			std::stringstream ss;
			ss << "Error reading body part from text file. Received:\n" <<
					line << "\nbut expected format:\n" <<
					"<0 or more tabs><slot index digit> <part type character> "\
					"<part id string> <orientation digit> <evt. parameters>";
			throw RobotRepresentationException(ss.str());
		}
		return false;
	}
}

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadWeightLine(std::ifstream &file, std::string &from,
		int &fromIoId, std::string &to, int &toIoId, double &value){
	// TODO with this regex, not sure if weight value is read
	static const boost::regex rx(
			"^([^\\s]+) (\\d+) ([^\\s]+) (\\d+) (-?\\d*\\.?\\d*)$");
	boost::cmatch match;
	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)){
		// match[0]:whole string, match[1]:from, match[2]:from IO id,
		// match[3]:to, match[4]:to IO id, match[5]:value
		from.assign(match[1]);
		fromIoId = std::atoi(match[2].first);
		to.assign(match[3]);
		toIoId = std::atoi(match[4].first);
		value = std::atof(match[5].first);
		return true;
	}
	else{
		// throw exception if poor formatting, i.e. line not empty
		static const boost::regex spacex("^\\s*$");
		if (!boost::regex_match(line.c_str(),spacex)){
			std::stringstream ss;
			ss << "Error reading brain weight from text file. Received:\n" <<
					line << "\nbut expected format:\n" <<
					"<source part id string> <source part io id> "\
					"<destination part id string> <destination part io id> "\
					"<weight>";
			throw RobotRepresentationException(ss.str());
		}
		return false;
	}
}

/**
 * Helper function for decoding a weight line of a robot text file.
 * @return true if successful read
 */
bool robotTextFileReadBiasLine(std::ifstream &file, std::string &node,
		int &ioId, double &value){
	// TODO with this regex, not sure if bias value is read
	static const boost::regex rx("^([^\\s]+) (\\d+) (-?\\d*\\.?\\d*)$");
	boost::cmatch match;
	std::string line;
	std::getline(file, line);
	if (boost::regex_match(line.c_str(), match, rx)){
		// match[0]:whole string, match[1]:node, match[2]:ioId, match[3]:value
		node.assign(match[1]);
		ioId = std::atoi(match[2].first);
		value = std::atof(match[3].first);
		return true;
	}
	else{
		// throw exception if poor formatting, i.e. line not empty
		static const boost::regex spacex("^\\s*$");
		if (!boost::regex_match(line.c_str(),spacex)){
			std::stringstream ss;
			ss << "Error reading brain bias from text file. Received:\n" <<
					line << "\nbut expected format:\n" <<
					"<part id string> <part io id> "\
					"<bias>";
			throw RobotRepresentationException(ss.str());
		}
		return false;
	}
}

RobotRepresentationException::RobotRepresentationException(
		const std::string& w): std::runtime_error(w){}

RobotRepresentation::RobotRepresentation(const RobotRepresentation &r){
	// we need to handle bodyTree_, neuralNetwork_ and reservedIds_
	// for the brainevolver, we could theoretically keep the bodyTree_ pointing
	// to the same body, but that would be easy to miss when resuming to body
	// evolution, so we'll just do proper copying right away

	// special treatment for base-pointed instances of derived classes as are
	// our body parts
	bodyTree_ = r.bodyTree_->cloneSubtree();
	// neural network pointer needs to be reset to a copy-constructed instance
	neuralNetwork_.reset(new NeuralNetworkRepresentation(
			*(r.neuralNetwork_.get())));
	// rebuild ID to part map
	idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(bodyTree_);
	while (!q.empty()){
		boost::shared_ptr<PartRepresentation> cur = q.front(); q.pop();
		idToPart_[cur->getId()] =  boost::weak_ptr<PartRepresentation>(cur);
		for (int i=1; i<=cur->getArity(); ++i){
			if (cur->getChild(i)){
				q.push(cur->getChild(i));
			}
		}
	}
	// fitness and associated flag are same
	fitness_ = r.fitness_;
	evaluated_ = r.evaluated_;
}

RobotRepresentation &RobotRepresentation::operator=(
		const RobotRepresentation &r){
	// same as copy constructor, see there for explanations
	bodyTree_ = r.bodyTree_->cloneSubtree();
	neuralNetwork_.reset(new NeuralNetworkRepresentation(
			*(r.neuralNetwork_.get())));
	// rebuild ID to part map
	idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(bodyTree_);
	while (!q.empty()){
		boost::shared_ptr<PartRepresentation> cur = q.front(); q.pop();
		idToPart_[cur->getId()] =  boost::weak_ptr<PartRepresentation>(cur);
		for (int i=1; i<=cur->getArity(); ++i){
			if (cur->getChild(i)){
				q.push(cur->getChild(i));
			}
		}
	}
	fitness_ = r.fitness_;
	evaluated_ = r.evaluated_;
	return *this;
}

RobotRepresentation::RobotRepresentation(std::string robotTextFile){
	// open file
	std::ifstream file;
	file.open(robotTextFile.c_str());
	if (!file.is_open()){
		std::stringstream ss;
		ss << "Could not open robot text file " << robotTextFile;
		throw RobotRepresentationException(ss.str());
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
			params) || indent){
		throw RobotRepresentationException("Robot text file contains no or"\
				" poorly formatted root node");
	}
	current = PartRepresentation::create(type,id,orientation,params);
	bodyTree_ = current;
	idToPart_[id] = boost::weak_ptr<PartRepresentation>(current);

	// process other body parts
	while(robotTextFileReadPartLine(file, indent, slot, type, id, orientation,
			params)){
		if (!indent){
			throw RobotRepresentationException("Attempt to create "\
					"multiple root nodes!");
		}
		// indentation: Adding children to current
		if (indent>(parentStack.size())){
			parentStack.push(current);
		}
		// indentation: done adding children to top of parent stack
		for (;indent<(parentStack.size());){
			parentStack.pop();
		}
		current = PartRepresentation::create(type,id,orientation,params);
		if (parentStack.top()->getChild(slot)){
			std::stringstream ss;
			ss << "Attempt to overwrite child " <<
					parentStack.top()->getChild(slot)->getId() << " of " <<
					parentStack.top()->getId() << " with " <<
					current->getId();
			throw RobotRepresentationException(ss.str());
		}
		parentStack.top()->setChild(slot, current);
		idToPart_[id] = boost::weak_ptr<PartRepresentation>(current);
	}

	// process brain
	std::string from, to;
	int fromIoId, toIoId;
	double value;
	// create neural network: create map from body id to ioId for all sensor and
	// motor body parts
	std::map<std::string, int> sensorMap, motorMap;
	for(std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator
			it = idToPart_.begin(); it != idToPart_.end(); it++){
		// omitting weak pointer checks, as this really shouldn't go wrong here!
		if (it->second.lock()->getMotors().size()){
			motorMap[it->first] = it->second.lock()->getMotors().size();
		}
		if (it->second.lock()->getSensors().size()){
			sensorMap[it->first] = it->second.lock()->getSensors().size();
		}
	}
	neuralNetwork_.reset(new NeuralNetworkRepresentation(sensorMap, motorMap));
	// weights
	while (robotTextFileReadWeightLine(file, from, fromIoId, to, toIoId,
			value)){
		neuralNetwork_->setWeight(from, fromIoId, to, toIoId, value);
	}
	// biases
	while (robotTextFileReadBiasLine(file, to, toIoId, value)){
		neuralNetwork_->setBias(to, toIoId, value);
	}
	file.close();
}

int RobotRepresentation::getSensorType(const std::string &id){
	IdPartMap::iterator it = idToPart_.find(id);
	// find part in map
	if (it == idToPart_.end()){
		std::stringstream ss;
		ss << "getSensorType supplied with an id \"" << id
				<< "\", which is not found in the body part map.";
		throw RobotRepresentationException(ss.str());
	}
	// find out sensor type
	if (boost::dynamic_pointer_cast<LightSensorRepresentation>(
			it->second.lock())){
		return LIGHT_SENSOR;
	}
	else if(boost::dynamic_pointer_cast<TouchSensorRepresentation>(
			it->second.lock())){
		return TOUCH_SENSOR;
	}
	else if(boost::dynamic_pointer_cast<CoreComponentRepresentation>(
			it->second.lock())){
		return IMU;
	}
	else{
		std::stringstream ss;
		ss << "getSensorType: part with id \"" << id
				<< "\", is not registered as sensor.";
		throw RobotRepresentationException(ss.str());
	}
}

robogenMessage::Robot RobotRepresentation::serialize() const{
	robogenMessage::Robot message;
	// id - this can probably be removed
	message.set_id(1);
	// body
	bodyTree_->addSubtreeToBodyMessage(message.mutable_body(), true);
	// brain
	*(message.mutable_brain()) = neuralNetwork_->serialize();
	// configuration - void - IMO this does not belong here from an OO perspect.
	// being set by the evolution engine when sending to the simulator
	message.set_configuration("");
	return message;
}

void RobotRepresentation::randomizeBrain(boost::random::mt19937	&rng){
	neuralNetwork_->initializeRandomly(rng);
}

void RobotRepresentation::getBrainGenome(std::vector<double*> &weights,
		std::vector<double*> &biases){
	neuralNetwork_->getGenome(weights, biases);
}

boost::shared_ptr<NeuralNetworkRepresentation> RobotRepresentation::getBrain()
const{
	return neuralNetwork_;
}

const RobotRepresentation::IdPartMap &RobotRepresentation::getBody() const{
	return idToPart_;
}

void RobotRepresentation::evaluate(TcpSocket *socket,
		std::string &simulatorConfFile){
	// 1. Prepare message to simulator
	boost::shared_ptr<robogenMessage::Robot> rsp =
			boost::shared_ptr<robogenMessage::Robot>(
					new robogenMessage::Robot(serialize()));
	rsp->set_configuration(simulatorConfFile);
	ProtobufPacket<robogenMessage::Robot>	robotPacket(rsp);
	std::vector<unsigned char> forgedMessagePacket;
	robotPacket.forge(forgedMessagePacket);

	// 2. send message to simulator
	socket->write(forgedMessagePacket);

	// 3. receive message from simulator
	ProtobufPacket<robogenMessage::EvaluationResult> resultPacket(
			boost::shared_ptr<robogenMessage::EvaluationResult>(
					new robogenMessage::EvaluationResult()) );
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
	if(!resultPacket.getMessage()->has_fitness()) {
		std::cerr << "Fitness field not set by Simulator!!!" <<
				std::endl;
		exit(EXIT_FAILURE);
	}
	else {
		fitness_ = resultPacket.getMessage()->fitness();
		evaluated_ = true;
	}

}

double RobotRepresentation::getFitness() const{
	return fitness_;
}

bool RobotRepresentation::isEvaluated() const{
	return evaluated_;
}

void RobotRepresentation::setDirty(){
	evaluated_ = false;
}

bool RobotRepresentation::trimBodyAt(std::string id){
	// thanks to shared pointer magic, we only need to reset the shared pointer
	// to the indicated body part
	PartRepresentation *parent = idToPart_[id].lock()->getParent();
	int position = idToPart_[id].lock()->getPosition();
	if (!parent){
		std::cout << "Trying to remove root body part!" << std::endl;
		return false;
	}
	std::cout << "Has references: " << idToPart_[id].lock().use_count() << std::endl;
	parent->setChild(position, boost::shared_ptr<PartRepresentation>());
	if (!parent->getChild(position)){
		std::cout << "Successfully removed" << std:: endl;
	}
	// need to update the id to body part map! Easily done with weak pointers
	for (IdPartMap::iterator it = idToPart_.begin(); it != idToPart_.end();){
		if (!it->second.lock()){
			idToPart_.erase(it++);
			std::cout << "Had a part to erase! " << parent << std::endl;
		}
		else ++it;
	}
	// TODO complete this: Cf. constructor from robot text file. sensorMap and
	// motorMap could be probably implemented as a RobotRep. property

	// neuralNetwork_->adoptBody(sensorMap, motorMap);

	// TODO then, complete adoptBody()

	return true;
}

bool operator >(const RobotRepresentation &a, const RobotRepresentation &b){
	if (!a.isEvaluated() || !b.isEvaluated()){
		throw RobotRepresentationException("Operator >: Trying to compare "\
				"non-evaluated robots!");
	}
	return a.getFitness()>b.getFitness();
}

}

#endif /* usage of fake robot representation */
