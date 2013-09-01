/*
 * @(#) NeuralNetworkRepresentation.cpp   1.0   Aug 28, 2013
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

#include "evolution/representation/NeuralNetworkRepresentation.h"
#include <queue>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace robogen {

NeuralNetworkRepresentationException::NeuralNetworkRepresentationException(
		const std::string& w) : std::runtime_error(w){}

NeuralNetworkRepresentation::NeuralNetworkRepresentation(
		std::map<std::string,int> &sensorParts,
		std::map<std::string,int> &motorParts){
	// generate neurons from sensor body parts
	for (std::map<std::string,int>::iterator it = sensorParts.begin();
			it != sensorParts.end(); it++){
		for (int i=0; i<it->second; i++){
			std::stringstream id;
			id << it->first << "-" << i;
			neurons_[std::pair<std::string,int>(it->first,i)] =
					boost::shared_ptr<NeuronRepresentation>(
							new NeuronRepresentation(id.str(),"input",0.,
									it->first,i));
		}
	}
	// generate neurons from motor body parts
	for (std::map<std::string,int>::iterator it = motorParts.begin();
			it != motorParts.end(); it++){
		for (int i=0; i<it->second; i++){
			std::stringstream id;
			id << it->first << "-" << i;
			neurons_[std::pair<std::string,int>(it->first,i)] =
					boost::shared_ptr<NeuronRepresentation>(
							new NeuronRepresentation(id.str(),"output",0.,
									it->first,i));
		}
	}
}

NeuralNetworkRepresentation::~NeuralNetworkRepresentation() {
}

void NeuralNetworkRepresentation::setWeight(std::string from, int fromIoId,
		std::string to, int toIoId, double value){
	std::map<std::pair<std::string,int>, boost::shared_ptr<NeuronRepresentation>
	>::iterator fi = neurons_.find(std::pair<std::string, int>(from, fromIoId));
	std::map<std::pair<std::string,int>, boost::shared_ptr<NeuronRepresentation>
	>::iterator ti = neurons_.find(std::pair<std::string, int>(to, toIoId));
	if (fi==neurons_.end()){
		std::stringstream ss;
		ss << "Specified weight input io id pair " << from << " " << fromIoId <<
				" is not in the body cache of the neural network."\
				"Candidates are:\n";
		for (fi = neurons_.begin(); fi != neurons_.end(); ++fi){
			ss << "(" << fi->first.first << " " << fi->first.second << "), ";
		}
		throw NeuralNetworkRepresentationException(ss.str());
	}
	if (ti==neurons_.end()){
		std::stringstream ss;
		ss << "Specified weight output io id pair " << to << " " << toIoId
				<< " is not in the body cache of the neural network."\
				"Candidates are:\n";
		for (ti = neurons_.begin(); ti != neurons_.end(); ++ti){
			ss << "(" << ti->first.first << " " << ti->first.second << "), ";
		}
		throw NeuralNetworkRepresentationException(ss.str());
	}
	if (ti->second->isInput()){
		std::stringstream ss;
		ss << "Attempted to make connection to input layer neuron " << to <<
				" " << toIoId;
		throw NeuralNetworkRepresentationException(ss.str());
	}
	weights_[std::pair<std::string, std::string>(fi->second->getId(),
			ti->second->getId())] = value;
}

void NeuralNetworkRepresentation::setBias(std::string bodyPart, int ioId,
		double value){
	std::map<std::pair<std::string,int>, boost::shared_ptr<NeuronRepresentation>
	>::iterator it = neurons_.find(std::pair<std::string, int>(bodyPart, ioId));
	if (it==neurons_.end()){
		std::stringstream ss;
		ss << "Specified weight output io id pair " << bodyPart << " " << ioId
				<< " is not in the body cache of the neural network."\
				"Candidates are:\n";
		for (it = neurons_.begin(); it != neurons_.end(); ++it){
			ss << "(" << it->first.first << ", " << it->first.second << "), ";
		}
		throw NeuralNetworkRepresentationException(ss.str());
	}
	if (it->second->isInput()){
		std::stringstream ss;
		ss << "Attempted to assign bias to input layer neuron " << bodyPart <<
				" " << ioId;
		throw NeuralNetworkRepresentationException(ss.str());
	}
	it->second->setBias(value);
}

robogenMessage::Brain NeuralNetworkRepresentation::serialize(){
	robogenMessage::Brain serialization;
	// neurons
	for (std::map<std::pair<std::string,int>,
			boost::shared_ptr<NeuronRepresentation>	>::iterator it =
					neurons_.begin(); it !=neurons_.end(); ++it){
		robogenMessage::Neuron *neuron = serialization.add_neuron();
		*neuron = it->second->serialize();
	}
	// connections
	for (std::map<std::pair<std::string, std::string>, double>::iterator it =
			weights_.begin(); it!=weights_.end(); it++){
		robogenMessage::NeuralConnection *connection =
				serialization.add_connection();
		// required string src = 1;
		connection->set_src(it->first.first);
		// required string dest = 2;
		connection->set_dest(it->first.second);
		// required float weight = 3;
		connection->set_weight(it->second);
	}
	return serialization;
}

} /* namespace robogen */
