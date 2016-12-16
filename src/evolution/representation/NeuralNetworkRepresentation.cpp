/*
 * @(#) NeuralNetworkRepresentation.cpp   1.0   Aug 28, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2015 Titus Cieslewski, Joshua Auerbach
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
#include <algorithm>
#include <utility>

namespace robogen {

NeuralNetworkRepresentation &NeuralNetworkRepresentation::operator =(
		const NeuralNetworkRepresentation &original) {
	// deep copy neurons
	neurons_.clear();
	for (NeuronMap::const_iterator it = original.neurons_.begin();
			it != original.neurons_.end(); ++it) {
		neurons_[it->first] = boost::shared_ptr<NeuronRepresentation>(
				new NeuronRepresentation(*(it->second.get())));
	}
	// replace weight map
	weights_ = original.weights_;
	return *this;
}

NeuralNetworkRepresentation::NeuralNetworkRepresentation(
		const NeuralNetworkRepresentation &original) {
	// deep copy neurons
	neurons_.clear();
	for (NeuronMap::const_iterator it = original.neurons_.begin();
			it != original.neurons_.end(); ++it) {
		neurons_[it->first] = boost::shared_ptr<NeuronRepresentation>(
				new NeuronRepresentation(*(it->second.get())));
	}
	// replace weight map
	weights_ = original.weights_;
}

NeuralNetworkRepresentation::NeuralNetworkRepresentation(
		std::map<std::string, int> &sensorParts,
		std::map<std::string, int> &motorParts) {
	// generate neurons from sensor body parts
	for (std::map<std::string, int>::iterator it = sensorParts.begin();
			it != sensorParts.end(); it++) {
		for (int i = 0; i < it->second; i++) {
			insertNeuron(ioPair(it->first, i), NeuronRepresentation::INPUT,
					NeuronRepresentation::SIMPLE);
		}
	}
	// generate neurons from motor body parts
	for (std::map<std::string, int>::iterator it = motorParts.begin();
			it != motorParts.end(); it++) {
		for (int i = 0; i < it->second; i++) {
			insertNeuron(ioPair(it->first, i), NeuronRepresentation::OUTPUT,
					NeuronRepresentation::SIGMOID);
		}
	}
}

NeuralNetworkRepresentation::~NeuralNetworkRepresentation() {
}

bool NeuralNetworkRepresentation::setWeight(std::string from, int fromIoId,
		std::string to, int toIoId, double value) {
	return setWeight(ioPair(from, fromIoId), ioPair(to, toIoId), value);
}

bool NeuralNetworkRepresentation::setWeight(ioPair fromPair, ioPair toPair,
		double value) {
	NeuronMap::iterator fi =
			neurons_.find(fromPair);
	NeuronMap::iterator ti =
			neurons_.find(toPair);
	if (fi == neurons_.end()) {
		std::cout << "Specified weight input io id pair " << fromPair.first
				<< " " << fromPair.second
				<< " is not in the body cache of the neural network."
						"Candidates are:" << std::endl;
		for (fi = neurons_.begin(); fi != neurons_.end(); ++fi) {
			std::cout << "(" << fi->first.first << " " << fi->first.second
					<< "), ";
		}
		std::cout << std::endl;
		return false;
	}
	if (ti == neurons_.end()) {
		std::cout << "Specified weight output io id pair " << toPair.first
				<< " " << toPair.second
				<< " is not in the body cache of the neural network."
						"Candidates are:" << std::endl;
		for (ti = neurons_.begin(); ti != neurons_.end(); ++ti) {
			std::cout << "(" << ti->first.first << " " << ti->first.second
					<< "), ";
		}
		std::cout << std::endl;
		return false;
	}
	if (ti->second->isInput()) {
		std::cout << "Attempted to make connection to input layer neuron " <<
				toPair.first << " " << toPair.second << std::endl;
		return false;
	}
	weights_[std::pair<std::string, std::string>(fi->second->getId(),
			ti->second->getId())] = value;
	return true;
}

bool NeuralNetworkRepresentation::setParams(std::string bodyPart, int ioId,
		unsigned int type, std::vector<double> params) {
	NeuronMap::iterator it = neurons_.find(ioPair(bodyPart, ioId));
	if (it == neurons_.end()) {
		std::cout << "Specified weight output io id pair " << bodyPart << " "
				<< ioId << " is not in the body cache of the neural network."
						"Candidates are:" << std::endl;
		for (it = neurons_.begin(); it != neurons_.end(); ++it) {
			std::cout << "(" << it->first.first << ", " << it->first.second
					<< "), ";
		}
		std::cout << std::endl;
		return false;
	}
	if (it->second->isInput()) {
		std::cout << "Attempted to assign params to input layer neuron "
				<< bodyPart << " " << ioId << std::endl;
		return false;
	}
	if(type == NeuronRepresentation::SIMPLE) {
		// should not really be used, but will leave as an option
		if(params.size() != 1) {
			std::cout << "Invalid number of params for simple neuron type.";
			std::cout << " Received "<< params.size() << ", but expected 1.";
			std::cout << std::endl;
			return false;
		}
	} else if(type == NeuronRepresentation::SIGMOID) {
		if(params.size() != 1) {
			std::cout << "Invalid number of params for sigmoid neuron type.";
			std::cout << " Received "<< params.size() << ", but expected 1.";
			std::cout << std::endl;
			return false;
		}
	} else if(type == NeuronRepresentation::CTRNN_SIGMOID) {
		if(params.size() != 2) {
			std::cout << "Invalid number of params for CTRNN sigmoid neuron type.";
			std::cout << " Received "<< params.size() << ", but expected 2.";
			std::cout << std::endl;
			return false;
		}
	} else if(type == NeuronRepresentation::OSCILLATOR) {
		if (params.size() != 3) {
			std::cout << "Invalid number of params for oscillator neuron type.";
			std::cout << " Received "<< params.size() << ", but expected 3.";
			std::cout << std::endl;
			return false;
		}
		std::cout << "neuron " << bodyPart << " " << ioId <<
				" set to be oscillator.  Will remove incoming connections" <<
				std::endl;
		removeIncomingConnections(it->second);

	} else {
		std::cout << "Invalid neuron type "	<< type << std::endl;
		return false;
	}
	it->second->setParams(type, params);
	return true;
}

void NeuralNetworkRepresentation::getGenome(std::vector<double*> &weights,
		std::vector<unsigned int> &types,
		std::vector<double*> &params) {
	// clean up
	weights.clear();
	params.clear();
	types.clear();
	// provide weights
	for (WeightMap::iterator it = weights_.begin(); it != weights_.end();
			++it) {
		weights.push_back(&it->second);
	}
	// provide biases, only include those for applicable neurons
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
			++it) {
		if (!it->second->isInput()) {
			std::vector<double*> neuronParams;
			it->second->getParamsPointers(neuronParams);
			params.insert(params.end(), neuronParams.begin(),
					neuronParams.end());
			types.push_back(it->second->getType());
		}
	}
}

std::string NeuralNetworkRepresentation::insertNeuron(ioPair identification,
		unsigned int layer, unsigned int type) {
	boost::shared_ptr<NeuronRepresentation> neuron = boost::shared_ptr<
			NeuronRepresentation>(
			new NeuronRepresentation(identification, layer, type));
	// insert into map
	if (neurons_.count(identification)) {
		std::cout << "ATTENTION: attempting to insert a neuron with id " <<
				identification.first << "-" << identification.second <<
				", which already exists" << std::endl;
	}
	neurons_[identification] = neuron;
	// generate weights
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
			++it) {
		// generate incoming
		if (!neuron->isInput() &&
				(neuron->getType() != NeuronRepresentation::OSCILLATOR)) {
			weights_[StringPair(it->second->getId(), neuron->getId())] = 0.;
		}
		// generate outgoing (no need to worry about double declaration of the
		// recursion, as we deal with a map!)
		if (!it->second->isInput() &&
				(it->second->getType() != NeuronRepresentation::OSCILLATOR))
			weights_[StringPair(neuron->getId(), it->second->getId())] = 0.;
	}
	return neuron->getId();
}

void NeuralNetworkRepresentation::cloneNeurons(std::string oldPartId,
		std::string newPartId, std::map<std::string, std::string> &oldNew) {
	std::vector<boost::weak_ptr<NeuronRepresentation> > neurons =
			getBodyPartNeurons(oldPartId);
	for (unsigned int i = 0; i < neurons.size(); ++i) {
		// remove all weights of the neuron
		boost::shared_ptr<NeuronRepresentation> neuron = neurons[i].lock();
		oldNew[neuron->getId()] = insertNeuron(
				ioPair(newPartId, neuron->getIoPair().second),
				neuron->getLayer(), neuron->getType());
	}
}

void NeuralNetworkRepresentation::generateCloneWeights(
		std::map<std::string, std::string> &oldNew) {
	typedef std::map<std::string, std::string> MyMap;

	// for every neuron in the cloned tree
	for (MyMap::iterator itNeuron = oldNew.begin(); itNeuron != oldNew.end(); ++itNeuron) {

		std::string oldRon = itNeuron->first;
		std::string newRon = itNeuron->second;

		// for every weight
		for (WeightMap::iterator it = weights_.begin(); it != weights_.end();
				++it) {
			// if outgoing
			if (it->first.first.compare(oldRon) == 0) {
				// if destination neuron was in original subtree
				if (oldNew.find(it->first.second) != oldNew.end()) {
					weights_[StringPair(newRon,
							oldNew.find(it->first.second)->second)] =
							it->second;
				} else {
					weights_[StringPair(newRon, it->first.second)] = it->second;
				}
			}
			// if incoming
			if (it->first.second.compare(oldRon) == 0) {
				// if destination neuron was in original subtree
				if (oldNew.find(it->first.first) != oldNew.end()) {
					weights_[StringPair(oldNew.find(it->first.first)->second,
							newRon)] = it->second;
				} else {
					weights_[StringPair(it->first.first, newRon)] = it->second;
				}
			}

		}
	}
}

void NeuralNetworkRepresentation::removeIncomingConnections(
		boost::shared_ptr<NeuronRepresentation> neuron) {
	// remove all incoming weights of the neuron
	WeightMap::iterator it = weights_.begin();
	while (it != weights_.end()) {
	   if (it->first.second.compare(neuron->getId()) == 0) {
		   //std::cout << it->first.first << " -> " << it->first.second << std::endl;
		   weights_.erase(it++);
	   } else {
		  it++;
	   }
	}
}
void NeuralNetworkRepresentation::removeOutgoingConnections(
		boost::shared_ptr<NeuronRepresentation> neuron) {
	// remove all outgoing weights of the neuron
	WeightMap::iterator it = weights_.begin();
	while (it != weights_.end()) {
	   if (it->first.first.compare(neuron->getId()) == 0) {
		   //std::cout << it->first.first << " -> " << it->first.second << std::endl;
		   weights_.erase(it++);
	   } else {
		  it++;
	   }
	}
}


void NeuralNetworkRepresentation::removeNeurons(std::string bodyPartId) {
	std::vector<boost::weak_ptr<NeuronRepresentation> > neurons =
			getBodyPartNeurons(bodyPartId);

	for (unsigned int i = 0; i < neurons.size(); ++i) {
		boost::shared_ptr<NeuronRepresentation> neuron = neurons[i].lock();
		assert(neuron);
		removeIncomingConnections(neuron);
		removeOutgoingConnections(neuron);
		// remove the neuron itself
		NeuronMap::iterator toErase = neurons_.find(neuron->getIoPair());
		if( toErase == neurons_.end()) {
			std::cout << "no neurons to remove for " << neuron->getIoPair().first << "-" << neuron->getIoPair().second;
		}
		neurons_.erase(toErase);
	}

}

std::vector<boost::weak_ptr<NeuronRepresentation> >
		NeuralNetworkRepresentation::getBodyPartNeurons(std::string bodyPart) {

	std::vector<boost::weak_ptr<NeuronRepresentation> > ret;

	// go through neurons, check body part id
	int ioId = 0;
	while (neurons_.find(ioPair(bodyPart, ioId)) != neurons_.end()) {
		ret.push_back(
				boost::weak_ptr<NeuronRepresentation>(
						neurons_[ioPair(bodyPart, ioId)]));
		ioId++;
	}
	return ret;
}

bool NeuralNetworkRepresentation::connectionExists(std::string from,
		std::string to) {
	return (weights_.count(StringPair(from, to)) > 0);
}


/*
bool NeuralNetworkRepresentation::getLinearRepresentation(
		std::vector<ioPair> &inputs, std::vector<ioPair> &outputs,
		std::vector<double> &weights, std::vector<double> &biases) {
	std::vector<boost::shared_ptr<NeuronRepresentation> > inputNeurons;
	std::vector<boost::shared_ptr<NeuronRepresentation> > outputNeurons;
	// basic house holding
	inputs.clear();
	outputs.clear();
	weights.clear();
	biases.clear();
	// fill inputs and outputs
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
			++it) {
		if (it->second->isInput()) {
			inputs.push_back(it->first);
			inputNeurons.push_back(it->second);
		} else {
			outputs.push_back(it->first); // ATTENTION: DEPENDS ON ANN ARCH.
			outputNeurons.push_back(it->second);
		}
	}
	// fill weight vector in proper order
	for (unsigned int i = 0; i < inputs.size(); ++i) {
		for (unsigned int j = 0; j < outputs.size(); ++j) {
			WeightMap::iterator it = weights_.find(
					std::pair<std::string, std::string>(
							inputNeurons[i]->getId(),
							outputNeurons[j]->getId()));
			if (it == weights_.end()) {
				std::cout << "Can't get weight from "
						<< inputNeurons[i]->getId() << " to "
						<< outputNeurons[j]->getId()
						<< "\nIs the weight map filled properly?" << std::endl;
				return false;
			}
			weights.push_back(it->second);
		}
	}
	for (unsigned int i = 0; i < outputs.size(); ++i) {
		for (unsigned int j = 0; j < outputs.size(); ++j) {
			WeightMap::iterator it = weights_.find(
					std::pair<std::string, std::string>(
							outputNeurons[i]->getId(),
							outputNeurons[j]->getId()));
			if (it == weights_.end()) {
				std::cout << "Can't get weight from "
						<< outputNeurons[i]->getId() << " to "
						<< outputNeurons[j]->getId()
						<< "\nIs the weight map filled properly?" << std::endl;
				return false;
			}
			weights.push_back(it->second);
		}
		// fill bias vector properly
		biases.push_back(*outputNeurons[i]->getBiasPointer());
	}
	return true;
}
*/
robogenMessage::Brain NeuralNetworkRepresentation::serialize() {
	robogenMessage::Brain serialization;

	// neurons
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
			++it) {
		robogenMessage::Neuron *neuron = serialization.add_neuron();
		*neuron = it->second->serialize();
	}
	// connections
	for (std::map<std::pair<std::string, std::string>, double>::iterator it =
			weights_.begin(); it != weights_.end(); it++) {
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

std::string NeuralNetworkRepresentation::toString() {

	std::stringstream str;
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
			++it) {

		str << "Neuron : " << it->second->getId();
		if (it->second->isInput()) {
			str << " <- ";
		} else {
			str << " -> ";
		}
		str << it->second->getIoPair().first << ", "
				<< it->second->getIoPair().second << std::endl;

	}

	// connections
	for (std::map<std::pair<std::string, std::string>, double>::iterator it =
			weights_.begin(); it != weights_.end(); it++) {
		str << it->first.first << " --> " << it->first.second << " (" << it->second << ")";
	}

	return str.str();
}

int NeuralNetworkRepresentation::getNumInputs() {
	int numInputs = 0;
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
				++it) {
		if(it->second->getLayer() == NeuronRepresentation::INPUT)
			numInputs++;
	}
	return numInputs;
}

int NeuralNetworkRepresentation::getNumHidden() {
	int numHidden = 0;
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
				++it) {
		if(it->second->getLayer() == NeuronRepresentation::HIDDEN)
			numHidden++;
	}
	return numHidden;
}

int NeuralNetworkRepresentation::getNumOutputs() {
	int numOutputs = 0;
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
				++it) {
		if(it->second->getLayer() == NeuronRepresentation::OUTPUT)
			numOutputs++;
	}
	return numOutputs;
}

} /* namespace robogen */
