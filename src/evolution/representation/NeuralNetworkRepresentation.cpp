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
#include <algorithm>
#include <utility>

namespace robogen {

NeuralNetworkRepresentation &NeuralNetworkRepresentation::operator =(
		const NeuralNetworkRepresentation &original) {
	// deep copy neurons
	neurons_.clear();
	for (std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::const_iterator it =
			original.neurons_.begin(); it != original.neurons_.end(); ++it) {
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
	for (std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::const_iterator it =
			original.neurons_.begin(); it != original.neurons_.end(); ++it) {
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
			insertNeuron(ioPair(it->first, i), false);
		}
	}
	// generate neurons from motor body parts
	for (std::map<std::string, int>::iterator it = motorParts.begin();
			it != motorParts.end(); it++) {
		for (int i = 0; i < it->second; i++) {
			insertNeuron(ioPair(it->first, i), true);
		}
	}
}

NeuralNetworkRepresentation::~NeuralNetworkRepresentation() {
}

void NeuralNetworkRepresentation::initializeRandomly(
		boost::random::mt19937 &rng) {
	// clear all existing weights and biases
	weights_.clear();
	// create random number generator
	boost::random::uniform_real_distribution<double> weightDistrib(0., 1.);
	// motivation: should be able to take any value if inputs 0, too
	boost::random::uniform_real_distribution<double> biasDistrib(-1., 1.);

	// for each neuron
	for (std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::iterator it =
			neurons_.begin(); it != neurons_.end(); it++) {
		// generate random weight to other neuron if valid
		for (std::map<std::pair<std::string, int>,
				boost::shared_ptr<NeuronRepresentation> >::iterator jt =
				neurons_.begin(); jt != neurons_.end(); jt++) {
			// can't create connection to an input neuron
			if (!jt->second->isInput()) {
				weights_[std::pair<std::string, std::string>(
						it->second->getId(), jt->second->getId())] =
						weightDistrib(rng);
			}
		}
		// generate bias, if not input neuron
		if (!it->second->isInput()) {
			it->second->setBias(biasDistrib(rng));
		}
	}
}

bool NeuralNetworkRepresentation::setWeight(std::string from, int fromIoId,
		std::string to, int toIoId, double value) {
	std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::iterator fi =
			neurons_.find(std::pair<std::string, int>(from, fromIoId));
	std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::iterator ti =
			neurons_.find(std::pair<std::string, int>(to, toIoId));
	if (fi == neurons_.end()) {
		std::cout << "Specified weight input io id pair " << from << " "
				<< fromIoId
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
		std::cout << "Specified weight output io id pair " << to << " "
				<< toIoId << " is not in the body cache of the neural network."
						"Candidates are:" << std::endl;
		for (ti = neurons_.begin(); ti != neurons_.end(); ++ti) {
			std::cout << "(" << ti->first.first << " " << ti->first.second
					<< "), ";
		}
		std::cout << std::endl;
		return false;
	}
	if (ti->second->isInput()) {
		std::cout << "Attempted to make connection to input layer neuron " << to
				<< " " << toIoId << std::endl;
		return false;
	}
	weights_[std::pair<std::string, std::string>(fi->second->getId(),
			ti->second->getId())] = value;
	return true;
}

bool NeuralNetworkRepresentation::setBias(std::string bodyPart, int ioId,
		double value) {
	std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::iterator it =
			neurons_.find(std::pair<std::string, int>(bodyPart, ioId));
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
		std::cout << "Attempted to assign bias to input layer neuron "
				<< bodyPart << " " << ioId;
		return false;
	}
	it->second->setBias(value);
	return true;
}

void NeuralNetworkRepresentation::getGenome(std::vector<double*> &weights,
		std::vector<double*> &biases) {
	// clean up
	weights.clear();
	biases.clear();
	// provide weights
	for (WeightMap::iterator it = weights_.begin(); it != weights_.end();
			++it) {
		weights.push_back(&it->second);
	}
	// provide biases
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
			++it) {
		biases.push_back(it->second->getBiasPointer());
	}
}

std::string NeuralNetworkRepresentation::insertNeuron(ioPair identification,
		bool isOutput) {
	boost::shared_ptr<NeuronRepresentation> neuron = boost::shared_ptr<
			NeuronRepresentation>(
			new NeuronRepresentation(identification, isOutput, 0.));
	// insert into map
	neurons_[identification] = neuron;
	// generate weights
	for (NeuronMap::iterator it = neurons_.begin(); it != neurons_.end();
			++it) {
		// generate incoming
		if (isOutput) {
			weights_[StringPair(it->second->getId(), neuron->getId())] = 0.;
		}
		// generate outgoing (no need to worry about double declaration of the
		// recursion, as we deal with a map!)
		if (!it->second->isInput())
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
				!neuron->isInput());
	}
}

void NeuralNetworkRepresentation::generateCloneWeights(
		std::map<std::string, std::string> &oldNew) {
	typedef std::map<std::string, std::string> MyMap;
	// for every neuron in the cloned tree
	for (MyMap::iterator it = oldNew.begin(); it != oldNew.end(); ++it) {
		std::string oldRon = it->first;
		std::string newRon = it->second;
		// for every weight
		for (WeightMap::iterator it = weights_.begin(); it != weights_.end();
				++it) {
			// if outgoing
			if (it->first.first == oldRon) {
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
			if (it->first.second == oldRon) {
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

void NeuralNetworkRepresentation::removeNeurons(std::string bodyPartId) {
	std::vector<boost::weak_ptr<NeuronRepresentation> > neurons =
			getBodyPartNeurons(bodyPartId);
	for (unsigned int i = 0; i < neurons.size(); ++i) {
		// remove all weights of the neuron
		boost::shared_ptr<NeuronRepresentation> neuron = neurons[i].lock();
		assert(neuron);
		for (WeightMap::iterator it = weights_.begin(); it != weights_.end();
				++it) {
			if (it->first.first == neuron->getId()
					|| it->first.second == neuron->getId()) {
				weights_.erase(it);
			}
		}
		// remove the neuron itself
		neurons_.erase(neurons_.find(neuron->getIoPair()));
	}
}

std::vector<boost::weak_ptr<NeuronRepresentation> > NeuralNetworkRepresentation::getBodyPartNeurons(
		std::string bodyPart) {
	std::vector<boost::weak_ptr<NeuronRepresentation> > ret;
	// go through neurons, check body part id
	int ioId = 0;
	while (neurons_[ioPair(bodyPart, ioId)])
		// TODO This is probably wrong!!
		ret.push_back(
				boost::weak_ptr<NeuronRepresentation>(
						neurons_[ioPair(bodyPart, ioId)]));
	return ret;
}

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

robogenMessage::Brain NeuralNetworkRepresentation::serialize() {
	robogenMessage::Brain serialization;

	// neurons
	for (std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::iterator it =
			neurons_.begin(); it != neurons_.end(); ++it) {
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
	for (std::map<std::pair<std::string, int>,
			boost::shared_ptr<NeuronRepresentation> >::iterator it =
			neurons_.begin(); it != neurons_.end(); ++it) {

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

} /* namespace robogen */
