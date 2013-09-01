/*
 * @(#) NeuralNetworkRepresentation.h   1.0   Aug 28, 2013
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
#ifndef NEURALNETWORKREPRESENTATION_H_
#define NEURALNETWORKREPRESENTATION_H_

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <set>
#include <map>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/NeuronRepresentation.h"
#include "robogen.pb.h"

namespace robogen {

class NeuralNetworkRepresentationException : public std::runtime_error {
public:
	NeuralNetworkRepresentationException(const std::string& w);
};

class NeuralNetworkRepresentation {
public:
	/**
	 * Creates a new neural network representation. The sensor and motor cache
	 * sets used to validate or generate neural connections will be filled.
	 * Weights and biases will not be generated (value=0), so that users can try
	 * out own neural networks by only specifying non-zero weights and biases.
	 * @param bodyParts a map that maps body part id's to amount of ioIds
	 */
	NeuralNetworkRepresentation(std::map<std::string,int> &sensorParts,
			std::map<std::string,int> &motorParts);

	// Copy constructor should be provided by the compiler. As there is no
	// pointing going on, this should not cause any problems.

	virtual ~NeuralNetworkRepresentation();

	/**
	 * Using the current motor and sensor cache sets, creates all weights and
	 * biases and assigns random values to them.
	 */
	void initializeRandomly();

	/**
	 * Sets the weight from sensor or motor "from" to motor "to" to value after
	 * verifying validness of connection
	 */
	void setWeight(std::string fromPart, int fromIoId, std::string toPart,
			int toIoId, double value);

	/**
	 * Sets the bias of specified neuron after verifying that this is not an
	 * input layer neuron.
	 */
	void setBias(std::string bodyPart, int ioId, double value);

	/**
	 * Refreshes the sensor and motor cache sets according to the passed body,
	 * removes dangling weights and biases and randomly initializes new ones.
	 * @param motors motor id's to be registered
	 * @param sensors sensor id's to be registered
	 */
	void adoptBody(std::vector<std::string> motors,
			std::vector<std::string> sensors);

	/**
	 * Serialize brain into message that can be appended to the robot message
	 * @return protobuf message of brain
	 */
	robogenMessage::Brain serialize();

private:
	/**
	 * Neurons of the neural network. This is the principal representation of
	 * neurons and the holder of shared pointers. For other references use
	 * weak or raw pointers. This should always be filled to the current body.
	 */
	std::map<std::pair<std::string,int>,boost::shared_ptr<NeuronRepresentation>
	> neurons_;

	/**
	 * Connections of the neural network. Use the ids of neurons as keys.
	 */
	std::map<std::pair<std::string,std::string>, double>
	weights_;
};

} /* namespace robogen */
#endif /* NEURALNETWORKREPRESENTATION_H_ */
