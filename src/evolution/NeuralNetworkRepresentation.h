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
#include <stdexcept>
#include <boost/shared_ptr.hpp>

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
	 * @param motors motor id's to be registered
	 * @param sensors sensor id's to be registered
	 */
	NeuralNetworkRepresentation(std::vector<std::string> motors,
			std::vector<std::string> sensors);

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
	 * verifying with the cache sets that both from and to indeed exist
	 */
	void setWeight(std::string from, std::string to, double value);

	/**
	 * Sets the bias of motor to value after verifying with the cache sets that
	 * the motor indeed exists
	 */
	void setBias(std::string motor, double value);

	/**
	 * Refreshes the sensor and motor cache sets according to the passed body,
	 * removes dangling weights and biases and randomly initializes new ones.
	* @param motors motor id's to be registered
	 * @param sensors sensor id's to be registered
	 */
	void adoptBody(std::vector<std::string> motors,
			std::vector<std::string> sensors);

private:
	/**
	 * Set of allowed sensor identifiers
	 */
	std::set<std::string> sensors_;

	/**
	 * Set of allowed motor identifiers
	 */
	std::set<std::string> motors_;

	/**
	 * Weights of the neural network
	 */
	std::map<std::pair<std::string,std::string>, double> weights_;

	/**
	 * Biases of the neural network
	 */
	std::map<std::string, double> bias_;
};

} /* namespace robogen */
#endif /* NEURALNETWORKREPRESENTATION_H_ */
