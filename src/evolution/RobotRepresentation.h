/*
 * @(#) RobotRepresentation.h   1.0   Aug 28, 2013
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

#ifndef ROBOTREPRESENTATION_H
#define ROBOTREPRESENTATION_H

#include <string>
#include <set>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include "robogen.pb.h"
#include "evolution/PartRepresentation.h"
#include "evolution/NeuralNetworkRepresentation.h"

namespace robogen{

/**
 * Exception class for exceptions that happen when dealing with a robot
 * representation.
 */
class RobotRepresentationException : public std::runtime_error {
public:
	RobotRepresentationException(const std::string& w);
};

/**
 * Robot representation to be used for evolution. More lightweight than the
 * robot representation of the simulator, and implements evolution-specific
 * methods.
 */
class RobotRepresentation{
public:
	/**
	 * Constructs a robot representation from a robot text file
	 * @param robotTextFile text file of the robot description
	 * @todo make a better handling of formatting errors
	 */
	RobotRepresentation(std::string robotTextFile);

	/**
	 * @return the root node of the body and thus the body tree
	 */
	boost::shared_ptr<PartRepresentation> getBody();

	/**
	 * Mutates the brain of the robot
	 * @param pWeight probability of each weight to mutate
	 * @param pBias probability of each bias to mutate
	 * @return true if anything has been mutated
	 */
	bool mutateBrain(float pWeight, float pBias);

private:
	/**
	 * Gathers motor id's from all body parts
	 * @todo detect duplicate motor id's
	 */
	std::vector<std::string> getMotors();

	/**
	 * Gathers sensor id's from all body parts
	 * @todo detect duplicate sensor id's
	 */
	std::vector<std::string> getSensors();

	/**
	 * Points to the root of the robot body tree
	 */
	boost::shared_ptr<PartRepresentation> bodyTree_;

	/**
	 * Neural network representation of the robot
	 */
	boost::shared_ptr<NeuralNetworkRepresentation> neuralNetwork_;

	/**
	 * Set of reserved body part id's, for body evolution
	 * @todo use
	 */
	std::set<std::string> reservedIds_;
};

}

#endif /* ROBOTREPRESENTATION_H */
