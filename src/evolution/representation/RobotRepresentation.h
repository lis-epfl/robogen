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
#include <boost/weak_ptr.hpp>
#include "robogen.pb.h"
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"

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
	 * Copy constructor: Deep copy body parts and Neural network
	 */
	RobotRepresentation(const RobotRepresentation &r);

	/**
	 * assignment operator: Deep copy body parts and Neural network
	 */
	RobotRepresentation &operator=(const RobotRepresentation &r);

	/**
	 * Constructs a robot representation from a robot text file
	 * @param robotTextFile text file of the robot description
	 * @todo make a better handling of formatting errors
	 */
	RobotRepresentation(std::string robotTextFile);

	/**
	 * @return the root node of the body and thus the body tree
	 * @todo is this even needed?
	 */
	boost::shared_ptr<PartRepresentation> getBody();

	/**
	 * @return robot message of this robot to be transmitted to simulator
	 * or stored as population checkpoint
	 */
	robogenMessage::Robot serialize();

	/**
	 * Initializes the brain to be completely random.
	 */
	void randomizeBrain();

	/**
	 * Mutates the brain of the robot
	 * @param pWeight probability of each weight to mutate
	 * @param pBias probability of each bias to mutate
	 * @return true if anything has been mutated
	 */
	bool mutateBrain(float pWeight, float pBias);

private:
	/**
	 * Points to the root of the robot body tree
	 */
	boost::shared_ptr<PartRepresentation> bodyTree_;

	/**
	 * Neural network representation of the robot
	 */
	boost::shared_ptr<NeuralNetworkRepresentation> neuralNetwork_;

	/**
	 * Map from part id to part representation
	 * @todo use to avoid multiple samenames
	 */
	std::map<std::string, boost::weak_ptr<PartRepresentation> > idToPart_;

	// DO NOT FORGET TO COMPLETE COPY CONSTRUCTOR AND ASSIGNMENT OPERATOR!!!
};

}

#endif /* ROBOTREPRESENTATION_H */
