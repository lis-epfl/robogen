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

#if 0 // set to 1 to use fake robots - evolution algorithm benchmark
#include "evolution/representation/FakeRobotRepresentation.h"
#else

#include <string>
#include <set>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include "robogen.pb.h"
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"
#include "utils/network/TcpSocket.h"

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
	/**
	 * Map from an id string to a weak pointer of a part representation
	 */
	typedef std::map<std::string, boost::weak_ptr<PartRepresentation> >
	IdPartMap;

public:
	/**
	 * Return codes for getSensorType(body id)
	 */
	enum SensorTypes{
		LIGHT_SENSOR,
		TOUCH_SENSOR,
		IMU
	};

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
	 * This function is used by the arduino code compiler
	 * @return type of sensor for the given body part id
	 */
	int getSensorType(const std::string &id);

	/**
	 * @return robot message of this robot to be transmitted to simulator
	 * or stored as population checkpoint
	 */
	robogenMessage::Robot serialize();

	/**
	 * @return fitness of the robot when exposed to the given scenario
	 */
	double evaluate(TcpSocket *socket, std::string simulatorConfFile);

	/**
	 * Initializes the brain to be completely random.
	 */
	void randomizeBrain(boost::random::mt19937	&rng);

	/**
	 * Provides weight and bias handles for a mutator.
	 * @param weights reference to a vector to be filled with weight pointers
	 * @param biases reference to a vector to be filled with bias pointers
	 */
	void getBrainGenome(std::vector<double*> &weights,
			std::vector<double*> &biases);

	/**
	 * @return a const reference to the robots brain
	 */
	boost::shared_ptr<NeuralNetworkRepresentation> getBrain() const;

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
	 IdPartMap idToPart_;

	// DO NOT FORGET TO COMPLETE COPY CONSTRUCTOR AND ASSIGNMENT OPERATOR!!!
};

}

#endif /* use of fake robot benchmark */

#endif /* ROBOTREPRESENTATION_H */
