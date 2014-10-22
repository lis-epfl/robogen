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

#if 0 // set to 1 to use fake robots for evolution algorithm benchmark
#include "evolution/representation/FakeRobotRepresentation.h"
#else

#include <string>
#include <set>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "config/RobogenConfig.h"
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"
#include "utils/network/TcpSocket.h"
#include "robogen.pb.h"

namespace robogen {

/**
 * Robot representation to be used for evolution. More lightweight than the
 * robot representation of the simulator, and implements evolution-specific
 * methods.
 */
class RobotRepresentation {

public:

	/**
	 * Map from an id string to a weak pointer of a part representation
	 */
	typedef std::map<std::string, boost::weak_ptr<PartRepresentation> > IdPartMap;

	/**
	 * Copy constructor: Deep copy body parts and Neural network
	 */
	RobotRepresentation(const RobotRepresentation &r);

	/**
	 * Error-less constructor for memory assignment
	 */
	RobotRepresentation();

	/**
	 * assignment operator: Deep copy body parts and Neural network
	 */
	RobotRepresentation &operator=(const RobotRepresentation &r);


	/**
	 * Constructs a robot representation from nothing.
	 * Will have just the core component.
	 */
	bool init();

	/**
	 * Constructs a robot representation from a robot text file
	 * @param robotTextFile text file of the robot description
	 * @todo make a better handling of formatting errors
	 */
	bool init(std::string robotTextFile);

	/**
	 * @return robot message of this robot to be transmitted to simulator
	 * or stored as population checkpoint
	 */
	robogenMessage::Robot serialize() const;

	/**
	 * Provides weight and bias handles for a mutator.
	 * @param weights reference to a vector to be filled with weight pointers
	 * @param types reference to a vector to be filled with types of neurons
	 * @param params reference to a vector to be filled with params pointers
	 */
	void getBrainGenome(std::vector<double*> &weights,
			std::vector<unsigned int> &types,
			std::vector<double*> &params);

	/**
	 * @return a shared pointer to the robots brain
	 */
	boost::shared_ptr<NeuralNetworkRepresentation> getBrain() const;

	/**
	 * @return a shared pointer to the robots body
	 */
	const IdPartMap &getBody() const;

	/**
	 * Evaluate individual using given socket and given configuration file.
	 * @param socket
	 * @param robotConf
	 */
	void evaluate(TcpSocket *socket,
			boost::shared_ptr<RobogenConfig> robotConf);

	/**
	 * @return fitness of individual
	 */
	double getFitness() const;

	/**
	 * @return evaluated_
	 */
	bool isEvaluated() const;

	/**
	 * Makes robot be not evaluated again
	 */
	void setDirty();

	/**
	 * Removes body part and all children at indicated position.
	 * @return false upon failure
	 */
	bool trimBodyAt(const std::string& id);

	/**
	 * @return a string unique id
	 */
	std::string generateUniqueIdFromSomeId();

	/**
	 * Duplicate a subtree in the body tree
	 *
	 * @param subtreeRootPartId the root of the subtree to duplicate
	 * @param subtreeDestPartId the destination part where the subtree will be attached to
	 * @param slotId the slot identifier of the destination part where the subtree will be attached to
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool duplicateSubTree(const std::string& subtreeRootPartId,
			const std::string& subtreeDestPartId, unsigned int slotId);

	/**
	 * Swap subtrees in the body tree
	 *
	 * @param subtreeRoot1 the root of the first subtree
	 * @param subtreeRoot2 the root of the second subtree
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool swapSubTrees(const std::string& subtreeRoot1,
			const std::string& subtreeRoot2);

	/**
	 * Insert a part into the body tree
	 *
	 * @param parentPartId id of the part that will become the parent of newPart
	 * @param parentPartSlot slot id where the newPart will be inserted
	 * @param newPart the new part to insert
	 * @param newPartSlot the slot of the new part where the subtree of parentPartId
	 *        connected to parentPartSlot will be connected to
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool insertPart(const std::string& parentPartId,
			unsigned int parentPartSlot,
			boost::shared_ptr<PartRepresentation> newPart,
			unsigned int newPartSlot);

	/**
	 * Remove a part from the body tree
	 *
	 * @param partId id of the part to remove
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool removePart(const std::string& partId);

	/**
	 * @return the id of the root body part
	 */
	const std::string& getBodyRootId();

	/**
	 * Check the consistency of this robot
	 * @return true if the body is consistent with the neural representation, and there are no
	 * dangling body parts/neurons
	 */
	bool check();

	/**
	 * @return a string representation of the robot
	 */
	std::string toString();

private:
	/**
	 *
	 */
	void recurseNeuronRemoval(boost::shared_ptr<PartRepresentation> part);

	/**
	 * Insert parts to the body id-parts map
	 *
	 * @param part the root of the subtree to be inserted into the body id to parts map
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool addClonesToMap(boost::shared_ptr<PartRepresentation> part,
			std::map<std::string, std::string> &neuronReMapping);

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

	/**
	 * Fitness of robot, once evaluated.
	 */
	double fitness_;

	/**
	 * Counter for unique ID.
	 */
	int maxid_;

	/**
	 * Indicates whether robot evaluated
	 */
	bool evaluated_;

};

/**
 * Operator > returns true if fitness of a exceeds fitness of b
 */
bool operator >(const RobotRepresentation &a, const RobotRepresentation &b);

}

#endif /* use of fake robot benchmark */

#endif /* ROBOTREPRESENTATION_H */
