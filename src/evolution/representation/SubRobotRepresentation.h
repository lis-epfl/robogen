/*
 * @(#) RobotRepresentation.cpp   1.0   Aug 28, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2016 Titus Cieslewski, Andrea Maesani, Joshua Auerbach
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

#ifndef SUB_ROBOT_REPRESENTATION_H
#define SUB_ROBOT_REPRESENTATION_H

#include <string>
#include <set>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>

#include"ParseHelpers.h"

#include "config/RobogenConfig.h"
#include "evolution/representation/PartRepresentation.h"
#include "evolution/representation/NeuralNetworkRepresentation.h"
#include "utils/network/TcpSocket.h"
#include "robogen.pb.h"

namespace robogen{

/**
 * The purpose of this class is to reduce the redundancy in the code,
 * and also to spread the compelxity of Robogen across multiple
 * not-so-long files
 */

class SubRobotRepresentation{
public:
    /**
	 * Map from an id string to a weak pointer of a part representation
	 */
	typedef std::map<std::string, boost::weak_ptr<PartRepresentation> > IdPartMap;

	/**
	 * Constructor that casts a core as the root, nothing more
	 */
    SubRobotRepresentation();

	/**
	 * Copy constructor: Deep copy body parts and Neural network
	 */
	SubRobotRepresentation(const SubRobotRepresentation &r);

	/**
	 * @return a shared pointer to the robots body
	 */
	const IdPartMap &getBody() const;

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
	 * @return a shared pointer to the body tree
	 */
	boost::shared_ptr<PartRepresentation> getTree() const;

	int getMaxId();

	/**
	 * Overloading the equals operator!=
	 */
	SubRobotRepresentation &operator=(const SubRobotRepresentation &r);

	robogenMessage::Robot serialize() const;

	void rebuildBodyMap();

    /**
	 * Constructs a subrobot representation from nothing.
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
	 * Removes body part and all children at indicated position.
	 * @return false upon failure
	 */
	bool trimBodyAt(const std::string& id, bool printErrors=true);

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
			const std::string& subtreeDestPartId, unsigned int slotId,
			bool printErrors=true);

	/**
	 * Swap subtrees in the body tree
	 *
	 * @param subtreeRoot1 the root of the first subtree
	 * @param subtreeRoot2 the root of the second subtree
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool swapSubTrees(const std::string& subtreeRoot1,
			const std::string& subtreeRoot2, bool printErrors=true);

	/**
	 * Insert a part into the body tree
	 *
	 * @param parentPartId id of the part that will become the parent of newPart
	 * @param parentPartSlot slot id where the newPart will be inserted
	 * @param newPart the new part to insert
	 * @param newPartSlot the slot of the new part where the subtree of parentPartId
	 *        connected to parentPartSlot will be connected to
	 * @param motorNeuronType the type of the motor neurn (if applicable)
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool insertPart(const std::string& parentPartId,
			unsigned int parentPartSlot,
			boost::shared_ptr<PartRepresentation> newPart,
			unsigned int newPartSlot,
			unsigned int motorNeuronType,
			bool printErrors=true);

	/**
	 * Remove a part from the body tree
	 *
	 * @param partId id of the part to remove
	 * @return true if the operation completed successfully, false otherwise
	 */
	bool removePart(const std::string& partId, bool printErrors=true);


	/**
	*Change a part into another one
	*
	*@param partId id of the part that will change his children position
	*@param children the new children vector
	*@return true if the operation completed successfully, false otherwise
	*/
	bool setChildPosition(const std::string& partId,  
			std::vector<boost::shared_ptr<PartRepresentation>> children,
			bool printErrors);
	/**
	 * Auxiliar method to see the SubRobot as a string
	 */
	std::string toString();

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

	boost::shared_ptr<PartRepresentation> getNodeById(std::string id);
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
	 * @todo use to avoid multiple same names
	 */
	IdPartMap idToPart_;

    /**
	 * Counter for unique ID.
	 */
	int maxid_;
};

}
#endif //SUB_ROBOT_REPRESENTATION_H