/*
 * BodyCompiler.cpp
 * Created on: Nov 5, 2013
 * Author: Deniz Aydin (deniz.aydin@epfl.ch)
 *
 * Previous work by:
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * Gregoire Heitz (gregoire.heitz@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014
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

#include <stack>
#include "printing/BodyCompiler.h"
//#include "printing/BodyConfiguration.h"
#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "model/ActuatedComponent.h"
#include "model/PerceptiveComponent.h"
#include "model/motors/Motor.h"
#include "model/sensors/Sensor.h"
#include "model/sensors/LightSensor.h"
#include "model/sensors/TouchSensor.h"
#include "robogen.pb.h"
#include "PartList.h"
#include "evolution/representation/RobotRepresentation.h"

namespace robogen {

BodyCompiler::BodyCompiler() {
}

BodyCompiler::~BodyCompiler() {
}

void BodyCompiler::compile(Robot &robot, std::ofstream &file) {

	// Retrieve body parts and connections
	std::vector<boost::shared_ptr<Model> > bodyParts = robot.getBodyParts();
	const std::vector<boost::shared_ptr<Connection> > bodyConns =
			robot.getBodyConnections();

	// Body Graph is a map between a part and all undirected connections linked to it
	std::map<boost::shared_ptr<Model>,
			std::vector<boost::shared_ptr<Connection> > > bodyGraph;
	for (unsigned int i = 0; i < bodyConns.size(); i++) {
		bodyGraph[bodyConns[i]->getFrom()].push_back(bodyConns[i]);
		bodyGraph[bodyConns[i]->getTo()].push_back(bodyConns[i]);
	}

	// Traversed flags visited parts in the tree
	std::map<boost::shared_ptr<Model>, bool> traversed;

	// Stack of nodes to visit - pair of (nodes, depth)
	std::stack<std::pair<boost::shared_ptr<Connection>, int> > nodesToVisit;

	// Creates a dummy connection to the root node, and mark it as visited
	boost::shared_ptr<Connection> dummy(
			new Connection(boost::shared_ptr<Model>(), 0,
					bodyParts[robot.getRoot()], 0));
	nodesToVisit.push(std::pair<boost::shared_ptr<Connection>, int>(dummy, 0));
	traversed[bodyParts[robot.getRoot()]] = true;

	// Retrieve body parts also from message, to read body types and maps a
	// body part id to its body type
	const robogenMessage::Body &body = robot.getMessage().body();
	std::map<std::string, char> partIdToType;
	std::map<std::string, char> partIdToOrientation;
	std::map<std::string, std::vector<double> > partIdToParams;

	for (int i = 0; i < body.part_size(); i++) {
		const robogenMessage::BodyPart& bodyPart = body.part(i);
		partIdToType[bodyPart.id()] = INVERSE_PART_TYPE_MAP.at(bodyPart.type());
		partIdToOrientation[bodyPart.id()] = bodyPart.orientation();
		std::vector<double> params;
		for (int j = 0; j < bodyPart.evolvableparam_size(); ++j) {
			params.push_back(bodyPart.evolvableparam(j).paramvalue());
		}
		partIdToParams[bodyPart.id()] = params;
	}

	// While there are nodes to visit
	while (!nodesToVisit.empty()) {

		// Get a node to visit
		std::pair<boost::shared_ptr<Connection>, int> current =
				nodesToVisit.top();
		nodesToVisit.pop();

		// Write correct indentation to file and current slot
		for (int i = 0; i < current.second; i++) {
			file << "\t";
		}
		file << current.first->getFromSlot() << " "
				<< partIdToType[current.first->getTo()->getId()] << " "
				<< current.first->getTo()->getId() << " "
				<< current.first->getTo()->getOrientationToParentSlot();
		for (unsigned int i = 0;
				i < partIdToParams[current.first->getTo()->getId()].size();
				i++) {
			file << " " << partIdToParams[current.first->getTo()->getId()][i];
		}
		file << std::endl;

		// Add children to the nodes to visit
		for (unsigned int i = 0; i < bodyGraph[current.first->getTo()].size();
				i++) {

			if (!traversed[bodyGraph[current.first->getTo()][i]->getTo()]) {
				nodesToVisit.push(
						std::pair<boost::shared_ptr<Connection>, int>(
								bodyGraph[current.first->getTo()][i],
								current.second + 1));
				traversed[bodyGraph[current.first->getTo()][i]->getTo()] = true;
			}

			if (!traversed[bodyGraph[current.first->getTo()][i]->getFrom()]) {
				nodesToVisit.push(
						std::pair<boost::shared_ptr<Connection>, int>(
								bodyGraph[current.first->getTo()][i],
								current.second + 1));
				std::cout
						<< "Error: tree traversal visited the same node twice!"
						<< std::endl;
				return;
			}

		}
	}

	const robogenMessage::Brain &brain = robot.getMessage().brain();
	std::map<std::string, const robogenMessage::Neuron*> idToNeuron;

	for (int i = 0; i < brain.neuron_size(); i++) {

		const robogenMessage::Neuron& neuron = brain.neuron(i);

		idToNeuron[neuron.id()] = &neuron;

	}

	for (int i = 0; i < brain.connection_size(); ++i) {

		const robogenMessage::NeuralConnection& connection = brain.connection(
				i);

		file << idToNeuron[connection.src()]->bodypartid() << " "
				<< idToNeuron[connection.src()]->ioid() << " "
				<< idToNeuron[connection.dest()]->bodypartid() << " "
				<< idToNeuron[connection.dest()]->ioid() << " "
				<< connection.weight() << std::endl;

	}
	file << std::endl;
	file << std::endl;

	for (int i = 0; i < brain.neuron_size(); i++) {

		const robogenMessage::Neuron& neuron = brain.neuron(i);

		if (neuron.layer().compare("output") == 0) {

			file << neuron.bodypartid() << " " << neuron.ioid() << " "
					<< neuron.bias() << std::endl;
			// TODO handle other neuron types!!

		}

	}

}

} /* namespace robogen */

