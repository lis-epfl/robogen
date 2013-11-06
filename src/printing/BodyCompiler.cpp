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
#include "model/sensors/SimpleSensor.h"
#include "robogen.pb.h"
#include "evolution/representation/RobotRepresentation.h"

namespace robogen {

BodyCompiler::BodyCompiler() {
}

BodyCompiler::~BodyCompiler() {
}

void BodyCompiler::compile(Robot &robot,
		std::ofstream &file){


	int nLight = 0, nTouch = 0, nServo = 0;
	std::vector<int> input;

	std::vector<boost::shared_ptr<Model> > bodyParts = robot.getBodyParts();
	const std::vector<boost::shared_ptr<Connection> > bodyConns = robot.getBodyConnections();
	//robot.traverseBody(bodyParts,bodyConns);

	// body traversal to type parts
	// get body connections
	std::map<boost::shared_ptr<Model>, std::vector<boost::shared_ptr<Connection> > > bodyGraph;
	std::map<boost::shared_ptr<Model>, bool > traversed;

	for(unsigned int i=0; i<bodyConns.size(); i++){
		bodyGraph[bodyConns[i]->getFrom()].push_back(bodyConns[i]);
		bodyGraph[bodyConns[i]->getTo()].push_back(bodyConns[i]);
	}
	std::stack<std::pair<boost::shared_ptr<Connection>, int> > todo;
	boost::shared_ptr<Connection> dummy(new Connection(
			boost::shared_ptr<Model>(),0,bodyParts[robot.getRoot()],0));
	todo.push(std::pair<boost::shared_ptr<Connection>, int>(dummy,0));

	traversed[bodyParts[robot.getRoot()]]= true;

	const robogenMessage::Body &body = robot.getMessage().body();
	std::map<std::string, const robogenMessage::BodyPart*> idToBodyPart;
/*
	for(int i=0; i<body.neuron_size(); i++){


		const robogenMessage::Neuron& neuron = brain.neuron(i);

		if (neuron.layer().compare("output") == 0) {
			//TODO: save to map the bodypartIDs

			idToNeuron[neuron.id()] = &neuron;

		}

	} */

	while( !todo.empty() ){
		std::pair<boost::shared_ptr<Connection>, int> current = todo.top();
		todo.pop();
		for (int i = 0; i< current.second; i++) {
			file << "\t";
		}


		file << current.first->getFromSlot() << " " << std::endl;
		//TODO: pointercast to find out type
		for(int i=0; i<bodyGraph[current.first->getTo()].size(); i++){
			if(!traversed[bodyGraph[current.first->getTo()][i]->getTo()]){
				todo.push(std::pair<boost::shared_ptr<Connection>, int>
				(bodyGraph[current.first->getTo()][i],current.second+1));
				traversed[bodyGraph[current.first->getTo()][i]->getTo()] = true;
			}

			if(!traversed[bodyGraph[current.first->getTo()][i]->getFrom()]){
				todo.push(std::pair<boost::shared_ptr<Connection>, int>
				(bodyGraph[current.first->getTo()][i],
						current.second+1));
				std::cout << "Titus loses" << std::endl;
			}

		}
	}

/*
	for (unsigned int i=0; i<bodyParts.size(); i++){
		// light sensor
		bodyParts[i];
		if (boost::dynamic_pointer_cast<LightSensor>(
				bodyParts[i])){
			// TODO: Type all parts, write out to file
			file  << bodyParts[i]->getId() << std::endl;
		}

	} */



	const robogenMessage::Brain &brain = robot.getMessage().brain();
	std::map<std::string, const robogenMessage::Neuron*> idToNeuron;

	for(int i=0; i<brain.neuron_size(); i++){


		const robogenMessage::Neuron& neuron = brain.neuron(i);

		if (neuron.layer().compare("output") == 0) {
			//TODO: save to map the bodypartIDs

			idToNeuron[neuron.id()] = &neuron;

		}

	}

	for (int i = 0; i < brain.connection_size(); ++i){

		const robogenMessage::NeuralConnection& connection =
				brain.connection(i);


		file << idToNeuron[connection.src()]->bodypartid() << " " << idToNeuron[connection.src()]->ioid() << " " <<
				idToNeuron[connection.dest()]->bodypartid() << " " << idToNeuron[connection.dest()]->ioid() << " " << connection.weight() << std::endl;



	}

	for(int i=0; i<brain.neuron_size(); i++){


		const robogenMessage::Neuron& neuron = brain.neuron(i);

		if (neuron.layer().compare("output") == 0) {

			file << neuron.bodypartid() << " " << neuron.ioid() << " " <<
					neuron.biasweight() << std::endl;

		}

	}

}

} /* namespace robogen */


