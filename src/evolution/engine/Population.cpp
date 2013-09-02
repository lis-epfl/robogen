/*
 * @(#) Population.cpp   1.0   Sep 1, 2013
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

#include "evolution/engine/Population.h"
#include <algorithm>
#include "robogen.pb.h"
#include "utils/network/ProtobufPacket.h"

namespace robogen {

/**
 * Used for ordering individuals by fitness
 */
bool operator >(const Individual &a, const Individual &b){
	return a.fitness > b.fitness;
}

Population::Population(RobotRepresentation &robot, int popSize,
		boost::random::mt19937	&rng) {
	// fill population vector
	robots_.resize(popSize);
	for (int i=0; i<popSize; i++){
		robots_[i].robot = boost::shared_ptr<RobotRepresentation>(
				new RobotRepresentation(robot));
		robots_[i].robot->randomizeBrain(rng);
		robots_[i].fitness = 0.;
		robots_[i].evaluated = false;
	}
	evaluated_ = false;
}

Population::Population(std::vector<Individual> &robots) : evaluated_(true){
	// need to explicitly call copy constructor!!! This is puzzling...
	robots_.clear();
	robots_.resize(robots.size());
	for (unsigned int i=0; i<robots.size(); i++){
		robots_[i] = robots[i];
		robots_[i].robot = boost::shared_ptr<RobotRepresentation>(
				new RobotRepresentation(*robots[i].robot.get()));
	}
}

Population::~Population() {
}

boost::shared_ptr<RobotRepresentation> Population::getRobot(int n){
	return robots_[n].robot;
}

// TODO use threads & load balancing to speed things up even further
void Population::evaluate(std::vector<TcpSocket*> &sockets){
	for (unsigned int i=0; i<robots_.size(); i+=sockets.size()){
		for (unsigned int j=0; j<sockets.size(); j++){
			boost::shared_ptr<robogenMessage::Robot> rsp =
					boost::shared_ptr<robogenMessage::Robot>(
							new robogenMessage::Robot(
									robots_[i+j].robot->serialize()));


			ProtobufPacket<robogenMessage::Robot>	robotPacket(rsp);
			std::vector<unsigned char> forgedMessagePacket;
			robotPacket.forge(forgedMessagePacket);

			// Write vector<unsigned char> to server
			sockets[j]->write(forgedMessagePacket);
		}

		for (unsigned int j=0; j<sockets.size(); j++){
			// Reading fitness packet from server
			ProtobufPacket<robogenMessage::EvaluationResult> resultPacket(
					boost::shared_ptr<robogenMessage::EvaluationResult>
			(new robogenMessage::EvaluationResult()) );
			std::vector<unsigned char> responseMessage;
			// Wait to receive a vector<unsigned char> from server
			sockets[j]->read(responseMessage,
					ProtobufPacket<robogenMessage::EvaluationResult>::HEADER_SIZE);
			// Decode the Header and read the payload-message-size
			size_t msgLen = resultPacket.decodeHeader(responseMessage);
			responseMessage.clear();
			// Read the fitness payload message
			sockets[j]->read(responseMessage, msgLen);
			// Decode the packet
			resultPacket.decodePayload(responseMessage);


			// Obtain the fitness from the evaluationResponse message
			// TODO exceptions
			if(!resultPacket.getMessage()->has_fitness()) {
				std::cerr << "Fitness field not set by Simulator!!!" <<
						std::endl;
				exit(EXIT_FAILURE);
			}
			else {
				robots_[i+j].fitness = resultPacket.getMessage()->fitness();
				robots_[i+j].evaluated = true;
			}
		}
	}
	evaluated_ = true;
	// sort individuals by fitness, descending
	std::sort(robots_.begin(), robots_.end(), operator >);
	std::cout << "Fitnesses:" << std::endl;
	for (unsigned int i=0; i<robots_.size(); i++){
		std::cout << robots_[i].fitness << " ";
	}
	std::cout << std::endl;
}

std::vector<Individual> &Population::orderedEvaluatedRobots(){
	// TODO throw population exception if not evaluated
	return robots_;
}

} /* namespace robogen */
