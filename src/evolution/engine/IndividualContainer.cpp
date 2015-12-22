/*
 * @(#) IndividualContainer.cpp   1.0   Sep 12, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
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

#include "evolution/engine/IndividualContainer.h"
#include <algorithm>
#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#ifdef EMSCRIPTEN
#include <utils/network/FakeJSSocket.h>
#include <boost/lexical_cast.hpp>
void sendJSEvent(std::string name, std::string jsonData);
#endif

namespace robogen {

IndividualContainer::IndividualContainer() :
		evaluated_(false), sorted_(false) {
}

IndividualContainer::~IndividualContainer() {
	// TODO Auto-generated destructor stub
}

/**
 * Thread function assigned to a socket
 * @param indiQueue queue of Individuals to be evaluated
 * @param queueMutex mutex for access to queue
 * @param socket socket to simulator
 * @param confFile simulator configuration file to be used for evaluations
 */
void evaluationThread(
		std::queue<boost::shared_ptr<RobotRepresentation> >& indiQueue,
		boost::mutex& queueMutex, Socket& socket,
		boost::shared_ptr<RobogenConfig> robotConf) {

	while (true) {

		boost::mutex::scoped_lock lock(queueMutex);
		if (indiQueue.empty()) {
			return;
		}

		boost::shared_ptr<RobotRepresentation> current = indiQueue.front();
		indiQueue.pop();
		std::cout << "." << std::flush;
		lock.unlock();

		current->evaluate(&socket, robotConf);

	}

}

void IndividualContainer::evaluate(boost::shared_ptr<RobogenConfig> robotConf,
		std::vector<Socket*> &sockets) {

	// 1. Create mutexed queue of Individual pointers
	std::queue<boost::shared_ptr<RobotRepresentation> > indiQueue;
	boost::mutex queueMutex;
	for (unsigned int i = 0; i < this->size(); i++) {
		if (!this->at(i)->isEvaluated()) {
			indiQueue.push(this->at(i));
		}
	}
	std::cout << indiQueue.size() << " individuals queued for evaluation."
			<< " Progress:" << std::endl;

#ifdef EMSCRIPTEN
	std::string message = "[";
	bool firstIndividual = true;
	int sent = 0;
	while (!indiQueue.empty()){
		++sent;
		FakeJSSocket socket;
		boost::shared_ptr<RobotRepresentation> currentRobot = indiQueue.front();
		indiQueue.pop();
		currentRobot->evaluate(&socket, robotConf);
		int ptrToIndividual = (int) currentRobot.get();
		if (!firstIndividual) {
			message += ",";
		} else {
			firstIndividual = false;
		}
		message += "{ptr:";
		message += boost::lexical_cast<std::string>(ptrToIndividual);
		message += ", packet : [";
		bool firstByte = true;
		std::vector<unsigned char> content = socket.getContent();
		for (size_t k = 0 ; k < content.size(); ++k) {
			if (!firstByte) {
				message += ",";
			} else {
				firstByte = false;
			}
			message += boost::lexical_cast<std::string>((int) content[k]);
		}
		message += "]}";
	}
	message += "]";
	sendJSEvent("needsEvaluation", message);
	std::cout << sent << " inidividual sent to the javascript scheduler" << std::endl;

#else

	// 2. Prepare thread structure
	boost::thread_group evaluators;

	// 3. Launch threads
	for (unsigned int i = 0; i < sockets.size(); i++) {
		evaluators.add_thread(
				new boost::thread(evaluationThread, boost::ref(indiQueue), boost::ref(queueMutex),
						boost::ref(*sockets[i]), robotConf));
	}

	// 4. Join threads. Individuals are now evaluated.
	evaluators.join_all();

	// newline after per-individual dots
	std::cout << std::endl;
#endif

	evaluated_ = true;
}

bool robotFitnessComparator(const boost::shared_ptr<RobotRepresentation>& a,
		const boost::shared_ptr<RobotRepresentation>& b) {

	return a->getFitness() > b->getFitness();

}

void IndividualContainer::sort(bool forceSort) {

	if (sorted_  && (!forceSort)) {
		return;
	}

	std::sort(this->begin(), this->end(), robotFitnessComparator);
	sorted_ = true;
}

IndividualContainer &IndividualContainer::operator +=(
		const IndividualContainer &other) {

	this->insert(this->end(), other.begin(), other.end());
	sorted_ = false;
	return *this;

}

bool IndividualContainer::areEvaluated() const {
	return evaluated_;
}

} /* namespace robogen */
