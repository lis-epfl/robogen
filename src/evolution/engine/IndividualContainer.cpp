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

namespace robogen {

IndividualContainer::IndividualContainer() : evaluated_(false), sorted_(false){
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
void evaluationThread(std::queue<RobotRepresentation*> *indiQueue,
		boost::mutex *queueMutex, TcpSocket *socket, std::string &confFile){
	while (true){
		boost::mutex::scoped_lock lock(*queueMutex);
		if (indiQueue->empty()) return;
		RobotRepresentation *current = indiQueue->front(); indiQueue->pop();
		std::cout << "." <<	std::flush;
		lock.unlock();

		current->evaluate(socket, confFile);
	}
}

void IndividualContainer::evaluate(std::string confFile,
		std::vector<TcpSocket*> &sockets){

	// 1. Create mutexed queue of Individual pointers
	std::queue<RobotRepresentation*> indiQueue;
	boost::mutex queueMutex;
	for (unsigned int i=0; i<this->size(); i++){
		if (!this->at(i).isEvaluated()){
			indiQueue.push(&this->at(i));
		}
	}
	std::cout << indiQueue.size() << " individuals queued for evaluation." <<
			" Progress:" << std::endl;

	// 2. Prepare thread structure
	boost::thread_group evaluators;

	// 3. Launch threads
	for (unsigned int i=0; i<sockets.size(); i++){
		evaluators.add_thread(new boost::thread(evaluationThread, &indiQueue,
				&queueMutex,sockets[i],confFile));
	}

	// 4. Join threads. Individuals are now evaluated.
	evaluators.join_all();
	// newline after per-individual dots
	std::cout << std::endl;

	evaluated_ = true;
}

void IndividualContainer::sort(){
	if (sorted_) return;
	std::sort(this->begin(), this->end(), operator >);
	sorted_ = true;
}

IndividualContainer &IndividualContainer::operator += (
		const IndividualContainer &other){
	this->insert(this->end(),other.begin(),other.end());
	sorted_ = false;
	return *this;
}

bool IndividualContainer::areEvaluated() const{
	return evaluated_;
}


} /* namespace robogen */
