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
#include <queue>
#include <math.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
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

/**
 * Thread function assigned to a socket
 * @param indiQueue queue of Individuals to be evaluated
 * @param queueMutex mutex for access to queue
 * @param socket socket to simulator
 * @param confFile simulator configuration file to be used for evaluations
 */
void evaluationThread(std::queue<Individual*> *indiQueue,
		boost::mutex *queueMutex, TcpSocket *socket, std::string &confFile){
	while (true){
		boost::mutex::scoped_lock lock(*queueMutex);
		if (indiQueue->empty()) return;
		Individual *current = indiQueue->front(); indiQueue->pop();
		std::cout << "." <<	std::flush;
		lock.unlock();

		current->fitness = current->robot->evaluate(socket, confFile);
		current->evaluated = true;
	}
}

void Population::evaluate(std::string confFile,
		std::vector<TcpSocket*> &sockets){

	// 1. Create mutexed queue of Individual pointers
	std::queue<Individual*> indiQueue;
	boost::mutex queueMutex;
	for (unsigned int i=0; i<robots_.size(); i++){
		indiQueue.push(&robots_[i]);
	}

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

	// 5. sort individuals by fitness, descending
	std::sort(robots_.begin(), robots_.end(), operator >);

	// 6. calculate best, average and std
	boost::accumulators::accumulator_set<double,
	boost::accumulators::stats<boost::accumulators::tag::mean,
	boost::accumulators::tag::variance,
	boost::accumulators::tag::max> > acc;
	for (unsigned int i=0; i<robots_.size(); i++) acc(robots_[i].fitness);
	best_ = boost::accumulators::max(acc);
	average_ = boost::accumulators::mean(acc);
	std_ = std::sqrt((double)boost::accumulators::variance(acc));
	evaluated_ = true;

	std::cout << "Best: " << best_ << " Average: " << average_ << " STD: " <<
			std_ << std::endl;
}

std::vector<Individual> &Population::orderedEvaluatedRobots(){
	// TODO throw population exception if not evaluated
	return robots_;
}

boost::shared_ptr<RobotRepresentation> Population::bestRobot() const{
	if (!evaluated_){
		// TODO throw exception
	}
	return robots_[0].robot;
}

void Population::getStat(double &best, double &average, double &stdev) const{
	if (!evaluated_){
		// TODO throw exception
	}
	best = best_;
	average = average_;
	stdev = std_;
}

} /* namespace robogen */
