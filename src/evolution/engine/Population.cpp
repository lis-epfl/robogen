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

PopulationException::PopulationException(const std::string& w) :
						std::runtime_error(w){}

Population::Population(RobotRepresentation &robot, int popSize,
		boost::random::mt19937	&rng) : IndividualContainer() {
	// fill population vector
	this->resize(popSize);
	for (int i=0; i<popSize; i++){
		this->at(i) = new RobotRepresentation(robot);
		this->at(i).randomizeBrain(rng);
		this->at(i).setDirty();
	}
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

Population::Population(const IndividualContainer &origin, int popSize){
	IndividualContainer(origin);
	// clone: Make sure to invoke copy constructor TODO feels weird
	for (unsigned int i=0; i<this->size(); ++i){
		this->at(i) = this->at(i).clone();
	}
	this->sort();
	this->resize(popSize);
}

Population::~Population() {
}

Individual &Population::best() const{
	if (!this->areEvaluated()){
		throw PopulationException("Trying to get best individual from"\
				" non-evaluated population!");
	}
	this->sort();
	return this->at(0);
}

void Population::getStat(double &best, double &average, double &stdev) const{
	boost::accumulators::accumulator_set<double,
	boost::accumulators::stats<boost::accumulators::tag::mean,
	boost::accumulators::tag::variance,
	boost::accumulators::tag::max> > acc;
	for (unsigned int i=0; i<this->size(); i++) acc(this->at(i).getFitness());
	best = boost::accumulators::max(acc);
	average = boost::accumulators::mean(acc);
	std = std::sqrt((double)boost::accumulators::variance(acc));
}

} /* namespace robogen */
