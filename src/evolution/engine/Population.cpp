/*
 * @(#) Population.cpp   1.0   Sep 1, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2015 Titus Cieslewski, Joshua Auerbach
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
#include "robogen.pb.h"
#include "utils/network/ProtobufPacket.h"
#include "evolution/engine/BodyVerifier.h"

namespace robogen {

Population::Population() :
		IndividualContainer() {

}

bool Population::init(boost::shared_ptr<RobotRepresentation> robot, int popSize,
		boost::shared_ptr<Mutator> mutator, bool growBodies,
		bool randomizeBrains) {

	// fill population vector
	for (int i = 0; i < popSize; i++) {

		if (i == 0 || randomizeBrains) {
			this->push_back(
				boost::shared_ptr<RobotRepresentation>(
						new RobotRepresentation(*robot.get())));

			if (randomizeBrains) {
				mutator->randomizeBrain(this->back());
			}
		} else { // i > 0 and !randomizeBrains, create mutated copy of seed
			this->push_back( mutator->createOffspring(robot)[0] );
		}

		if (growBodies) {
			mutator->growBodyRandomly(this->back());
		}
		//BodyVerifier::fixRobotBody(this->back());
	}
	return true;
}

bool Population::init(const IndividualContainer &origin, unsigned int popSize) {

	if (!origin.areEvaluated()) {
		std::cout << "Trying to initialize population of n best Robots from "
				"IndividualContainer which is not evaluated!" << std::endl;
		return false;
	}

	for (unsigned int i = 0; i < origin.size(); i++) {
		this->push_back(origin[i]);
	}

	this->sort();

	// idea was to call this->resize(popSize);, but that requires a default
	// constructor to be present for RobotRepresentation, which is not the case
	// on purpose
	while (this->size() > popSize) {
		this->pop_back();
	}

	this->evaluated_ = true;

	return true;
}

Population::~Population() {
}

boost::shared_ptr<RobotRepresentation> Population::best() {
	if (!this->areEvaluated()) { // undefined behavior. No exception (hint)
		return this->at(0);
	}
	this->sort();
	return this->at(0);
}

bool Population::getStat(double &bestFit, double &average,
		double &stdev) const {

	if (!this->areEvaluated()) {
		std::cout << "Trying to get stats on non-evaluated population"
				<< std::endl;
	}
	boost::accumulators::accumulator_set<double,
			boost::accumulators::stats<boost::accumulators::tag::mean,
					boost::accumulators::tag::variance,
					boost::accumulators::tag::max> > acc;
	for (unsigned int i = 0; i < this->size(); i++)
		acc(this->at(i)->getFitness());
	bestFit = boost::accumulators::max(acc);
	average = boost::accumulators::mean(acc);
	stdev = std::sqrt((double) boost::accumulators::variance(acc));
	return true;

}

}
