/*
 * @(#) Population.h   1.0   Sep 1, 2013
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

#ifndef POPULATION_H_
#define POPULATION_H_

#include <vector>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include "evolution/representation/RobotRepresentation.h"
#include "utils/network/TcpSocket.h"

namespace robogen {

/**
 * Represents a robot as a member of a population. Additional attributes are
 * fitness and a bool whether the latter is set.
 * @todo put this into a class?
 */
typedef struct{
	boost::shared_ptr<RobotRepresentation> robot;
	double fitness;
	bool evaluated;
}Individual;

class Population {
public:
	/**
	 * Constructs a population from a given robot body, keeping the body
	 * constant, but randomly initializing the brain.
	 * @param robot Reference robot body
	 * @param popSize chosen size for population
	 * @param rng boost random number generator for diverse purposes
	 */
	Population(RobotRepresentation &robot, int popSize, boost::random::mt19937
			&rng);

	/**
	 * Constructs a population that will contain the specified robots.
	 * @param robots Robots to be used as population
	 */
	Population(std::vector<Individual> &robots);

	virtual ~Population();

	/**
	 * Evaluate the population using the given sockets to transmit the robot
	 * to simulator instances in parallel. Then order the population by fitness.
	 * @param
	 */
	void evaluate(std::vector<TcpSocket*> &sockets);

	/**
	 * This function shall be used by the classes that act upon a population,
	 * e.g. Selector, Mutator
	 * @return reference to ordered and evaluated robots
	 */
	std::vector<Individual> &orderedEvaluatedRobots();

	/**
	 * Only for debugging purposes: Obtain some robot representation
	 * @param n robot to get
	 * @note does not perform bounds test
	 * @todo remove
	 */
	boost::shared_ptr<RobotRepresentation> getRobot(int n);

private:
	/**
	 * The population: the robots, with their fitness. If evaluated, this shall
	 * always be ordered from best to worst individual.
	 */
	std::vector<Individual>	robots_;

	/**
	 * Are all robots evaluated?
	 */
	bool evaluated_;
};

} /* namespace robogen */
#endif /* POPULATION_H_ */
