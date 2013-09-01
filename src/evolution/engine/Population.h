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
#include "evolution/representation/RobotRepresentation.h"
#include "utils/network/TcpSocket.h"

namespace robogen {

class Population {
public:
	/**
	 * Constructs a population from a given robot body, keeping the body
	 * constant, but randomly initializing the brain.
	 */
	Population(RobotRepresentation &robot, int popSize);

	virtual ~Population();

	/**
	 * Evaluate the population using the given sockets to transmit the robot
	 * to simulator instances in parallel. Then order the population by fitness.
	 */
	void evaluate(std::vector<TcpSocket> &sockets);

	/**
	 * Only for debugging purposes: Obtain some robot representation
	 * @param n robot to get
	 * @note does not perform bounds test
	 */
	boost::shared_ptr<RobotRepresentation> getRobot(int n);

private:
	/**
	 * The population: the robots, with their fitness. If evaluated, this shall
	 * always be ordered from best to worst individual.
	 */
	std::vector<std::pair<boost::shared_ptr<RobotRepresentation>, double> >
	robots_;

	/**
	 * Are the robots evaluated?
	 */
	bool evaluated_;
};

} /* namespace robogen */
#endif /* POPULATION_H_ */
