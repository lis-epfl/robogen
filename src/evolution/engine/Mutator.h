/*
 * @(#) Mutator.h   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2016 Titus Cieslewski, Joshua Auerbach
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

#ifndef MUTATOR_H_
#define MUTATOR_H_

// enable the following to perform body mutation:
// #define BODY_MUTATION

#include <boost/random/normal_distribution.hpp>
#include <boost/random/bernoulli_distribution.hpp>

#include "config/EvolverConfiguration.h"
#include "evolution/representation/RobotRepresentation.h"
#include "evolution/engine/BodyVerifier.h"

//#define DEBUG_MUTATE

#define PRINT_ERRORS (false)

#ifdef DEBUG_MUTATE
#define PRINT_ERRORS (true)
#endif


#define MAX_MUTATION_ATTEMPTS 100 //TODO move this somewhere else
namespace robogen {

class Mutator{
public:
	/**
	 * Performs mutation and crossover on a pair of robots
	 */
	virtual std::vector<boost::shared_ptr<RobotRepresentation> > createOffspring(
				boost::shared_ptr<RobotRepresentation> parent1,
				boost::shared_ptr<RobotRepresentation> parent2 =
						boost::shared_ptr<RobotRepresentation>()) = 0;

	virtual void growBodyRandomly(boost::shared_ptr<RobotRepresentation>& robot) = 0;
	virtual void randomizeBrain(boost::shared_ptr<RobotRepresentation>& robot) = 0;
};

}

#endif /* MUTATOR_H_ */
