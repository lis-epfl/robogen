/*
 * @(#) DeterministicTournament.h   1.0   Sep 10, 2013
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

#ifndef DETERMINISTICTOURNAMENT_H_
#define DETERMINISTICTOURNAMENT_H_

#include <utility>
#include "evolution/engine/Selector.h"

namespace robogen{

class DeterministicTournament : public Selector {
public:
	/**
	 * Initializes a deterministic tournament that for each parent
	 * randomly draws n robots, the best robot getting the place
	 */
	DeterministicTournament(unsigned int tSize, boost::random::mt19937 &rng);

	/**
	 * Destructor
	 */
	virtual ~DeterministicTournament();

	/**
	 * Registers population
	 */
	virtual void initPopulation(boost::shared_ptr<Population> pop);

	/**
	 * Selects a parent from a population
	 * @param pop the old population
	 * @return the new population
	 */
	virtual bool select(boost::shared_ptr<RobotRepresentation> &selected);

private:
	/**
	 * Selection pool population
	 */
	boost::shared_ptr<Population> population_;

	/**
	 * Tournament size
	 */
	unsigned int tSize_;

	/**
	 * Random number generator reference
	 */
	boost::random::mt19937 &rng_;
};

}
#endif /* DETERMINISTICTOURNAMENT_H_ */
