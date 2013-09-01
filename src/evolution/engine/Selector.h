/*
 * @(#) Selector.h   1.0   Sep 1, 2013
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

#ifndef SELECTOR_H_
#define SELECTOR_H_

#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include "evolution/engine/Population.h"

namespace robogen {

/**
 * Class containing the logic of selection for an evaluated population.
 * @todo do different strategies with derived classes?
 */
class Selector {
public:
	/**
	 * Creates a ranking selector that selects the n best individuals
	 * and embeds them periodically into the next population.
	 * @param n amount of individuals to be selected
	 * @param rng random number generator to be used
	 */
	Selector(int n, boost::random::mt19937 &rng);

	/**
	 * Creates a new population from an old one according to the current
	 * strategy.
	 * @param pop the old population
	 * @return the new population
	 */
	boost::shared_ptr<Population> select(boost::shared_ptr<Population> pop);

	virtual ~Selector();

private:
	/**
	 * Amount of best individuals that are selected
	 */
	int nselected_;

	/**
	 * Random number generator to be used
	 */
	boost::random::mt19937 &rng_;
};

} /* namespace robogen */
#endif /* SELECTOR_H_ */
