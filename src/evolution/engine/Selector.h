/*
 * @(#) Selector.h   1.0   Sep 1, 2013
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

#ifndef SELECTOR_H_
#define SELECTOR_H_

#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include "evolution/engine/Population.h"
#include "evolution/representation/RobotRepresentation.h"

namespace robogen {

/**
 * Selector interface definition
 */
class Selector {
public:
	Selector(){

	}

	/**
	 * Allows selector to perform pre-selection actions, i.e. select pool from
	 * which selected individuals shall stem.
	 */
	virtual void initPopulation(boost::shared_ptr<Population> pop){
		pop = boost::shared_ptr<Population>();
	}

	/**
	 * Select a parents from a population
	 * @param pop the old population
	 * @return the new population
	 */
	virtual bool select(boost::shared_ptr<RobotRepresentation> &selected) = 0;

	virtual ~Selector(){

	}
};

} /* namespace robogen */
#endif /* SELECTOR_H_ */
