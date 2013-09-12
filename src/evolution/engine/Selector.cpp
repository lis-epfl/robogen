/*
 * @(#) Selector.cpp   1.0   Sep 1, 2013
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

#include <vector>
#include <iostream>
#include "evolution/engine/Selector.h"

namespace robogen {

Selector::Selector(int n, boost::random::mt19937 &rng) : nselected_(n),
		rng_(rng) {
}

void Selector::initPopulation(boost::shared_ptr<Population> pop){

}

Selector::~Selector() {
}

std::pair<Individual, Individual> Selector::select(){
	iterator_ %= nselected_;
	std::pair<Individual, Individual> retpair;
	retpair.first = preselection_[iterator_++%nselected_];
	retpair.second = preselection_[iterator_++%nselected_];
	return retpair;
}

} /* namespace robogen */
