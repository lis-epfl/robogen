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

#include <vector>
#include "evolution/engine/Selector.h"

namespace robogen {

Selector::Selector(int n, boost::random::mt19937 &rng) : nselected_(n),
		rng_(rng) {
}

Selector::~Selector() {
}

boost::shared_ptr<Population> Selector::select(
		boost::shared_ptr<Population> pop){
	// gets individuals that are guaranteed to be ordered descending by fitness
	std::vector<Individual> chosenOnes = pop->orderedEvaluatedRobots();
	for (unsigned int i = nselected_; i<chosenOnes.size(); i++){
		chosenOnes[i] = chosenOnes[i%nselected_];
	}
	return boost::shared_ptr<Population>(new Population(chosenOnes));
}

} /* namespace robogen */
