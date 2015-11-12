/*
 * @(#) DeterministicTournament.cpp   1.0   Sep 10, 2013
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

#include "DeterministicTournament.h"
#include <algorithm>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

namespace robogen {

DeterministicTournament::DeterministicTournament(unsigned int tSize,
		boost::random::mt19937 &rng) :
		tSize_(tSize), rng_(rng) {
}

DeterministicTournament::~DeterministicTournament() {
}

void DeterministicTournament::initPopulation(
		boost::shared_ptr<Population> pop) {
	population_ = pop;
}

// http://sureshamrita.wordpress.com/2011/08/27/random_shuffle-boost-generator/
bool DeterministicTournament::select(boost::shared_ptr<RobotRepresentation>
								&selected) {

	if (!population_) {
		std::cout << "Trying to perform selection, but no "
				"population initiated!" << std::endl;
		return false;
	}

	// prepare stuff for random_shuffle
	boost::uniform_int<> uni_dist;
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randomNumber(
			rng_, uni_dist);

	// prepare a pointer vector for a random shuffle
	std::vector<boost::shared_ptr<RobotRepresentation> > shuffVec;
	for (unsigned int i = 0; i < population_->size(); i++) {
		shuffVec.push_back(population_->at(i));
	}
	// let tSize_ robots "compete"

	unsigned int selectionIndex = 0;
	// shuffle robots -> first tSize are in tournament
	std::random_shuffle(shuffVec.begin(), shuffVec.end(), randomNumber);
	for (unsigned int i = 1; i < tSize_; i++) {
		if (shuffVec[i]->getFitness()
				> shuffVec[selectionIndex]->getFitness()) {
			selectionIndex = i;
		}
	}

	selected = shuffVec[selectionIndex];


	return true;
}

}

