/*
 * @(#) DeterministicTournament.cpp   1.0   Sep 10, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014
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

#include "evolution/engine/Selectors/DeterministicTournament.h"
#include <algorithm>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

namespace robogen{

DeterministicTournament::DeterministicTournament(unsigned int tSize,
		boost::random::mt19937 &rng) : tSize_(tSize), rng_(rng){
}

DeterministicTournament::~DeterministicTournament() {
}

void DeterministicTournament::initPopulation(boost::shared_ptr<Population> pop){
	population_ = pop;
}

// http://sureshamrita.wordpress.com/2011/08/27/random_shuffle-boost-generator/
bool DeterministicTournament::select(boost::shared_ptr<std::pair<
		RobotRepresentation, RobotRepresentation> > &selected){
	if (!population_){
		std::cout << "Trying to perform selection, but no "\
				"population initiated!" << std::endl;
		return false;
	}
	RobotRepresentation *selection[] = {NULL,NULL};
	// prepare stuff for random_shuffle
	boost::uniform_int<> uni_dist;
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> >
	randomNumber(rng_, uni_dist);
	// prepare a pointer vector for a random shuffle
	std::vector<RobotRepresentation*> shuffVec;
	for (unsigned int i=0; i<population_->size(); i++){
		shuffVec.push_back(&population_->at(i));
	}
	// for both parents, let tSize_ robots "compete"
	for (int p=0; p<2; p++){
		unsigned int selectionIndex = 0;
		// shuffle robots -> first tSize are in tournament
		std::random_shuffle(shuffVec.begin(), shuffVec.end(), randomNumber);
		for (unsigned int i=1; i<tSize_; i++){
			if (shuffVec[i]->getFitness() >
			shuffVec[selectionIndex]->getFitness()){
				selectionIndex = i;
			}
		}
		selection[p] = shuffVec[selectionIndex];
		// remove selected parent from robots competing for other position
		shuffVec.erase(shuffVec.begin() + selectionIndex);
	}
	selected = boost::shared_ptr<std::pair<RobotRepresentation,
			RobotRepresentation> > (new std::pair<RobotRepresentation,
					RobotRepresentation>(RobotRepresentation(*selection[0]),
							RobotRepresentation(*selection[1])));
	return true;
}

}

