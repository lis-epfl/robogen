/*
 * @(#) Population.cpp   1.0   Sep 1, 2013
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

#include "evolution/engine/Population.h"

namespace robogen {

Population::Population(RobotRepresentation &robot, int popSize) {
	// fill population vector
	for (int i=0; i<popSize; i++){
		robots_.push_back(std::pair<boost::shared_ptr<RobotRepresentation>,
				double>(boost::shared_ptr<RobotRepresentation>(
						new RobotRepresentation(robot)),0.));
		robots_.back().first->randomizeBrain();
	}
}

Population::~Population() {
}

boost::shared_ptr<RobotRepresentation> Population::getRobot(int n){
	return robots_[n].first;
}

} /* namespace robogen */
