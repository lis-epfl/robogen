/*
 * @(#) RacingScenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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
#include "config/RobogenConfig.h"
#include "config/StartPositionConfig.h"
#include "scenario/Environment.h"
#include "scenario/RacingScenario.h"
#include "Robot.h"
#include "Models.h"

namespace robogen {

RacingScenario::RacingScenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		Scenario(robogenConfig), curTrial_(0) {

	this->setEnvironment(boost::shared_ptr<Environment>(new Environment()));

}

RacingScenario::~RacingScenario() {

}

bool RacingScenario::setupSimulation() {

	// Compute robot ending position
	startPosition_.push_back(this->getRobot()->getCoreComponent()->getRootPosition());

	return true;

}

bool RacingScenario::afterSimulationStep() {

	return true;
}

bool RacingScenario::endSimulation() {

	// Compute robot ending position
	endPosition_.push_back(this->getRobot()->getCoreComponent()->getRootPosition());
	curTrial_++;
	// Set next starting position
	this->setStartingPosition(curTrial_);
	return true;

}

double RacingScenario::getFitness() {

	double fitness = 0;
	for (unsigned int i = 0; i < startPosition_.size(); ++i) {
		osg::Vec3 temp = endPosition_[i] - startPosition_[i];
		fitness += temp.length();
	}

	return fitness/startPosition_.size();
}

bool RacingScenario::remainingTrials() {
	boost::shared_ptr<StartPositionConfig> startPos = this->getRobogenConfig()->getStartingPos();
	return curTrial_ < startPos->getStartPosition().size();
}

}
