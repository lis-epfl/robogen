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

	// Compute robot start position,
	startPosition_.push_back(this->getCurrentStartPosition()->getPosition());

	return true;

}

bool RacingScenario::afterSimulationStep() {

	return true;
}

bool RacingScenario::endSimulation() {

	// Compute robot ending position from its closest part to the origin
	double minDistance = std::numeric_limits<double>::max();
	const std::vector<boost::shared_ptr<Model> >& bodyParts = this->getRobot()->getBodyParts();
	for (unsigned int i = 0; i < bodyParts.size(); ++i) {
		osg::Vec2 curBodyPos = osg::Vec2(bodyParts[i]->getRootPosition().x(), bodyParts[i]->getRootPosition().y());
		osg::Vec2 curDistance = startPosition_[startPosition_.size()-1] - curBodyPos;
		if (curDistance.length() < minDistance) {
			minDistance = curDistance.length();
		}
	}

	distances_.push_back(minDistance);
	curTrial_++;
	// Set next starting position
	this->setStartingPosition(curTrial_);
	return true;

}

double RacingScenario::getFitness() {
	double fitness = 1000000;
	for (unsigned int i = 0; i < distances_.size(); ++i) {
		if (distances_[i] < fitness)
			fitness = distances_[i];
	}

	return fitness;
}

bool RacingScenario::remainingTrials() {
	boost::shared_ptr<StartPositionConfig> startPos = this->getRobogenConfig()->getStartingPos();
	return curTrial_ < startPos->getStartPosition().size();
}

int RacingScenario::getCurTrial() const {
	return curTrial_;
}

}
