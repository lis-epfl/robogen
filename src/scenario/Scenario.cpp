/*
 * @(#) Scenario.cpp   1.0   Mar 13, 2013
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
#include <iostream>
#include "config/RobogenConfig.h"
#include "config/TerrainConfig.h"
#include "model/objects/BoxObstacle.h"
#include "scenario/Scenario.h"
#include "scenario/Terrain.h"
#include "Robot.h"

namespace robogen {

Scenario::Scenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		robogenConfig_(robogenConfig), startPositionId_(0) {

}

Scenario::~Scenario() {

}

bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace,
		boost::shared_ptr<Robot> robot) {

	odeWorld_ = odeWorld;
	odeSpace_ = odeSpace;

	robot_ = robot;

	// Setup terrain
	boost::shared_ptr<TerrainConfig> terrainConfig =
			robogenConfig_->getTerrainConfig();

	terrain_.reset(new Terrain(odeWorld_, odeSpace_));
	if (terrainConfig->isFlat()) {
		terrain_->initFlat(terrainConfig->getLength(),
				terrainConfig->getWidth());
	} else {
		terrain_->initRough(terrainConfig->getHeightFieldFileName(),
				terrainConfig->getLength(), terrainConfig->getWidth(),
				terrainConfig->getHeight());
	}

	// Setup robot position
	double minX = 0;
	double maxX = 0;
	double minY = 0;
	double maxY = 0;
	double minZ = 0;
	double maxZ = 0;

	// Starting position and orientation
	osg::Vec2 startingPosition =
			robogenConfig_->getStartingPos()->getStartPosition(
					startPositionId_)->getPosition();
	float startingAzimuth = robogenConfig_->getStartingPos()->getStartPosition(
			startPositionId_)->getAzimuth();
	osg::Quat roboRot;
	roboRot.makeRotate(osg::inDegrees(startingAzimuth), osg::Vec3(0,0,1));

	robot->rotateRobot(roboRot);
	robot->getBB(minX, maxX, minY, maxY, minZ, maxZ);
	robot->translateRobot(
			osg::Vec3(startingPosition.x() - (maxX - minX) / 2,
					startingPosition.y() - (maxY - minY) / 2,
					terrainConfig->getHeight() + inMm(2) - minZ));
	robot->getBB(minX, maxX, minY, maxY, minZ, maxZ);

	std::cout
			<< "The robot is enclosed in the AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
			<< minX << ", " << maxX << ", " << minY << ", " << maxY << ", "
			<< minZ << ", " << maxZ << ")" << std::endl;
	std::cout << "Obstacles in this range will not be generated" << std::endl << std::endl;

	// Setup obstacles
	boost::shared_ptr<ObstaclesConfig> obstacles =
			robogenConfig_->getObstaclesConfig();

	// Instance the boxes above the maximum terrain height
	const std::vector<osg::Vec2>& c = obstacles->getCoordinates();
	const std::vector<osg::Vec3>& s = obstacles->getSize();
	const std::vector<float>& d = obstacles->getDensity();
	for (unsigned int i = 0; i < c.size(); ++i) {

		float oMinX = c[i].x() - s[i].x() / 2;
		float oMaxX = c[i].x() + s[i].x() / 2;
		float oMinY = c[i].y() - s[i].y() / 2;
		float oMaxY = c[i].y() + s[i].y() / 2;

		// Do not insert the obstacle if it is in the robot range
		bool inRangeX = false;
		if ((oMinX <= minX && oMaxX >= maxX) || (oMinX >= minX && oMinX <= maxX)
				|| (oMaxX >= minX && oMaxX <= maxX)) {
			inRangeX = true;
		}

		bool inRangeY = false;
		if ((oMinY <= minY && oMaxY >= maxY) || (oMinY >= minY && oMinY <= maxY)
				|| (oMaxY >= minY && oMaxY <= maxY)) {
			inRangeY = true;
		}

		// Do not insert obstacles in the robot range
		if (!(inRangeX && inRangeY)) {
			osg::Vec3 position(c[i].x(), c[i].y(), s[i].z()/2);
			obstacles_.push_back(
					boost::shared_ptr<BoxObstacle>(
							new BoxObstacle(odeWorld_, odeSpace_, position,
									s[i], d[i])));
		}

	}

	return true;
}

void Scenario::prune(){
	odeWorld_ = 0;
	odeSpace_ = 0;
	robot_.reset();
	terrain_.reset();
	obstacles_.clear();
}

std::vector<boost::shared_ptr<BoxObstacle> > Scenario::getObstacles() {
	return obstacles_;
}

boost::shared_ptr<Terrain> Scenario::getTerrain() {
	return terrain_;
}

boost::shared_ptr<Robot> Scenario::getRobot() {
	return robot_;
}

boost::shared_ptr<RobogenConfig> Scenario::getRobogenConfig() {
	return robogenConfig_;
}

// TODO exception throwing? Would be robust...
void Scenario::setStartingPosition(int id) {
	startPositionId_ = id;
}

boost::shared_ptr<Environment> Scenario::getEnvironment() {
	return environment_;
}

void Scenario::setEnvironment(boost::shared_ptr<Environment> env) {
	environment_ = env;
}

}
