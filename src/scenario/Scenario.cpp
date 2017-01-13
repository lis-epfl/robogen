/*
 * @(#) Scenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#include "Environment.h"

namespace robogen {

Scenario::Scenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		robogenConfig_(robogenConfig), startPositionId_(0),
		stopSimulationNow_(false) {

}

Scenario::~Scenario() {

}

bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace,
		boost::shared_ptr<Robot> robot) {

	environment_ = boost::shared_ptr<Environment>(new
			Environment(odeWorld, odeSpace, robogenConfig_));

	if(!environment_->init()) {
		return false;
	}

	stopSimulationNow_ = false;


	robot_ = robot;

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
	robot->getAABB(minX, maxX, minY, maxY, minZ, maxZ);
	robot->translateRobot(
			osg::Vec3(startingPosition.x(),
					startingPosition.y(),
					robogenConfig_->getTerrainConfig()->getHeight()
						+ inMm(2) - minZ));
	robot->getAABB(minX, maxX, minY, maxY, minZ, maxZ);

	std::cout
			<< "The robot is enclosed in the AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
			<< minX << ", " << maxX << ", " << minY << ", " << maxY << ", "
			<< minZ << ", " << maxZ << ")" << std::endl;
	std::cout << "Obstacles in this range will not be generated" << std::endl << std::endl;

	// Setup obstacles
	boost::shared_ptr<ObstaclesConfig> obstacles =
			robogenConfig_->getObstaclesConfig();

	// Instance the boxes above the maximum terrain height
	const std::vector<osg::Vec3>& obstacleCoordinates = obstacles->getCoordinates();
	const std::vector<osg::Vec3>& obstacleSizes = obstacles->getSizes();
	const std::vector<float>& d = obstacles->getDensities();
	const std::vector<osg::Vec3>& rotationAxis = obstacles->getRotationAxes();
	const std::vector<float>& rotationAngles = obstacles->getRotationAngles();

	obstaclesRemoved_ = false;

	double overlapMaxZ=minZ;

	for (unsigned int i = 0; i < obstacleCoordinates.size(); ++i) {
		boost::shared_ptr<BoxObstacle> obstacle(
									new BoxObstacle(odeWorld, odeSpace,
											obstacleCoordinates[i],
											obstacleSizes[i], d[i], rotationAxis[i],
											rotationAngles[i]));
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);

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

		bool inRangeZ = false;
		if ((oMinZ <= minZ && oMaxZ >= maxZ) || (oMinZ >= minZ && oMinZ <= maxZ)
				|| (oMaxZ >= minZ && oMaxZ <= maxZ)) {
			inRangeZ = true;
		}

		// Do not insert obstacles in the robot range
		if (!(inRangeX && inRangeY && inRangeZ)) {
			environment_->addObstacle(obstacle);
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (oMaxZ > overlapMaxZ)
					overlapMaxZ = oMaxZ;
				environment_->addObstacle(obstacle);

			} else {
				obstacle->remove();
				obstaclesRemoved_ = true;
			}
		}

	}

	// Setup light sources
	boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
			robogenConfig_->getLightSourcesConfig();

	std::vector<boost::shared_ptr<LightSource> > lightSources;

	std::vector<osg::Vec3> lightSourcesCoordinates =
			lightSourcesConfig->getCoordinates();
	std::vector<float> lightSourcesIntensities =
				lightSourcesConfig->getIntensities();

	for (unsigned int i = 0; i < lightSourcesCoordinates.size(); ++i) {
		double lMinX = lightSourcesCoordinates[i].x() - LightSource::RADIUS;
		double lMaxX = lightSourcesCoordinates[i].x() + LightSource::RADIUS;
		double lMinY = lightSourcesCoordinates[i].y() - LightSource::RADIUS;
		double lMaxY = lightSourcesCoordinates[i].y() + LightSource::RADIUS;
		double lMinZ = lightSourcesCoordinates[i].z() - LightSource::RADIUS;
		double lMaxZ = lightSourcesCoordinates[i].z() + LightSource::RADIUS;

		// Do not insert the ligh source if it is in the robot range
		bool inRangeX = false;
		if ((lMinX <= minX && lMaxX >= maxX) || (lMinX >= minX && lMinX <= maxX)
				|| (lMaxX >= minX && lMaxX <= maxX)) {
			inRangeX = true;
		}

		bool inRangeY = false;
		if ((lMinY <= minY && lMaxY >= maxY) || (lMinY >= minY && lMinY <= maxY)
				|| (lMaxY >= minY && lMaxY <= maxY)) {
			inRangeY = true;
		}

		bool inRangeZ = false;
		if ((lMinZ <= minZ && lMaxZ >= maxZ) || (lMinZ >= minZ && lMinZ <= maxZ)
				|| (lMaxZ >= minZ && lMaxZ <= maxZ)) {
			inRangeZ = true;
		}


		if (!(inRangeX && inRangeY && inRangeZ)) {
			lightSources.push_back(boost::shared_ptr<LightSource>(
						new LightSource(odeSpace, lightSourcesCoordinates[i],
								lightSourcesIntensities[i])));
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (lMaxZ > overlapMaxZ)
					overlapMaxZ = lMaxZ;
				lightSources.push_back(boost::shared_ptr<LightSource>(
						new LightSource(odeSpace, lightSourcesCoordinates[i],
								lightSourcesIntensities[i])));
			} else {
				obstaclesRemoved_ = true;
			}
		}



	}
	environment_->setLightSources(lightSources);



	if (robogenConfig_->getObstacleOverlapPolicy() ==
			RobogenConfig::ELEVATE_ROBOT) {

		robot->translateRobot(
				osg::Vec3(0, 0,
						overlapMaxZ + inMm(2) - minZ));
	}




	// optimize the physics!  replace all fixed joints with composite bodies
	robot->optimizePhysics();

	return true;
}

boost::shared_ptr<StartPosition> Scenario::getCurrentStartPosition() {
	return robogenConfig_->getStartingPos()->getStartPosition(
			startPositionId_);
}

void Scenario::prune(){
	environment_.reset();
	robot_.reset();
}

boost::shared_ptr<Robot> Scenario::getRobot() {
	return robot_;
}

boost::shared_ptr<RobogenConfig> Scenario::getRobogenConfig() {
	return robogenConfig_;
}

void Scenario::setStartingPosition(int id) {
	startPositionId_ = id;
}

boost::shared_ptr<Environment> Scenario::getEnvironment() {
	return environment_;
}


}
