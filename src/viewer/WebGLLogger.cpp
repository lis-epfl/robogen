/*
 * @(#) WebGLLogger.cpp   1.0
 *
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Guillaume Leclerc, Joshua Auerbach
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

#include "viewer/WebGLLogger.h"
#include <Models.h>
#include <model/objects/BoxObstacle.h>
#include <model/objects/LightSource.h>
#include <utils/RobogenUtils.h>
#include <scenario/Terrain.h>
#include <iostream>
#include <jansson.h>
#include <boost/lexical_cast.hpp>
#include <osg/ShapeDrawable>
#include <osg/Quat>
#include <osg/Vec3>
#include "Robot.h"

// if using an older version of jannson
#ifndef JSON_REAL_PRECISION
#define JSON_REAL_PRECISION(n)  (((n) & 0x1F) << 11)
#endif

namespace robogen {

const char *WebGLLogger::STRUCTURE_TAG = "structure";
const char *WebGLLogger::LOG_TAG = "log";
const char *WebGLLogger::POSITION_TAG = "position";
const char *WebGLLogger::ATTITUDE_TAG = "attitude";
const char *WebGLLogger::REL_POS_TAG = "rel_position";
const char *WebGLLogger::REL_ATT_TAG = "rel_attitude";
const char *WebGLLogger::MESH_PATH = "filename";
const char *WebGLLogger::MAP_TAG = "map";
const char *WebGLLogger::MAP_DIM_TAG = "size";
const char *WebGLLogger::MAP_DATA_TAG = "data";
const char *WebGLLogger::OBSTACLE_TAGS = "obstacles";
const char *WebGLLogger::OBSTACLE_DEF_TAG = "definition";
const char *WebGLLogger::OBSTACLE_LOG_TAG = "log";
const char *WebGLLogger::LIGHT_TAGS = "lights";

WebGLLogger::WebGLLogger(std::string inFileName,
		boost::shared_ptr<Scenario> in_scenario, double targetFrameRate) :
		frameRate(targetFrameRate), lastFrame(-1000.0), robot(
				in_scenario->getRobot()), scenario(in_scenario), fileName(
				inFileName) {
	this->jsonRoot = json_object();
	this->jsonStructure = json_array();
	this->jsonLog = json_object();
	this->jsonMap = json_object();
	this->jsonObstacles = json_object();
	this->jsonLights = json_object();
	this->jsonObstaclesLog = json_object();
	this->jsonObstaclesDefinition = json_array();
	this->bodies = std::vector<struct BodyDescriptor>();
	this->writeObstaclesDefinition();
	this->generateBodyCollection();
	this->writeJSONHeaders();
	this->writeRobotStructure();
	this->generateMapInfo();
}

std::string WebGLLogger::getFormatedStringForCuboid(double width, double height,
		double thickness) {
	std::stringstream ss;
	ss << "cuboid(" << width << ", " << height << ", " << thickness << ")";
	return ss.str();
}
std::string WebGLLogger::getFormatedStringForCylinder(double radius,
		double height) {
	std::stringstream ss;
	ss << "cylinder(" << radius << ", " << height << ")";
	return ss.str();
}

void WebGLLogger::writeObstaclesDefinition() {
	std::vector<boost::shared_ptr<Obstacle> > obstacles =
			this->scenario->getEnvironment()->getObstacles();
	for (std::vector<boost::shared_ptr<Obstacle> >::iterator it =
			obstacles.begin(); it != obstacles.end(); ++it) {

		boost::shared_ptr<BoxObstacle> boxObstacle =
						boost::dynamic_pointer_cast<BoxObstacle> ((*it));

		if(!boxObstacle) {
			std::cerr << "Invalid obstacle!!" << std::endl;
			exitRobogen(EXIT_FAILURE);
		}

		json_t* json_size = json_array();
		json_array_append(this->jsonObstaclesDefinition, json_size);
		osg::Vec3 size = boxObstacle->getSize();
		json_array_append(json_size, json_real(size.x()));
		json_array_append(json_size, json_real(size.y()));
		json_array_append(json_size, json_real(size.z()));
	}
}

void WebGLLogger::generateBodyCollection() {
	for (size_t i = 0; i < this->robot->getBodyParts().size(); ++i) {
		boost::shared_ptr<Model> currentModel = this->robot->getBodyParts()[i];
		boost::shared_ptr<ParametricBrickModel> brickModel =
				boost::dynamic_pointer_cast<ParametricBrickModel>(currentModel);
		std::vector<int> ids = currentModel->getIDs();
		for (std::vector<int>::iterator it = ids.begin(); it != ids.end();
				++it) {
			std::string meshName;
			if (brickModel != 0) {
				switch (*it) {
				case ParametricBrickModel::B_CONNECTION_PART_ID:
					meshName = WebGLLogger::getFormatedStringForCuboid(
							brickModel->getConnectionLength(),
							ParametricBrickModel::CONNECTION_PART_WIDTH,
							ParametricBrickModel::CONNECTION_PART_THICKNESS);
					break;
				case ParametricBrickModel::B_CYLINDER_ID:
					meshName = WebGLLogger::getFormatedStringForCylinder(
							ParametricBrickModel::CYLINDER_RADIUS,
							ParametricBrickModel::CONNECTION_PART_WIDTH);
					break;
				case ParametricBrickModel::B_FIXED_BAR__ID:
					meshName = WebGLLogger::getFormatedStringForCuboid(
							ParametricBrickModel::FIXED_BAR_LENGTH,
							ParametricBrickModel::CONNECTION_PART_WIDTH,
							ParametricBrickModel::CONNECTION_PART_THICKNESS);
					break;
				case ParametricBrickModel::B_SLOT_A_ID:
				case ParametricBrickModel::B_SLOT_B_ID:
					meshName = WebGLLogger::getFormatedStringForCuboid(
							ParametricBrickModel::SLOT_THICKNESS,
							ParametricBrickModel::SLOT_WIDTH,
							ParametricBrickModel::SLOT_WIDTH);
					break;
				default:
					meshName = "";
				}
			} else {
				meshName = RobogenUtils::getMeshFile(currentModel, *it);
			}
			if (meshName.length() > 0) {
				struct BodyDescriptor desc;
				desc.meshName = meshName;
				desc.model = currentModel;
				desc.bodyId = *it;
				this->bodies.push_back(desc);
			}
		}
	}
}
WebGLLogger::~WebGLLogger() {
	json_dump_file(this->jsonRoot, this->fileName.c_str(), JSON_TAGS);
	json_decref(this->jsonRoot);
}

void WebGLLogger::writeRobotStructure() {
	for (std::vector<struct BodyDescriptor>::iterator it = this->bodies.begin();
			it != this->bodies.end(); ++it) {
		osg::Vec3 relativePosition = RobogenUtils::getRelativePosition(
				it->model, it->bodyId) / 1000;
		osg::Quat relativeAttitude = RobogenUtils::getRelativeAttitude(
				it->model, it->bodyId);

		// NOTE : we are still having issues with meshes displaying differently
		// in the web viewer.    Since there is no specific code for each model
		// in the webgl engine it is cleaner to differentiate this behavior
		// here.
		// todo : figure out cause and fix (this stuff should not be needed!)

		std::cout << RobogenUtils::getPartType(it->model) << " " <<
				it->bodyId;

		// we rotate 180 degrees around z for :
		//	both meshes of ActiveWheels, ActiveWhegs, Rotators
		//  slot of PassiveWheel,
		//  IrSensors, LightSensors

		if (	boost::dynamic_pointer_cast<ActiveWheelModel>(it->model) ||
				boost::dynamic_pointer_cast<ActiveWhegModel>(it->model) ||
				boost::dynamic_pointer_cast<RotateJointModel>(it->model) ||
				boost::dynamic_pointer_cast<IrSensorModel>(it->model) ||
				boost::dynamic_pointer_cast<LightSensorModel>(it->model) ||
				(boost::dynamic_pointer_cast<PassiveWheelModel>(it->model)
						&& it->bodyId == PassiveWheelModel::B_SLOT_ID)) {

			relativeAttitude *= osg::Quat(osg::inDegrees(180.0),
										  osg::Vec3(0, 0, 1));

			std::cout << " rotating!";
		}

		// then rotate 180 degrees around y for Active Wheel meshes
		// and Active Wheg meshes
		if (	(boost::dynamic_pointer_cast<ActiveWheelModel>(it->model)
				 && it->bodyId == ActiveWheelModel::B_WHEEL_ID) ||
				(boost::dynamic_pointer_cast<ActiveWhegModel>(it->model)
				&& it->bodyId == ActiveWhegModel::B_WHEG_BASE)) {

			relativeAttitude *= osg::Quat(osg::inDegrees(180.0),
										  osg::Vec3(0, 1, 0));

			std::cout << " rotating!";
		}

		// 180 degrees around x for LightSensors, IrSensors, ActiveHinge motors
		// PassiveHinges
		if (	boost::dynamic_pointer_cast<LightSensorModel>(it->model) ||
				boost::dynamic_pointer_cast<IrSensorModel>(it->model) ||
				(boost::dynamic_pointer_cast<ActiveHingeModel>(it->model) &&
				 it->bodyId == ActiveHingeModel::B_SLOT_B_ID) ||
				 boost::dynamic_pointer_cast<HingeModel>(it->model)) {

			relativeAttitude *= osg::Quat(osg::inDegrees(180.0),
										  osg::Vec3(1, 0, 0));

			std::cout << " rotating!";
		}



		std::cout << std::endl
				<< relativeAttitude.x() <<  " " << relativeAttitude.y() <<
				" " << relativeAttitude.z() << " " << relativeAttitude.w() <<
				std::endl;



		json_t* obDescriptor = json_object();
		json_t* relAttitude = json_array();
		json_t* relPosition = json_array();
		json_object_set_new(obDescriptor, WebGLLogger::MESH_PATH,
				json_string(it->meshName.c_str()));

		json_array_append(relAttitude, json_real(relativeAttitude.x()));
		json_array_append(relAttitude, json_real(relativeAttitude.y()));
		json_array_append(relAttitude, json_real(relativeAttitude.z()));
		json_array_append(relAttitude, json_real(relativeAttitude.w()));
		json_object_set_new(obDescriptor, WebGLLogger::REL_ATT_TAG,
				relAttitude);

		json_array_append(relPosition, json_real(relativePosition.x()));
		json_array_append(relPosition, json_real(relativePosition.y()));
		json_array_append(relPosition, json_real(relativePosition.z()));
		json_object_set_new(obDescriptor, WebGLLogger::REL_POS_TAG,
				relPosition);

		json_array_append(this->jsonStructure, obDescriptor);
	}
}

void WebGLLogger::writeJSONHeaders() {
	json_object_set_new(this->jsonRoot, WebGLLogger::LOG_TAG, this->jsonLog);
	json_object_set_new(this->jsonRoot, WebGLLogger::STRUCTURE_TAG,
			this->jsonStructure);
	json_object_set_new(this->jsonRoot, WebGLLogger::MAP_TAG, this->jsonMap);
	json_object_set_new(this->jsonRoot, WebGLLogger::OBSTACLE_TAGS,
			this->jsonObstacles);
	json_object_set_new(this->jsonObstacles, WebGLLogger::OBSTACLE_DEF_TAG,
			this->jsonObstaclesDefinition);
	json_object_set_new(this->jsonObstacles, WebGLLogger::OBSTACLE_LOG_TAG,
			this->jsonObstaclesLog);
	json_object_set_new(this->jsonRoot, WebGLLogger::LIGHT_TAGS,
			this->jsonLights);
}

void WebGLLogger::generateMapInfo() {
	boost::shared_ptr<Terrain> terrain = scenario->getEnvironment()->getTerrain();
	json_t *dims = json_array();
	json_object_set_new(this->jsonMap, WebGLLogger::MAP_DIM_TAG, dims);
	json_array_append(dims, json_real(terrain->getWidth()));
	json_array_append(dims, json_real(terrain->getDepth()));

	if (terrain->getType() == TerrainConfig::ROUGH) {
		json_array_append(dims, json_real(terrain->getHeightFieldHeight()));
		json_t *mapData = json_array();
		json_object_set_new(this->jsonMap, WebGLLogger::MAP_DATA_TAG, mapData);
		unsigned int cols = terrain->getHeightFieldData()->s();
		unsigned int rows = terrain->getHeightFieldData()->t();
		for (unsigned int i = 0; i < cols; ++i) {
			json_t *current_row = json_array();
			json_array_append(mapData, current_row);
			for (unsigned int j = 0; j < rows; ++j) {
				int value =
						static_cast<int>(*terrain->getHeightFieldData()->data(i,
								j));
				json_array_append(current_row, json_integer(value));
			}
		}
	} else {
		json_array_append(dims, json_real(0));
	}
}

std::string WebGLLogger::getStructureJSON() {
	char * res = json_dumps(this->jsonRoot, JSON_TAGS);
	std::string result(res);
	free(res);
	return result;
}

std::string WebGLLogger::getObstaclesDefinitionJSON() {
	char * res = json_dumps(this->jsonObstaclesDefinition, JSON_TAGS);
	std::string result(res);
	free(res);
	return result;
}

std::string WebGLLogger::getLastLogJSON() {
	std::string obKey = boost::lexical_cast<std::string>(this->lastFrame);
	json_t* lastRobotLog = json_object_get(this->jsonLog, obKey.c_str());
	json_t* lastObstaclesLog = json_object_get(this->jsonObstaclesLog,
			obKey.c_str());
	json_t* res = json_object();
	json_object_set_new(res, "time", json_real(this->lastFrame));
	if (lastRobotLog != NULL) {
		json_object_set(res, "robot", lastRobotLog);
	}
	if (lastObstaclesLog) {
		json_object_set(res, "obstacles", lastObstaclesLog);
	}

	char * resChar = json_dumps(res, JSON_TAGS);
	json_decref(res);
	std::string result(resChar);
	free(resChar);
	return result;
}

std::string WebGLLogger::getLightsJSON() {
	char * res = json_dumps(this->jsonLights, JSON_TAGS);
	std::string result(res);
	free(res);
	return result;
}

void WebGLLogger::log(double dt) {
	if (dt - lastFrame >= 1.0 / frameRate) {
		std::string obKey = boost::lexical_cast < std::string > (dt);
		json_t* positions = json_array();
		json_object_set_new(this->jsonLog, obKey.c_str(), positions);

		for (std::vector<struct BodyDescriptor>::iterator it =
				this->bodies.begin(); it != this->bodies.end(); ++it) {
			json_t* bodyLog = json_array();
			json_t* attitude = json_array();
			json_t* position = json_array();
			json_array_append_new(bodyLog, attitude);
			json_array_append_new(bodyLog, position);
			json_array_append_new(positions, bodyLog);

			osg::Vec3 currentPosition = it->model->getBodyPosition(it->bodyId);
			osg::Quat currentAttitude = it->model->getBodyAttitude(it->bodyId);

			json_array_append_new(position, json_real(currentPosition.x()));
			json_array_append_new(position, json_real(currentPosition.y()));
			json_array_append_new(position, json_real(currentPosition.z()));

			json_array_append_new(attitude, json_real(currentAttitude.x()));
			json_array_append_new(attitude, json_real(currentAttitude.y()));
			json_array_append_new(attitude, json_real(currentAttitude.z()));
			json_array_append_new(attitude, json_real(currentAttitude.w()));

		}

		std::vector<boost::shared_ptr<Obstacle> > obstacles =
				this->scenario->getEnvironment()->getObstacles();
		json_t* obstacles_positions = json_array();
		json_object_set_new(this->jsonObstaclesLog, obKey.c_str(),
				obstacles_positions);
		for (std::vector<boost::shared_ptr<Obstacle> >::iterator it =
				obstacles.begin(); it != obstacles.end(); ++it) {
			json_t* obstacleLog = json_array();
			json_t* attitude = json_array();
			json_t* position = json_array();
			json_array_append_new(obstacleLog, attitude);
			json_array_append_new(obstacleLog, position);
			json_array_append_new(obstacles_positions, obstacleLog);

			osg::Vec3 currentPosition = (*it)->getPosition();
			osg::Quat currentAttitude = (*it)->getAttitude();

			json_array_append_new(position, json_real(currentPosition.x()));
			json_array_append_new(position, json_real(currentPosition.y()));
			json_array_append_new(position, json_real(currentPosition.z()));

			json_array_append_new(attitude, json_real(currentAttitude.x()));
			json_array_append_new(attitude, json_real(currentAttitude.y()));
			json_array_append_new(attitude, json_real(currentAttitude.z()));
			json_array_append_new(attitude, json_real(currentAttitude.w()));
		}

		lastFrame = dt;

		std::vector < boost::shared_ptr<LightSource> > lights =
				this->scenario->getEnvironment()->getLightSources();
		json_t* jsonFrameLights = json_array();
		json_object_set_new(this->jsonLights, obKey.c_str(), jsonFrameLights);

		for (std::vector<boost::shared_ptr<LightSource> >::iterator light =
				lights.begin(); light != lights.end(); ++light) {
			json_t* jsonLightCoordinates = json_array();
			json_array_append_new(jsonFrameLights, jsonLightCoordinates);
			osg::Vec3 coordinates = (*light)->getPosition();

			json_array_append_new(jsonLightCoordinates,
					json_real(coordinates.x()));
			json_array_append_new(jsonLightCoordinates,
					json_real(coordinates.y()));
			json_array_append_new(jsonLightCoordinates,
					json_real(coordinates.z()));
		}
	}
}
}
