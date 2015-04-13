#include "viewer/WebGLLogger.h"
#include <Models.h>
#include <utils/RobogenUtils.h>
#include <iostream>
#include <jansson.h>
#include <boost/lexical_cast.hpp>
#include <osg/Quat>
#include <osg/Vec3>
#include "render/Mesh.h"
#include "Robot.h"
#include <osg/Node>
#include <osgDB/ReadFile>



namespace robogen {

const char *WebGLLogger::STRUCTURE_TAG = "structure";
const char *WebGLLogger::LOG_TAG = "log";
const char *WebGLLogger::POSITION_TAG = "position";
const char *WebGLLogger::ATTITUDE_TAG = "attitude";
const char *WebGLLogger::REL_POS_TAG = "rel_position";
const char *WebGLLogger::REL_ATT_TAG = "rel_attitude";
const char *WebGLLogger::MESH_PATH = "filename";
const char *WebGLLogger::CENTER_SHIFT_TAG = "center";

WebGLLogger::WebGLLogger(std::string inFileName,
		boost::shared_ptr<Robot> inRobot) :
		robot(inRobot), fileName(inFileName) {
	this->jsonRoot = json_object();
	this->jsonStructure = json_array();
	this->jsonLog = json_object();
	this->generateBodyCollection();
	this->writeJSONHeaders();
	this->writeRobotStructure();
}

void WebGLLogger::generateBodyCollection() {
	int nbMeshesIgnored = 0;
	for (size_t i = 0; i < this->robot->getBodyParts().size(); ++i) {
		boost::shared_ptr<Model> currentModel = this->robot->getBodyParts()[i];
		std::vector<int> ids = currentModel->getIDs();
		for (std::vector<int>::iterator it = ids.begin(); it != ids.end();
				++it) {
			std::string meshName = RobogenUtils::getMeshFile(currentModel, *it);
			if (meshName.length() > 0) {
				struct BodyDescriptor desc;
				desc.model = currentModel;
				desc.bodyId = *it;
				this->bodies.push_back(desc);
			} else {
				++nbMeshesIgnored;
			}
		}
	}

	std::cout << nbMeshesIgnored << " have an empty string as mesh"
			<< std::endl;
}
WebGLLogger::~WebGLLogger() {
	json_dump_file(this->jsonRoot, this->fileName.c_str(), 0);
	json_decref(this->jsonRoot);
}

void WebGLLogger::writeRobotStructure() {
	for (std::vector<struct BodyDescriptor>::iterator it = this->bodies.begin();
			it != this->bodies.end(); ++it) {
		osg::Vec3 relativePosition = RobogenUtils::getRelativePosition(
				it->model, it->bodyId) / 1000;
		osg::Quat relativeAttitude = RobogenUtils::getRelativeAttitude(
				it->model, it->bodyId);
		json_t* obDescriptor = json_object();
		json_t* relAttitude = json_array();
		json_t* relPosition = json_array();
		json_t* shiftPos = json_array();
		std::string filename = RobogenUtils::getMeshFile(it->model, it->bodyId);

		json_object_set_new(obDescriptor, WebGLLogger::MESH_PATH,
				json_string(filename.c_str()));

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

		osg::Node* node(osgDB::readNodeFile(filename));
		osg::Vec3 shiftPosition(node->getBound().center());
		json_array_append(shiftPos, json_real(shiftPosition.x()));
		json_array_append(shiftPos, json_real(shiftPosition.y()));
		json_array_append(shiftPos, json_real(shiftPosition.z()));
		json_object_set_new(obDescriptor, WebGLLogger::CENTER_SHIFT_TAG, shiftPos);

		json_array_append(this->jsonStructure, obDescriptor);
	}
}

void WebGLLogger::writeJSONHeaders() {
	json_object_set_new(this->jsonRoot, WebGLLogger::LOG_TAG, this->jsonLog);
	json_object_set_new(this->jsonRoot, WebGLLogger::STRUCTURE_TAG,
			this->jsonStructure);
}

void WebGLLogger::log(double dt) {
	std::string obKey = boost::lexical_cast<std::string>(dt);
	json_t* positions = json_array();
	json_object_set_new(this->jsonLog, obKey.c_str(), positions);

	for (std::vector<struct BodyDescriptor>::iterator it = this->bodies.begin();
			it != this->bodies.end(); ++it) {
		json_t* bodyLog = json_object();
		json_t* attitude = json_array();
		json_t* position = json_array();
		json_object_set_new(bodyLog, WebGLLogger::ATTITUDE_TAG, attitude);
		json_object_set_new(bodyLog, WebGLLogger::POSITION_TAG, position);
		json_array_append(positions, bodyLog);

		osg::Vec3 currentPosition = it->model->getBodyPosition(it->bodyId);
		osg::Quat currentAttitude = it->model->getBodyAttitude(it->bodyId);

		json_array_append(position, json_real(currentPosition.x()));
		json_array_append(position, json_real(currentPosition.y()));
		json_array_append(position, json_real(currentPosition.z()));

		json_array_append(attitude, json_real(currentAttitude.x()));
		json_array_append(attitude, json_real(currentAttitude.y()));
		json_array_append(attitude, json_real(currentAttitude.z()));
		json_array_append(attitude, json_real(currentAttitude.w()));

	}
}

}
