#include "viewer/WebGLLogger.h"
#include <Models.h>
#include <utils/RobogenUtils.h>
#include <iostream>
#include <jansson.h>
#include "Robot.h"

namespace robogen {

const char *WebGLLogger::STRUCTURE_TAG = "structure";
const char *WebGLLogger::LOG_TAG = "log";

WebGLLogger::WebGLLogger(std::string inFileName,
		boost::shared_ptr<Robot> inRobot) :
		fileName(inFileName) {
	this->robot = inRobot;
	this->jsonRoot = json_object();
	this->jsonStructure = json_array();
	this->jsonLog = json_array();
	this->generateBodyCollection();
	this->writeJSONHeaders();
	this->writeRobotStructure();
}

void WebGLLogger::generateBodyCollection() {
	for(size_t i = 0 ; i < this->robot->getBodyParts().size(); ++i) {
		boost::shared_ptr<Model> currentModel = this->robot->getBodyParts()[i];
		std::vector<int> ids = currentModel->getIDs();
		for(std::vector<int>::iterator it = ids.begin();
				it != ids.end(); ++it) {
			struct BodyDescriptor desc;
			desc.model = currentModel;
			desc.bodyId = *it;
			this->bodies.push_back(desc);
		}
	}
}

WebGLLogger::~WebGLLogger() {
	json_dump_file(this->jsonRoot, this->fileName.c_str(), 0);
	std::cout << "Should have been written to disk" << std::endl;
	json_decref(this->jsonRoot);
}

void WebGLLogger::writeRobotStructure() {
	for (std::vector<struct BodyDescriptor>::iterator it =
			this->bodies.begin(); it != this->bodies.end(); ++it) {
		json_array_append(this->jsonStructure,
				json_string(RobogenUtils::getMeshFile(it->model,
						it->bodyId).c_str()));
	}
}

void WebGLLogger::writeJSONHeaders() {
	json_object_set_new(this->jsonRoot, WebGLLogger::LOG_TAG,
			this->jsonLog);
	json_object_set_new(this->jsonRoot, WebGLLogger::STRUCTURE_TAG,
			this->jsonStructure);
}

}
