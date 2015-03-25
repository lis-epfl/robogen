#include "viewer/WebGLLogger.h"
#include <iostream>
#include "Robot.h"

using namespace boost;
using namespace std;

namespace robogen {
	WebGLLogger::WebGLLogger() {
		this->fileHandler.open(fileName.c_str());
		this->_instance = shared_ptr<WebGLLogger>(this);
	}

	shared_ptr<WebGLLogger> WebGLLogger::getInstance() {
		if (!_instance) {
			_instance = new WebGLLogger();
		} 
		return _instance;
	}

	void WebGLLogger::setFileName(string in_filename) {
		fileName = in_filename;
	}

	void WebGLLogger::logRobot(boost::shared_ptr<Robot> robot, double time) {
		cout << fileName << endl;
	}
}
