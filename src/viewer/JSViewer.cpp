/*
 * @(#) FileViewerLog.h   1.0   Aug 20, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Titus Cieslweski, Joshua Auerbach
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

#include <viewer/JSViewer.h>
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

#ifdef EMSCRIPTEN
#include "emscripten.h"
#include <emscripten/bind.h>


using namespace emscripten;
#endif

void sendJSEvent(std::string name, std::string jsonData);

namespace robogen {

JSViewer::JSViewer() {
	sendJSEvent("ViewerInit", "null");
}

JSViewer::~JSViewer() {
	sendJSEvent("ViewerDestructed", "null");
}

} /* namespace robogen */

bool robogen::JSViewer::configureScene(
		std::vector<boost::shared_ptr<Model> > bodyParts,
		boost::shared_ptr<Scenario> scenario) {
	sendJSEvent("configuration received", "null");
	this->innerLogger = boost::shared_ptr<WebGLLogger>(new WebGLLogger("", scenario));
	sendJSEvent("structure", this->innerLogger->getStructureJSON());
	sendJSEvent("obstaclesDefinition", this->innerLogger->getObstaclesDefinitionJSON());
	sendJSEvent("lights", this->innerLogger->getLightsJSON());
	return true;
}

bool robogen::JSViewer::done() {
	return false;
}

bool robogen::JSViewer::frame(double simulatedTime,
		unsigned int numTimeSteps) {
	this->innerLogger->log(simulatedTime);
	sendJSEvent("frameLog", this->innerLogger->getLastLogJSON());
	return true;
}

bool robogen::JSViewer::isPaused() {
	return true;
}
