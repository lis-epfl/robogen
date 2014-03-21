/*
 * @(#) TouchSensorRenderModel.cpp   1.0   Feb 27, 2013
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
#include <osgDB/ReadFile>

#include "render/callback/BodyCallback.h"
#include "render/components/TouchSensorRenderModel.h"
#include "render/Mesh.h"

namespace robogen {

TouchSensorRenderModel::TouchSensorRenderModel(
		boost::shared_ptr<TouchSensorModel> model) :
		RenderModel(model) {
	this->partA_.reset(new Mesh());
}

TouchSensorRenderModel::~TouchSensorRenderModel() {

}

bool TouchSensorRenderModel::initRenderModel() {

	bool meshLoadingA = this->partA_->loadMesh("../models/TouchSensor.stl");

	if (!meshLoadingA) {
		std::cerr << "[TouchSensorRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
		return true;
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> partA =
			this->partA_->getMesh();
	partA->setAttitude(osg::Quat(osg::inDegrees(-90.0), osg::Vec3(0, 0, 1)));
	partA->setPosition(
			fromOde(
					osg::Vec3(
							TouchSensorModel::SENSOR_BASE_THICKNESS / 2
									+ TouchSensorModel::SENSOR_THICKNESS / 2
									- 0.001, 0, 0)));

	partA_->setColor(osg::Vec4(1, 0, 0, 1));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getRootNode()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(),
					TouchSensorModel::B_SENSOR_BASE_ID));

	return true;

}

void TouchSensorRenderModel::showDebugView() {

	this->attachBox(TouchSensorModel::B_SENSOR_BASE_ID,
			TouchSensorModel::SENSOR_BASE_THICKNESS,
			TouchSensorModel::SENSOR_BASE_WIDTH,
			TouchSensorModel::SENSOR_BASE_WIDTH);

	this->attachBox(TouchSensorModel::B_SENSOR_LEFT,
			TouchSensorModel::SENSOR_THICKNESS, TouchSensorModel::SENSOR_WIDTH,
			TouchSensorModel::SENSOR_HEIGHT);

	this->attachBox(TouchSensorModel::B_SENSOR_RIGHT,
			TouchSensorModel::SENSOR_THICKNESS, TouchSensorModel::SENSOR_WIDTH,
			TouchSensorModel::SENSOR_HEIGHT);

}

void TouchSensorRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
}

}
