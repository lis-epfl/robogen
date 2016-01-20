/*
 * @(#) LightSensorRenderModel.cpp   1.0   Feb 9, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Andrea Maesani, Joshua Auerbach
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
#include "render/components/LightSensorRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

LightSensorRenderModel::LightSensorRenderModel(
		boost::shared_ptr<LightSensorModel> model, bool internalSensor) :
		RenderModel(model), internalSensor_(internalSensor) {
	this->partA_.reset(new Mesh());
}

LightSensorRenderModel::~LightSensorRenderModel() {

}

bool LightSensorRenderModel::initRenderModel() {

	bool meshLoadingA;

	if (internalSensor_) {
		std::cerr << "Internal light sensor has been deprecated" << std::endl;
		return false;
		//meshLoadingA = this->partA_->loadMesh(
		//		"../models/LightSensor_Internal.stl");
	} else {
		meshLoadingA = this->partA_->loadMesh(
				RobogenUtils::getMeshFile(this->getModel(),
						LightSensorModel::B_SENSOR_BASE_ID));
	}

	if (!meshLoadingA) {
		std::cerr << "[LightSensorRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> partA =
			this->partA_->getMesh();

	partA_->setColor(osg::Vec4(1, 0, 0, 0.5));

	if (internalSensor_) {
		std::cerr << "Internal light sensor has been deprecated" << std::endl;
		return false;
	} else {
		partA->setPosition(
				RobogenUtils::getRelativePosition(this->getModel(),
						LightSensorModel::B_SENSOR_BASE_ID));

		partA->setAttitude(
				RobogenUtils::getRelativeAttitude(this->getModel(),
						LightSensorModel::B_SENSOR_BASE_ID));
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getMeshes()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(),
					LightSensorModel::B_SENSOR_BASE_ID));

	if (isDebugActive()) {
		this->activateTransparency(patPartA->getOrCreateStateSet());
	}

	return true;

}

void LightSensorRenderModel::showDebugView() {

	this->attachGeoms();
}

void LightSensorRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
}

}
