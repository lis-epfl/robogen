/*
 * @(#) LightSensorRenderModel.cpp   1.0   Feb 9, 2013
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
#include "render/components/LightSensorRenderModel.h"
#include "render/Mesh.h"

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
		meshLoadingA = this->partA_->loadMesh(
				"../models/LightSensor_Internal.stl");
	} else {
		meshLoadingA = this->partA_->loadMesh(
				"../models/LightSensor_External.stl");
	}

	if (!meshLoadingA) {
		std::cerr << "[LightSensorRenderModel] Error loading model"
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

	partA_->setColor(osg::Vec4(1, 0, 0, 1));

	if (internalSensor_) {
		partA->setPosition(osg::Vec3(-4.25, 0, 0));
		partA->setAttitude(osg::Quat(osg::inDegrees(90.0), osg::Vec3(0, 1, 0)));
	} else {
		partA->setPosition(osg::Vec3(4.25, 0, 0));
		partA->setAttitude(osg::Quat(osg::inDegrees(90.0), osg::Vec3(0, 1, 0)));
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getRootNode()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(),
					LightSensorModel::B_SENSOR_BASE_ID));

	return true;

}

void LightSensorRenderModel::showDebugView() {

	this->attachBox(LightSensorModel::B_SENSOR_BASE_ID,
			LightSensorModel::SENSOR_BASE_THICKNESS,
			LightSensorModel::SENSOR_BASE_WIDTH,
			LightSensorModel::SENSOR_BASE_WIDTH);
	this->attachAxis(this->getRootNode());

}

void LightSensorRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
}

}
