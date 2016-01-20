/*
 * @(#) IrSensorRenderModel.cpp   1.0   Jan 19, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
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
#include "render/components/IrSensorRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

IrSensorRenderModel::IrSensorRenderModel(boost::shared_ptr<IrSensorModel>
		model) : RenderModel(model) {
	this->partA_.reset(new Mesh());
}

IrSensorRenderModel::~IrSensorRenderModel() {

}

bool IrSensorRenderModel::initRenderModel() {

	bool meshLoadingA  = this->partA_->loadMesh(
				RobogenUtils::getMeshFile(this->getModel(),
						IrSensorModel::B_SENSOR_BASE_ID));

	if (!meshLoadingA) {
		std::cerr << "[IrSensorRenderModel] Error loading model"
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

	partA->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
						IrSensorModel::B_SENSOR_BASE_ID));

	partA->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
						IrSensorModel::B_SENSOR_BASE_ID));


	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getMeshes()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(),
					IrSensorModel::B_SENSOR_BASE_ID));

	if (isDebugActive()) {
		this->activateTransparency(patPartA->getOrCreateStateSet());
	}

	return true;

}

void IrSensorRenderModel::showDebugView() {

	this->attachGeoms();
}

void IrSensorRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
}

}


