/*
 * @(#) HingeRenderModel.cpp   1.0   Feb 9, 2013
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
#include "render/components/HingeRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

HingeRenderModel::HingeRenderModel(boost::shared_ptr<HingeModel> model) :
		RenderModel(model) {
	this->partA_.reset(new Mesh());
	this->partB_.reset(new Mesh());
}

HingeRenderModel::~HingeRenderModel() {

}

bool HingeRenderModel::initRenderModel() {

	bool meshLoadingA = this->partA_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  HingeModel::B_SLOT_A_ID));

	if (!meshLoadingA) {
		std::cerr << "[HingeRenderModel] Error loading model" << std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  HingeModel::B_SLOT_B_ID));

	if (!meshLoadingB) {
		std::cerr << "[HingeRenderModel] Error loading model" << std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> partA =
			this->partA_->getMesh();

	partA_->setColor(osg::Vec4(1, 0, 0, 0.5));
	partB_->setColor(osg::Vec4(0, 1, 0, 0.5));

	partA->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
			  HingeModel::B_SLOT_A_ID));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getMeshes()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(), HingeModel::B_SLOT_A_ID));


	// PART B
	osg::ref_ptr<osg::PositionAttitudeTransform> partB =
			this->partB_->getMesh();
	partB->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
			HingeModel::B_SLOT_B_ID));
	partB->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
			HingeModel::B_SLOT_B_ID));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartB(
			new osg::PositionAttitudeTransform());
	patPartB->addChild(partB.get());

	this->getMeshes()->addChild(patPartB.get());
	patPartB->setUpdateCallback(
			new BodyCallback(this->getModel(), HingeModel::B_SLOT_B_ID));



	if(isDebugActive()) {
		this->activateTransparency(patPartA->getOrCreateStateSet());
		this->activateTransparency(patPartB->getOrCreateStateSet());
	}


	return true;

}

void HingeRenderModel::showDebugView() {

	this->attachGeoms();
}

void HingeRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
