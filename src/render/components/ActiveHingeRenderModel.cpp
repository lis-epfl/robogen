/*
 * @(#) ActiveHingeRenderModel.cpp   1.0   Feb 12, 2013
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
#include "render/components/ActiveHingeRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

ActiveHingeRenderModel::ActiveHingeRenderModel(
		boost::shared_ptr<ActiveHingeModel> model) :
		RenderModel(model) {
	this->partA_.reset(new Mesh());
	this->partB_.reset(new Mesh());
}

ActiveHingeRenderModel::~ActiveHingeRenderModel() {

}

bool ActiveHingeRenderModel::initRenderModel() {

	bool meshLoadingA = this->partA_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  ActiveHingeModel::B_SLOT_A_ID));

	if (!meshLoadingA) {
		std::cerr << "[ActiveHingeRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  ActiveHingeModel::B_SLOT_B_ID));

	if (!meshLoadingB) {
		std::cerr << "[ActiveHingeRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> frame =
			this->partA_->getMesh();

	partA_->setColor(osg::Vec4(1, 0, 0, 0.5));
	partB_->setColor(osg::Vec4(0, 1, 0, 0.5));


	frame->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
			  ActiveHingeModel::B_SLOT_A_ID));

	frame->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
			  ActiveHingeModel::B_SLOT_A_ID));

	osg::ref_ptr<osg::PositionAttitudeTransform> patFrame(
			new osg::PositionAttitudeTransform());
	patFrame->addChild(frame);

	this->getMeshes()->addChild(patFrame.get());
	patFrame->setUpdateCallback(
			new BodyCallback(this->getModel(), ActiveHingeModel::B_SLOT_A_ID));

	// PART B

	osg::ref_ptr<osg::PositionAttitudeTransform> servo =
			this->partB_->getMesh();
	servo->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
			  ActiveHingeModel::B_SLOT_B_ID));

	servo->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
			  ActiveHingeModel::B_SLOT_B_ID));

	osg::ref_ptr<osg::PositionAttitudeTransform> patServo(
			new osg::PositionAttitudeTransform());
	patServo->addChild(servo.get());

	this->getMeshes()->addChild(patServo.get());
	patServo->setUpdateCallback(
			new BodyCallback(this->getModel(), ActiveHingeModel::B_SLOT_B_ID));


	if(isDebugActive()) {
		this->activateTransparency(patFrame->getOrCreateStateSet());
		this->activateTransparency(patServo->getOrCreateStateSet());
	}

	return true;
}

void ActiveHingeRenderModel::showDebugView() {
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pats = this->attachGeoms();


}

void ActiveHingeRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
