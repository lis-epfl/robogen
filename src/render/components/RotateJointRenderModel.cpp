/*
 * @(#) RotateJointRenderModel.cpp   1.0   Feb 18, 2013
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
#include "render/Mesh.h"
#include "render/components/RotateJointRenderModel.h"

#include "utils/RobogenUtils.h"

namespace robogen {

RotateJointRenderModel::RotateJointRenderModel(
		boost::shared_ptr<RotateJointModel> model) :
		RenderModel(model) {
	this->partA_.reset(new Mesh());
	this->partB_.reset(new Mesh());
}

RotateJointRenderModel::~RotateJointRenderModel() {

}

bool RotateJointRenderModel::initRenderModel() {

	bool meshLoadingA = this->partA_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  RotateJointModel::B_SLOT_ID));

	if (!meshLoadingA) {
		std::cerr << "[RotateJointModel] Error loading model" << std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  RotateJointModel::B_JOINT_CONNECTION_ID));

	if (!meshLoadingB) {
		std::cerr << "[RotateJointModel] Error loading model" << std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
	}

	partA_->setColor(osg::Vec4(0, 0, 1, 0.5));
	partB_->setColor(osg::Vec4(0, 1, 0, 0.5));

	// SLOT
	osg::ref_ptr<osg::PositionAttitudeTransform> slot = this->partA_->getMesh();
	slot->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
			  RotateJointModel::B_SLOT_ID));
	slot->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
 			  RotateJointModel::B_SLOT_ID));

	osg::ref_ptr<osg::PositionAttitudeTransform> patSlot(
			new osg::PositionAttitudeTransform());
	patSlot->addChild(slot);

	this->getMeshes()->addChild(patSlot.get());
	patSlot->setUpdateCallback(
			new BodyCallback(this->getModel(), RotateJointModel::B_SLOT_ID));

	// JOINT_CONNECTION
	osg::ref_ptr<osg::PositionAttitudeTransform> jointConnection =
			this->partB_->getMesh();
	jointConnection->setAttitude(
			RobogenUtils::getRelativeAttitude(this->getModel(),
			  RotateJointModel::B_JOINT_CONNECTION_ID));
	jointConnection->setPosition(
				RobogenUtils::getRelativePosition(this->getModel(),
				  RotateJointModel::B_JOINT_CONNECTION_ID));


	osg::ref_ptr<osg::PositionAttitudeTransform> patJointConnection(
			new osg::PositionAttitudeTransform());
	patJointConnection->addChild(jointConnection);

	this->getMeshes()->addChild(patJointConnection.get());
	patJointConnection->setUpdateCallback(
			new BodyCallback(this->getModel(),
					RotateJointModel::B_JOINT_CONNECTION_ID));

	if(isDebugActive()) {
		this->activateTransparency(patSlot->getOrCreateStateSet());
		this->activateTransparency(patJointConnection->getOrCreateStateSet());
		attachAxis(patJointConnection);
	}


	return true;

}

void RotateJointRenderModel::showDebugView() {

	this->attachGeoms();

}

void RotateJointRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
