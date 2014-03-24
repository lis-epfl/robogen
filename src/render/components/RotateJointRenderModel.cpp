/*
 * @(#) RotateJointRenderModel.cpp   1.0   Feb 18, 2013
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
#include "render/Mesh.h"
#include "render/components/RotateJointRenderModel.h"

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
			"../models/ActiveRotation_Motor_Holder.stl");

	if (!meshLoadingA) {
		std::cerr << "[RotateJointModel] Error loading model" << std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh(
			"../models/ActiveRotation_Connection.stl");

	if (!meshLoadingB) {
		std::cerr << "[RotateJointModel] Error loading model" << std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
		return true;
	}

	partA_->setColor(osg::Vec4(0, 0, 1, 1));
	//partA_->setColor(osg::Vec4(1, 0, 0, 1));
	partB_->setColor(osg::Vec4(0, 1, 0, 1));

	float slotCorrectionZ = inMm(1.5);

	// SLOT
	osg::ref_ptr<osg::PositionAttitudeTransform> slot = this->partA_->getMesh();
	slot->setAttitude(osg::Quat(osg::inDegrees(90.0), osg::Vec3(1, 0, 0)));
	slot->setPosition(
			fromOde(
					osg::Vec3(
							RotateJointModel::SLOT_THICKNESS / 2
									+ RotateJointModel::SERVO_LENGTH / 2, 0,
									slotCorrectionZ)));
	//attachAxis(slot);

	osg::ref_ptr<osg::PositionAttitudeTransform> patSlot(
			new osg::PositionAttitudeTransform());
	patSlot->addChild(slot);

	this->getRootNode()->addChild(patSlot.get());
	patSlot->setUpdateCallback(
			new BodyCallback(this->getModel(), RotateJointModel::B_SLOT_ID));

	// JOINT_CONNECTION
	osg::ref_ptr<osg::PositionAttitudeTransform> jointConnection =
			this->partB_->getMesh();
	jointConnection->setAttitude(
			osg::Quat(osg::inDegrees(-90.0), osg::Vec3(0, 1, 0)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patJointConnection(
			new osg::PositionAttitudeTransform());
	patJointConnection->addChild(jointConnection);

	this->getRootNode()->addChild(patJointConnection.get());
	patJointConnection->setUpdateCallback(
			new BodyCallback(this->getModel(),
					RotateJointModel::B_JOINT_CONNECTION_ID));

	return true;

}

void RotateJointRenderModel::showDebugView() {

	this->attachBox(RotateJointModel::B_SLOT_ID,
			RotateJointModel::SLOT_THICKNESS, RotateJointModel::SLOT_WIDTH,
			RotateJointModel::SLOT_WIDTH, osg::Vec4(0, 1, 0, 1));

	this->attachBox(RotateJointModel::B_SERVO_ID,
			RotateJointModel::SERVO_LENGTH, RotateJointModel::SERVO_WIDTH,
			RotateJointModel::SERVO_HEIGHT);

	this->attachBox(RotateJointModel::B_JOINT_CONNECTION_ID,
			RotateJointModel::JOINT_CONNECTION_THICKNESS,
			RotateJointModel::JOINT_CONNECTION_WIDTH,
			RotateJointModel::JOINT_CONNECTION_WIDTH, osg::Vec4(0, 0, 1, 1));

}

void RotateJointRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
