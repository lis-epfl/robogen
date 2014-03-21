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

	bool meshLoadingA = this->partA_->loadMesh("../models/ActiveHinge_Frame.stl");

	if (!meshLoadingA) {
		std::cerr << "[ActiveHingeRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh(
			"../models/ActiveCardanHinge_Servo_Holder.stl");

	if (!meshLoadingB) {
		std::cerr << "[ActiveHingeRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
		return true;
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> frame =
			this->partA_->getMesh();

	partA_->setColor(osg::Vec4(1, 0, 0, 1));
	partB_->setColor(osg::Vec4(0, 1, 0, 1));

	frame->setPosition(
			fromOde(
					osg::Vec3(
							ActiveHingeModel::FRAME_LENGTH / 2, 0,
							0)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patFrame(
			new osg::PositionAttitudeTransform());
	patFrame->addChild(frame);

	this->getRootNode()->addChild(patFrame.get());
	patFrame->setUpdateCallback(
			new BodyCallback(this->getModel(), ActiveHingeModel::B_SLOT_A_ID));

	// PART B
	float servoMeshCorrection = inMm(2.2);

	osg::ref_ptr<osg::PositionAttitudeTransform> servo =
			this->partB_->getMesh();
	servo->setPosition(
			fromOde(
					osg::Vec3(
							-(ActiveHingeModel::SERVO_LENGTH / 2) - servoMeshCorrection, 0,
							0)));

	osg::Quat rotateServoZ(osg::inDegrees(90.0), osg::Vec3(0, 0, 1));
	osg::Quat rotateServoY(osg::inDegrees(-90.0), osg::Vec3(0, 1, 0));
	servo->setAttitude(rotateServoY * rotateServoZ);

	osg::ref_ptr<osg::PositionAttitudeTransform> patServo(
			new osg::PositionAttitudeTransform());
	patServo->addChild(servo.get());

	this->getRootNode()->addChild(patServo.get());
	patServo->setUpdateCallback(
			new BodyCallback(this->getModel(), ActiveHingeModel::B_SLOT_B_ID));

	return true;

}

void ActiveHingeRenderModel::showDebugView() {

	this->attachBox(ActiveHingeModel::B_SLOT_A_ID,
			ActiveHingeModel::SLOT_THICKNESS, ActiveHingeModel::SLOT_WIDTH,
			ActiveHingeModel::SLOT_WIDTH);
	this->attachBox(ActiveHingeModel::B_SLOT_B_ID,
			ActiveHingeModel::SLOT_THICKNESS, ActiveHingeModel::SLOT_WIDTH,
			ActiveHingeModel::SLOT_WIDTH);
	this->attachBox(ActiveHingeModel::B_FRAME_ID,
			ActiveHingeModel::FRAME_LENGTH, ActiveHingeModel::SLOT_WIDTH,
			ActiveHingeModel::FRAME_HEIGHT);
	this->attachBox(ActiveHingeModel::B_SERVO_ID,
			ActiveHingeModel::SERVO_LENGTH, ActiveHingeModel::SLOT_WIDTH,
			ActiveHingeModel::SERVO_HEIGHT);

}

void ActiveHingeRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
