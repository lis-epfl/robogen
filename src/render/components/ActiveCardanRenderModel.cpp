/*
 * @(#) ActiveCardanRenderModel.cpp   1.0   Feb 14, 2013
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
#include "render/callback/ActiveCardanCrossCallback.h"
#include "render/components/ActiveCardanRenderModel.h"
#include "render/Mesh.h"

namespace robogen {

ActiveCardanRenderModel::ActiveCardanRenderModel(
		boost::shared_ptr<ActiveCardanModel> model) :
		RenderModel(model) {
	this->partA_.reset(new Mesh());
	this->partB_.reset(new Mesh());
	this->patCross_.reset(new Mesh());
}

ActiveCardanRenderModel::~ActiveCardanRenderModel() {

}

bool ActiveCardanRenderModel::initRenderModel() {

	bool meshLoadingA = this->partA_->loadMesh(
			"../models/ActiveCardanHinge_Servo_Holder.stl");

	if (!meshLoadingA) {
		std::cerr << "[ActiveCardanRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh(
			"../models/ActiveCardanHinge_Servo_Holder.stl");

	if (!meshLoadingB) {
		std::cerr << "[ActiveCardanRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	bool meshLoadingC = this->patCross_->loadMesh(
			"../models/ActiveCardan_CrossShaft.stl");

	if (!meshLoadingC) {
		std::cerr << "[ActiveCardanRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
		return true;
	}

	float meshCorrection = inMm(0.5);

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> partA =
			this->partA_->getMesh();

	partA_->setColor(osg::Vec4(1, 0, 0, 1));
	partB_->setColor(osg::Vec4(0, 1, 0, 1));

	partA->setPosition(
			osg::Vec3(
					fromOde(meshCorrection +
							ActiveCardanModel::SLOT_THICKNESS
									+ ActiveCardanModel::CONNNECTION_PART_LENGTH
											/ 2), 0, 0));

	osg::Quat rotateServoZ(osg::inDegrees(90.0), osg::Vec3(0, 0, 1));
	partA->setAttitude(rotateServoZ);

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getRootNode()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(), ActiveCardanModel::B_SLOT_A_ID));

	//attachAxis(patPartA);

	// Cross
	osg::ref_ptr<osg::PositionAttitudeTransform> cross =
			this->patCross_->getMesh();

	osg::ref_ptr<osg::PositionAttitudeTransform> patCross(
			new osg::PositionAttitudeTransform());
	patCross->addChild(cross.get());

	this->getRootNode()->addChild(patCross.get());
	patCross->setUpdateCallback(
			new BodyCallback(this->getModel(),
					ActiveCardanModel::B_CROSS_PART_A_ID));
	//attachAxis(patCross);

	// PART B
	osg::ref_ptr<osg::PositionAttitudeTransform> partB =
			this->partB_->getMesh();
	partB->setPosition(
			fromOde(
					osg::Vec3(
							-(meshCorrection + ActiveCardanModel::SLOT_THICKNESS
									+ ActiveCardanModel::CONNNECTION_PART_LENGTH
											/ 2), 0, 0)));

	osg::Quat rotateY(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
	osg::Quat rotateServoX(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
	partB->setAttitude(rotateServoX * rotateY *
			osg::Quat(osg::inDegrees(90.0), osg::Vec3(1, 0, 0)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartB(
			new osg::PositionAttitudeTransform());
	patPartB->addChild(partB.get());

	this->getRootNode()->addChild(patPartB.get());
	patPartB->setUpdateCallback(
			new BodyCallback(this->getModel(), ActiveCardanModel::B_SLOT_B_ID));

	//attachAxis(patPartB);

	return true;

}

void ActiveCardanRenderModel::showDebugView() {

	this->attachBox(ActiveCardanModel::B_SLOT_A_ID,
			ActiveCardanModel::SLOT_THICKNESS, ActiveCardanModel::SLOT_WIDTH,
			ActiveCardanModel::SLOT_WIDTH);

	this->attachBox(ActiveCardanModel::B_SLOT_B_ID,
			ActiveCardanModel::SLOT_THICKNESS, ActiveCardanModel::SLOT_WIDTH,
			ActiveCardanModel::SLOT_WIDTH);

	this->attachBox(ActiveCardanModel::B_CONNECTION_A_ID,
			ActiveCardanModel::CONNNECTION_PART_LENGTH,
			ActiveCardanModel::CONNNECTION_PART_WIDTH,
			ActiveCardanModel::CONNECTION_PART_HEIGHT);

	this->attachBox(ActiveCardanModel::B_CONNECTION_B_ID,
			ActiveCardanModel::CONNNECTION_PART_LENGTH,
			ActiveCardanModel::CONNECTION_PART_HEIGHT,
			ActiveCardanModel::CONNNECTION_PART_WIDTH);

	this->attachBox(ActiveCardanModel::B_CROSS_PART_A_ID,
			ActiveCardanModel::CROSS_THICKNESS, ActiveCardanModel::CROSS_WIDTH,
			ActiveCardanModel::CROSS_HEIGHT);

	this->attachBox(ActiveCardanModel::B_CROSS_PART_B_ID,
			ActiveCardanModel::CROSS_THICKNESS, ActiveCardanModel::CROSS_HEIGHT,
			ActiveCardanModel::CROSS_WIDTH);

	this->attachBox(ActiveCardanModel::B_CROSS_PART_A_EDGE_1_ID,
			ActiveCardanModel::CROSS_CENTER_OFFSET,
			ActiveCardanModel::CROSS_WIDTH, ActiveCardanModel::CROSS_THICKNESS);

	this->attachBox(ActiveCardanModel::B_CROSS_PART_A_EDGE_2_ID,
			ActiveCardanModel::CROSS_CENTER_OFFSET,
			ActiveCardanModel::CROSS_WIDTH, ActiveCardanModel::CROSS_THICKNESS);

	this->attachBox(ActiveCardanModel::B_CROSS_PART_B_EDGE_1_ID,
			ActiveCardanModel::CROSS_CENTER_OFFSET,
			ActiveCardanModel::CROSS_THICKNESS, ActiveCardanModel::CROSS_WIDTH);

	this->attachBox(ActiveCardanModel::B_CROSS_PART_B_EDGE_2_ID,
			ActiveCardanModel::CROSS_CENTER_OFFSET,
			ActiveCardanModel::CROSS_THICKNESS, ActiveCardanModel::CROSS_WIDTH);

}

void ActiveCardanRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
