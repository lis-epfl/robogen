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

namespace robogen {

HingeRenderModel::HingeRenderModel(boost::shared_ptr<HingeModel> model) :
		RenderModel(model) {
	this->partA_.reset(new Mesh());
	this->partB_.reset(new Mesh());
}

HingeRenderModel::~HingeRenderModel() {

}

bool HingeRenderModel::initRenderModel() {

	bool meshLoadingA = this->partA_->loadMesh("../models/PassiveHinge.stl");

	if (!meshLoadingA) {
		std::cerr << "[HingeRenderModel] Error loading model" << std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh("../models/PassiveHinge.stl");

	if (!meshLoadingB) {
		std::cerr << "[HingeRenderModel] Error loading model" << std::endl;
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
	partB_->setColor(osg::Vec4(0, 1, 0, 1));

	partA->setPosition(
			fromOde(
					osg::Vec3(
							HingeModel::CONNNECTION_PART_LENGTH / 2,
							0, 0)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getRootNode()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(), HingeModel::B_SLOT_A_ID));

	// PART B
	osg::ref_ptr<osg::PositionAttitudeTransform> partB =
			this->partB_->getMesh();
	partB->setPosition(
			fromOde(
					osg::Vec3(
							-(HingeModel::CONNNECTION_PART_LENGTH / 2),
							0, 0)));
	partB->setAttitude(osg::Quat(osg::inDegrees(180.0), osg::Vec3(0, 1, 0)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartB(
			new osg::PositionAttitudeTransform());
	patPartB->addChild(partB.get());

	this->getRootNode()->addChild(patPartB.get());
	patPartB->setUpdateCallback(
			new BodyCallback(this->getModel(), HingeModel::B_SLOT_B_ID));

	return true;

}

void HingeRenderModel::showDebugView() {

	this->attachBox(HingeModel::B_SLOT_A_ID, HingeModel::SLOT_THICKNESS,
			HingeModel::SLOT_WIDTH, HingeModel::SLOT_WIDTH);

	this->attachBox(HingeModel::B_SLOT_B_ID, HingeModel::SLOT_THICKNESS,
			HingeModel::SLOT_WIDTH, HingeModel::SLOT_WIDTH);

	this->attachBox(HingeModel::B_CONNECTION_A_ID,
			HingeModel::CONNNECTION_PART_LENGTH,
			HingeModel::CONNECTION_PART_THICKNESS,
			HingeModel::CONNECTION_PART_HEIGHT);

	this->attachBox(HingeModel::B_CONNECTION_B_ID,
			HingeModel::CONNNECTION_PART_LENGTH,
			HingeModel::CONNECTION_PART_THICKNESS,
			HingeModel::CONNECTION_PART_HEIGHT);

}

void HingeRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
