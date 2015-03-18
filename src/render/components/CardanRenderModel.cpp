/*
 * @(#) CardanRenderModel.cpp   1.0   Feb 13, 2013
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
#include "render/callback/CardanCrossCallback.h"
#include "render/components/CardanRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

CardanRenderModel::CardanRenderModel(boost::shared_ptr<CardanModel> model) :
		RenderModel(model) {
	this->partA_.reset(new Mesh());
	this->partB_.reset(new Mesh());
	this->patCross_.reset(new Mesh());
}

CardanRenderModel::~CardanRenderModel() {

}

bool CardanRenderModel::initRenderModel() {

	bool meshLoadingA = this->partA_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  CardanModel::B_SLOT_A_ID));

	if (!meshLoadingA) {
		std::cerr << "[CardanRenderModel] Error loading model" << std::endl;
		return false;
	}

	bool meshLoadingB = this->partB_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  CardanModel::B_SLOT_B_ID));

	if (!meshLoadingB) {
		std::cerr << "[CardanRenderModel] Error loading model" << std::endl;
		return false;
	}

	bool meshLoadingC = this->patCross_->loadMesh(
			RobogenUtils::getMeshFile(this->getModel(),
			  CardanModel::B_CONNECTION_A_ID));

	if (!meshLoadingC) {
		std::cerr << "[CardanRenderModel] Error loading model" << std::endl;
		return false;
	}

	//this->showDebugView();
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

							CardanModel::SLOT_THICKNESS / 2
									+ CardanModel::CONNNECTION_PART_LENGTH / 2,
							0, 0)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getRootNode()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(), CardanModel::B_SLOT_A_ID));

	//attachAxis(patPartA);

	// Cross
	osg::ref_ptr<osg::PositionAttitudeTransform> cross =
			this->patCross_->getMesh();
	cross->setAttitude(osg::Quat(osg::inDegrees(90.0), osg::Vec3(0, 0, 1)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patCross(
			new osg::PositionAttitudeTransform());
	patCross->addChild(cross.get());

	this->getRootNode()->addChild(patCross.get());
	patCross->setUpdateCallback(
			new CardanCrossCallback(
					boost::dynamic_pointer_cast<CardanModel>(
							this->getModel())));

	//attachAxis(patCross);

	// PART B
	osg::ref_ptr<osg::PositionAttitudeTransform> partB =
			this->partB_->getMesh();
	partB->setPosition(
			fromOde(
					osg::Vec3(
							-(CardanModel::SLOT_THICKNESS / 2
									+ CardanModel::CONNNECTION_PART_LENGTH / 2),
							0, 0)));
	partB->setAttitude(osg::Quat(osg::inDegrees(180.0), osg::Vec3(0, 1, 1)));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartB(
			new osg::PositionAttitudeTransform());
	patPartB->addChild(partB.get());

	this->getRootNode()->addChild(patPartB.get());
	patPartB->setUpdateCallback(
			new BodyCallback(this->getModel(), CardanModel::B_SLOT_B_ID));

	//attachAxis(patPartB);

	return true;

}

void CardanRenderModel::showDebugView() {

	this->attachBox(CardanModel::B_SLOT_A_ID, CardanModel::SLOT_THICKNESS,
			CardanModel::SLOT_WIDTH, CardanModel::SLOT_WIDTH);

	this->attachBox(CardanModel::B_SLOT_B_ID, CardanModel::SLOT_THICKNESS,
			CardanModel::SLOT_WIDTH, CardanModel::SLOT_WIDTH);

	this->attachBox(CardanModel::B_CONNECTION_A_ID,
			CardanModel::CONNNECTION_PART_LENGTH, CardanModel::SLOT_WIDTH,
			CardanModel::CONNECTION_PART_HEIGHT);

	this->attachBox(CardanModel::B_CONNECTION_B_ID,
			CardanModel::CONNNECTION_PART_LENGTH,
			CardanModel::CONNECTION_PART_HEIGHT, CardanModel::SLOT_WIDTH);

}

void CardanRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
	this->partB_->setColor(color);
}

}
