/*
 * @(#) ParametricBrickRenderModel.cpp   1.0   Feb 14, 2013
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
#include "render/components/ParametricBrickRenderModel.h"
#include "render/Mesh.h"

namespace robogen {

ParametricBrickRenderModel::ParametricBrickRenderModel(
		boost::shared_ptr<ParametricBrickModel> model) :
		RenderModel(model), brickModel_(model) {

}

ParametricBrickRenderModel::~ParametricBrickRenderModel() {

}

bool ParametricBrickRenderModel::initRenderModel() {

	this->showDebugView();

	return true;

}

void ParametricBrickRenderModel::showDebugView() {

	osg::ref_ptr<osg::PositionAttitudeTransform> patSlotA = this->attachBox(
			ParametricBrickModel::B_SLOT_A_ID,
			ParametricBrickModel::SLOT_THICKNESS,
			ParametricBrickModel::SLOT_WIDTH, ParametricBrickModel::SLOT_WIDTH);

	//this->attachAxis(patSlotA);

	osg::ref_ptr<osg::PositionAttitudeTransform> patSlotB = this->attachBox(
			ParametricBrickModel::B_SLOT_B_ID,
			ParametricBrickModel::SLOT_THICKNESS,
			ParametricBrickModel::SLOT_WIDTH, ParametricBrickModel::SLOT_WIDTH);

	//this->attachAxis(patSlotB);

	this->attachBox(ParametricBrickModel::B_CONNECTION_ID,
			brickModel_->getConnectionLength(),
			ParametricBrickModel::CONNECTION_PART_WIDTH,
			ParametricBrickModel::CONNECTION_PART_THICKNESS);

	/*this->attachBox(ParametricBrickModel::B_CAPSULE_ID,
	 ParametricBrickModel::CAPSULE_LENGTH,
	 ParametricBrickModel::CONNECTION_PART_WIDTH,
	 ParametricBrickModel::CAPSULE_HEIGHT);*/

}

void ParametricBrickRenderModel::setColor(osg::Vec4 /*color*/) {

}

}
