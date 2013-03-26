/*
 * @(#) CoreComponentRenderModel.cpp   1.0   Feb 5, 2013
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

#include "render/components/CoreComponentRenderModel.h"
#include "render/Mesh.h"

namespace robogen {

CoreComponentRenderModel::CoreComponentRenderModel(
		boost::shared_ptr<CoreComponentModel> model) :
		RenderModel(model) {
	this->mesh_.reset(new Mesh());
}

CoreComponentRenderModel::~CoreComponentRenderModel() {

}

bool CoreComponentRenderModel::initRenderModel() {

	bool meshLoading = this->mesh_->loadMesh("models/CoreComponent.stl");

	if (!meshLoading) {
		std::cerr << "[CoreComponentRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
		return true;
	}

	//attachAxis(this->getRootNode());

	this->getRootNode()->addChild(this->mesh_->getMesh());
	this->getRootNode()->setUpdateCallback(
			new BodyCallback(this->getModel(),
					CoreComponentModel::B_CORE_COMPONENT_ID));

	return true;

}

void CoreComponentRenderModel::showDebugView() {

	this->attachBox(CoreComponentModel::B_CORE_COMPONENT_ID,
			CoreComponentModel::WIDTH, CoreComponentModel::WIDTH,
			CoreComponentModel::WIDTH);

}

void CoreComponentRenderModel::setColor(osg::Vec4 color) {
	this->mesh_->setColor(color);
}

}
