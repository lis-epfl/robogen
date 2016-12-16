/*
 * @(#) ParametricPrismRenderModel.cpp   1.0   Oct 14, 2016
 *
 * Gaël Gorret (gael.gorret@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2016-2017 Gaël Gorret
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
#include "render/components/ParametricPrismRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

ParametricPrismRenderModel::ParametricPrismRenderModel(
		boost::shared_ptr<ParametricPrismModel> model) :
		RenderModel(model), prismModel_(model) {

}

ParametricPrismRenderModel::~ParametricPrismRenderModel() {

}

bool ParametricPrismRenderModel::initRenderModel() {

	if (isDebugActive()) {
		this->showDebugView();
	} else {
		std::vector<osg::Vec4> colors;
		colors.push_back(osg::Vec4(1,0,0,1));
		this->attachGeoms(colors);
	}

	return true;

}

void ParametricPrismRenderModel::showDebugView() {
	this->attachGeoms();
}

void ParametricPrismRenderModel::setColor(osg::Vec4 /*color*/) {

}

}
