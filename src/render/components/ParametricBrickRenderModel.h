/*
 * @(#) ParametricBrickRenderModel.h   1.0   Feb 14, 2013
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
#ifndef ROBOGEN_PARAMETRIC_BRICK_RENDER_MODEL_H_
#define ROBOGEN_PARAMETRIC_BRICK_RENDER_MODEL_H_

#include "model/components/ParametricBrickModel.h"
#include "render/RenderModel.h"

namespace robogen {

class Mesh;

class ParametricBrickRenderModel: public RenderModel {

public:

	ParametricBrickRenderModel(boost::shared_ptr<ParametricBrickModel> model);

	virtual ~ParametricBrickRenderModel();

	virtual bool initRenderModel();

	void showDebugView();

	virtual void setColor(osg::Vec4 color);

private:

	boost::shared_ptr<ParametricBrickModel> brickModel_;

};

}

#endif /* ROBOGEN_PARAMETRIC_BRICK_RENDER_MODEL_H_ */
