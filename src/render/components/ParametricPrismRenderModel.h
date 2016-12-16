/*
 * @(#) ParametricPrismRenderModel.h   1.0   Oct 14, 2016
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

#ifndef ROBOGEN_PARAMETRIC_PRISM_RENDER_MODEL_H_
#define ROBOGEN_PARAMETRIC_PRISM_RENDER_MODEL_H_

#include "model/components/ParametricPrismModel.h"
#include "render/RenderModel.h"

namespace robogen {

	class Mesh;

	class ParametricPrismRenderModel: public RenderModel {

	public:

		ParametricPrismRenderModel(boost::shared_ptr<ParametricPrismModel> model);

		virtual ~ParametricPrismRenderModel();

		virtual bool initRenderModel();

		void showDebugView();

		virtual void setColor(osg::Vec4 color);

	private:

		boost::shared_ptr<ParametricPrismModel> prismModel_;

	};

}

#endif /* ROBOGEN_PARAMETRIC_PRISM_RENDER_MODEL_H_ */