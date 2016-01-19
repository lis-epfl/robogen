/*
 * @(#) IrSensorRenderModel.h   1.0   Jan 19, 2016
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Joshua Auerbach
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
#ifndef ROBOGEN_IRSENSORRENDERMODEL_H_
#define ROBOGEN_IRSENSORRENDERMODEL_H_


#include "model/components/perceptive/IrSensorModel.h"
#include "render/RenderModel.h"

namespace robogen {

class Mesh;

class IrSensorRenderModel: public RenderModel {

public:

	IrSensorRenderModel(boost::shared_ptr<IrSensorModel> model);

	virtual ~IrSensorRenderModel();

	virtual bool initRenderModel();

	void showDebugView();

	virtual void setColor(osg::Vec4 color);

private:

	boost::shared_ptr<Mesh> partA_;

};

}



#endif /* ROBOGEN_IRSENSORRENDERMODEL_H_ */
