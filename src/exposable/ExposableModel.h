/*
 * @(#) ExposableModel.h   1.0   Dec 4, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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
#ifndef EXPOSABLEMODEL_H_
#define EXPOSABLEMODEL_H_


#include "Exposable.h"

namespace robogen {


class ExposableModel : public Exposable {

public :
	ExposableModel(boost::shared_ptr<Model> model) : model_(model) {}

EXPOSE_METHODS :

	osg::Vec3 getRootPosition();

private :
	boost::shared_ptr<Model> model_;


};

}

#endif /* EXPOSABLEMODEL_H_ */
