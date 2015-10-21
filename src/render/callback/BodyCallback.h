/*
 * @(#) BodyCallback.h   1.0   Feb 9, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_BODY_CALLBACK_H_
#define ROBOGEN_BODY_CALLBACK_H_

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <osg/Node>
#include "Robogen.h"

namespace robogen {

class Model;

class BodyCallback: public osg::NodeCallback {

public:

	BodyCallback(boost::shared_ptr<Model> model, int bodyId);

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);

private:

	/**
	 * The physics model
	 * keep as weak ptr so does not mess up ode cleanup
	 */
	boost::weak_ptr<Model> model_;

	/**
	 * The id of the body to track
	 */
	int bodyId_;

};

}

#endif /* ROBOGEN_BODY_CALLBACK_H_ */
