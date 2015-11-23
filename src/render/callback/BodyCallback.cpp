/*
 * @(#) BodyCallback.cpp   1.0   Feb 9, 2013
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
#include <osg/PositionAttitudeTransform>

#include "model/Model.h"
#include "render/callback/BodyCallback.h"


namespace robogen {

BodyCallback::BodyCallback(boost::shared_ptr<Model> model, int bodyId) :
      model_(model), bodyId_(bodyId) {

}

void BodyCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {

   osg::ref_ptr<osg::PositionAttitudeTransform> pat =
         node->asTransform()->asPositionAttitudeTransform();

   pat->setAttitude(model_.lock()->getBodyAttitude(bodyId_));
   pat->setPosition(fromOde(model_.lock()->getBodyPosition(bodyId_)));

   traverse(node, nv);
}

}
