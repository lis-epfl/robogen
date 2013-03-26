/*
 * @(#) PositionObservableCallback.cpp   1.0   Mar 20, 2013
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
#include <osg/PositionAttitudeTransform>

#include "model/Model.h"
#include "render/callback/PositionObservableCallback.h"


namespace robogen {

PositionObservableCallback::PositionObservableCallback(boost::shared_ptr<PositionObservable> model) :
      model_(model) {

}

void PositionObservableCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {

   osg::ref_ptr<osg::PositionAttitudeTransform> pat =
         node->asTransform()->asPositionAttitudeTransform();

   pat->setAttitude(model_->getAttitude());
   pat->setPosition(fromOde(model_->getPosition()));

   traverse(node, nv);
}

}
