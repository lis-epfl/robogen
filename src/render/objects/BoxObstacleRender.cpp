/*
 * @(#) BoxObstacleRender.cpp   1.0   Mar 12, 2013
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
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include "model/objects/BoxObstacle.h"
#include "render/callback/PositionObservableCallback.h"
#include "render/objects/BoxObstacleRender.h"

namespace robogen {

BoxObstacleRender::BoxObstacleRender(boost::shared_ptr<BoxObstacle> obstacle) {

	rootNode_ = new osg::PositionAttitudeTransform();

	osg::ref_ptr<osg::Box> box(
			new osg::Box(osg::Vec3(0, 0, 0), fromOde(obstacle->getSize().x()),
					fromOde(obstacle->getSize().y()), fromOde(obstacle->getSize().z())));

	osg::ref_ptr<osg::ShapeDrawable> boxDrawable(
			new osg::ShapeDrawable(box.get()));
	boxDrawable->setColor(osg::Vec4(1, 0, 0, 1));
	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
	geode->addDrawable(boxDrawable.get());
	rootNode_->addChild(geode);
	rootNode_->setUpdateCallback(new PositionObservableCallback(obstacle));

}

BoxObstacleRender::~BoxObstacleRender() {

}

osg::ref_ptr<osg::PositionAttitudeTransform> BoxObstacleRender::getRootNode() {
	return rootNode_;
}

}
