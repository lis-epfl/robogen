/*
 * @(#) LightSourceRender.cpp   1.0   Feb 28, 2013
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
#include "model/objects/LightSource.h"
#include "render/callback/PositionObservableCallback.h"
#include "render/objects/LightSourceRender.h"

#include <osg/Geode>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/Material>
#include <osg/ShapeDrawable>

namespace robogen {

LightSourceRender::LightSourceRender(
		boost::shared_ptr<LightSource> lightSource, osg::ref_ptr<osg::Group> rootNode) {

	osg::ref_ptr<osg::Light> light(new osg::Light());
	light->setLightNum(1);
	light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 50.0));
	light->setDiffuse(osg::Vec4(1.0, 0.0, 0.0, 1.0));
	light->setSpecular(osg::Vec4(1.0, 0.0, 1.0, 1.0));
	light->setAmbient(osg::Vec4(0.0, 0.0, 0.0, 1.0));

	osg::ref_ptr<osg::Material> material(new osg::Material());
	material->setDiffuse(osg::Material::FRONT,  osg::Vec4(1.0, 1.0, 1.0, 1.0));
	material->setEmission(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));

	osg::ref_ptr<osg::Geode> lightMarker(new osg::Geode());
	lightMarker->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), fromOde(LightSource::RADIUS))));
	lightMarker->getOrCreateStateSet()->setAttribute(material.get());

	osg::ref_ptr<osg::LightSource> osgLightSource(new osg::LightSource());
	osgLightSource->setLight(light);
	osgLightSource->setLocalStateSetModes(osg::StateAttribute::ON);
	osg::StateSet *lightStateSet = rootNode->getOrCreateStateSet();
	osgLightSource->setStateSetModes(*lightStateSet, osg::StateAttribute::ON);

	rootNode_ = osg::ref_ptr<osg::PositionAttitudeTransform>(new osg::PositionAttitudeTransform());
	rootNode_->addChild(osgLightSource);
	rootNode_->addChild(lightMarker);
	rootNode_->setUpdateCallback(
				new PositionObservableCallback(lightSource));


}

LightSourceRender::~LightSourceRender() {

}

osg::ref_ptr<osg::PositionAttitudeTransform> LightSourceRender::getRootNode() {
	return rootNode_;
}

}
