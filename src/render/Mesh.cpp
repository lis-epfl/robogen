/*
 * @(#) Mesh.cpp   1.0   Feb 6, 2013
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
#include "render/Mesh.h"
#include <osgDB/ReadFile>
#include <osg/Version>
#if OSG_VERSION_GREATER_OR_EQUAL(3, 2, 0)
	#include <osgUtil/SmoothingVisitor>
#endif

namespace robogen {

Mesh::Mesh() {

}

Mesh::~Mesh() {

}

bool Mesh::loadMesh(const std::string& mesh) {

	meshNode_ = osgDB::readNodeFile(mesh);
	if (meshNode_ == NULL) {
		return false;
	}

	// Translate mesh to center
	osg::BoundingBox bb;
	bb.expandBy(meshNode_->getBound());

	meshPat_ = new osg::PositionAttitudeTransform();

	osg::PositionAttitudeTransform* shiftToCenterPat =
			new osg::PositionAttitudeTransform();
	shiftToCenterPat->setPosition(-bb.center());
	shiftToCenterPat->addChild(meshNode_);

	meshPat_->addChild(shiftToCenterPat);

	xLen_ = bb.xMax() - bb.xMin();
	yLen_ = bb.yMax() - bb.yMin();
	zLen_ = bb.zMax() - bb.zMin();

#if OSG_VERSION_GREATER_OR_EQUAL(3, 2, 0)
	osgUtil::SmoothingVisitor sv;
	sv.setCreaseAngle(0);
	meshNode_->accept(sv);
#endif

	return true;

}

void Mesh::rescaleMesh(float scaleX, float scaleY, float scaleZ) {
	osg::PositionAttitudeTransform* rescalePat =
			new osg::PositionAttitudeTransform();
	rescalePat->setScale(osg::Vec3(scaleX, scaleY, scaleZ));
	rescalePat->addChild(meshPat_);
	meshPat_ = rescalePat;
}

osg::ref_ptr<osg::PositionAttitudeTransform> Mesh::getMesh() {
	// The mesh is returned with its pat
	return meshPat_;
}

float Mesh::xLen() {
	return xLen_;
}
float Mesh::yLen() {
	return yLen_;
}
float Mesh::zLen() {
	return zLen_;
}

void Mesh::setColor(osg::Vec4 color) {
#if OSG_VERSION_GREATER_OR_EQUAL(3, 2, 0)
	osg::Geometry* geometry =
	 meshNode_->asGroup()->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(color);
	colors->setBinding(osg::Array::BIND_OVERALL);
	geometry->setColorArray(colors);
#else
	osg::Geometry* geometry =
			this->meshNode_->asGeode()->getDrawable(0)->asGeometry();
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(color);
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
#endif
}

}
