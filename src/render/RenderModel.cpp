/*
 * @(#) RenderModel.cpp   1.0   Feb 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/ShapeDrawable>

#include "model/Model.h"

#include "render/callback/BodyCallback.h"
#include "render/RenderModel.h"

namespace robogen {

RenderModel::RenderModel(boost::shared_ptr<Model> model,
		bool debugActive) :
		model_(model), debugActive_(debugActive) {
	this->rootNode_ = osg::ref_ptr<osg::PositionAttitudeTransform>(
			new osg::PositionAttitudeTransform());
}

RenderModel::~RenderModel() {

}

osg::ref_ptr<osg::PositionAttitudeTransform> RenderModel::getRootNode() {
	return this->rootNode_;
}

bool RenderModel::isDebugActive() {
	return debugActive_;
}

boost::shared_ptr<Model> RenderModel::getModel() {
	return model_;
}

/**
 * Function taken from James Moliere, Open Scene Graph examples
 * x is red, y is blue, z is green
 */
void RenderModel::attachAxis(osg::Transform* transform) {

	osg::TessellationHints* hints = new osg::TessellationHints;

	double height = 100;
	double radius = .5;

	osg::MatrixTransform* zmt = new osg::MatrixTransform();

	transform->addChild(zmt);
	osg::ShapeDrawable *zShape = new osg::ShapeDrawable(
			new osg::Cylinder(osg::Vec3(0.0f, 0.0f, height / 2), radius,
					height), hints);
	osg::ShapeDrawable *zCone = new osg::ShapeDrawable(
			new osg::Cone(osg::Vec3(0.0f, 0.0f, 1.0), radius + 1.0, 2.0),
			hints);

	osg::MatrixTransform* zmtCone = new osg::MatrixTransform();
	osg::Geode *zgCone = new osg::Geode;

	zmtCone->setMatrix(osg::Matrix::translate(0.0, 0.0, height));
	transform->addChild(zmtCone);

	zShape->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	zCone->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	osg::Geode *z = new osg::Geode;
	z->addDrawable(zShape);
	zgCone->addDrawable(zCone);
	zmtCone->addChild(zgCone);
	zmt->addChild(z);

	osg::MatrixTransform* mt = new osg::MatrixTransform();
	transform->addChild(mt);

	osg::Matrix xMatrix = osg::Matrix::rotate(-osg::PI_2, 0.0, 1.0, 0.0);
	mt->setMatrix(xMatrix);

	osg::ShapeDrawable *xShape = new osg::ShapeDrawable(
			new osg::Cylinder(osg::Vec3(0.0f, 0.0f, height / 2), radius,
					height), hints);
	xShape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	osg::Geode *x = new osg::Geode;
	x->addDrawable(xShape);
	mt->addChild(x);

	osg::MatrixTransform *yMt = new osg::MatrixTransform();
	transform->addChild(yMt);
	osg::Matrix yMatrix = osg::Matrix::rotate(osg::PI_2, 1.0, 0.0, 0.0);
	yMt->setMatrix(yMatrix);

	osg::ShapeDrawable *yShape = new osg::ShapeDrawable(
			new osg::Cylinder(osg::Vec3(0.0f, 0.0f, height / 2), radius,
					height), hints);
	yShape->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	osg::Geode *y = new osg::Geode;
	y->addDrawable(yShape);
	yMt->addChild(y);
}



osg::ref_ptr<osg::Geode> RenderModel::getBox(float lengthX, float lengthY,
		float lengthZ) {
	return getBox(lengthX, lengthY, lengthZ, osg::Vec4(1, 0, 0, 1));
}

osg::ref_ptr<osg::Geode> RenderModel::getBox(float lengthX, float lengthY,
		float lengthZ, const osg::Vec4& color) {

	osg::ref_ptr<osg::Box> box(
			new osg::Box(osg::Vec3(0, 0, 0), lengthX, lengthY, lengthZ));
	osg::ref_ptr<osg::ShapeDrawable> boxDrawable(
			new osg::ShapeDrawable(box.get()));
	boxDrawable->setColor(color);
	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
	geode->addDrawable(boxDrawable.get());

	return geode;

}

osg::ref_ptr<osg::Geode> RenderModel::getCylinder(float radius, float height) {

	osg::ref_ptr<osg::Cylinder> cylinder(
			new osg::Cylinder(osg::Vec3(0, 0, 0), radius, height));
	osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable(
			new osg::ShapeDrawable(cylinder.get()));
	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
	geode->addDrawable(cylinderDrawable.get());

	return geode;

}

osg::ref_ptr<osg::Geode> RenderModel::getCapsule(float radius, float height) {

	osg::ref_ptr<osg::Capsule> capsule(
			new osg::Capsule(osg::Vec3(0, 0, 0), radius, height));
	osg::ref_ptr<osg::ShapeDrawable> capsuleDrawable(
			new osg::ShapeDrawable(capsule.get()));
	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
	geode->addDrawable(capsuleDrawable.get());

	return geode;

}

osg::ref_ptr<osg::PositionAttitudeTransform> RenderModel::attachBox(int label, float lengthX, float lengthY,
		float lengthZ) {

	return attachBox(label, lengthX, lengthY, lengthZ, osg::Vec4(1, 0, 0, 1));

}

osg::ref_ptr<osg::PositionAttitudeTransform> RenderModel::attachBox(int label, float lengthX, float lengthY,
		float lengthZ, const osg::Vec4& color) {

	osg::ref_ptr<osg::Geode> box = this->getBox(fromOde(lengthX), fromOde(lengthY), fromOde(lengthZ), color);

	osg::ref_ptr<osg::PositionAttitudeTransform> pat(
			new osg::PositionAttitudeTransform());
	pat->addChild(box);

	//this->attachAxis(pat);

	this->getRootNode()->addChild(pat.get());
	pat->setUpdateCallback(new BodyCallback(this->getModel(), label));

	return pat;

}

osg::ref_ptr<osg::PositionAttitudeTransform> RenderModel::attachGeode(int label, osg::ref_ptr<osg::Geode> geode) {

	osg::ref_ptr<osg::PositionAttitudeTransform> pat(
			new osg::PositionAttitudeTransform());
	pat->addChild(geode);

	this->getRootNode()->addChild(pat.get());
	pat->setUpdateCallback(new BodyCallback(this->getModel(), label));

	return pat;

}

}
