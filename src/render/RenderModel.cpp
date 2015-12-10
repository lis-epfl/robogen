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
#include <osg/BlendFunc>

#include "model/Model.h"

#include "render/callback/BodyCallback.h"
#include "render/RenderModel.h"

namespace robogen {

RenderModel::RenderModel(boost::shared_ptr<Model> model) :
		model_(model), debugActive_(false) {
	this->rootNode_ = osg::ref_ptr<osg::PositionAttitudeTransform>(
			new osg::PositionAttitudeTransform());
	this->primitives_ = osg::ref_ptr<osg::Group>(
			new osg::Group());
	this->meshes_ = osg::ref_ptr<osg::Group>(
				new osg::Group());
	this->rootNode_->addChild(this->primitives_);
	this->rootNode_->addChild(this->meshes_);

}

RenderModel::~RenderModel() {

}

osg::ref_ptr<osg::PositionAttitudeTransform> RenderModel::getRootNode() {
	return this->rootNode_;
}

bool RenderModel::isDebugActive() {
	return debugActive_;
}

void RenderModel::setDebugActive(bool debugActive) {
	this->debugActive_ = debugActive;

}

void RenderModel::togglePrimitives(bool primitives) {
	if(primitives) {
			this->primitives_->setNodeMask(0xffffffff);
	} else {
		this->primitives_->setNodeMask(0x0);
	}
}

void RenderModel::toggleMeshes(bool meshes) {
	if(meshes) {
		this->meshes_->setNodeMask(0xffffffff);
	} else {
		this->meshes_->setNodeMask(0x0);
	}
}

//toggle transparency on/off was encountering difficulties,
// so commenting out for now

/*void RenderModel::toggleTransparency(bool transparency) {
	std::cout << transparency << std::endl;
	osg::StateSet* set = this->getRootNode()->getOrCreateStateSet();
	if(transparency) {
		set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		//set->setMode(GL_BLEND,osg::StateAttribute::OVERRIDE);
		set->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA),
				osg::StateAttribute::OVERRIDE);
	} else {
		set->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		set->removeAttribute(osg::StateAttribute::BLENDFUNC);
	}
}*/

boost::shared_ptr<Model> RenderModel::getModel() {
	return model_.lock();
}

/**
 * Function taken from James Moliere, Open Scene Graph examples
 * x is red, y is green, z is blue
 */
void RenderModel::attachAxis(osg::Transform* transform) {

	std::cout << "Displaying axis: x=red, y=green, z=blue" << std::endl;

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

osg::ref_ptr<osg::Geode> RenderModel::getCylinder(float radius, float height,
		const osg::Vec4& color) {

	osg::ref_ptr<osg::Cylinder> cylinder(
			new osg::Cylinder(osg::Vec3(0, 0, 0), radius, height));
	osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable(
			new osg::ShapeDrawable(cylinder.get()));
	cylinderDrawable->setColor(color);
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

void RenderModel::activateTransparency(osg::StateSet* set) {
	set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	//set->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
	set->setMode(GL_BLEND,osg::StateAttribute::ON);
}

osg::ref_ptr<osg::PositionAttitudeTransform> RenderModel::attachBox(int label, float lengthX, float lengthY,
		float lengthZ, const osg::Vec4& color) {

	osg::ref_ptr<osg::Geode> box = this->getBox(fromOde(lengthX), fromOde(lengthY), fromOde(lengthZ), color);

	osg::ref_ptr<osg::PositionAttitudeTransform> pat(
			new osg::PositionAttitudeTransform());
	pat->addChild(box);

	this->primitives_->addChild(pat.get());
	pat->setUpdateCallback(new BodyCallback(this->getModel(), label));

	activateTransparency(box->getOrCreateStateSet());
	return pat;

}

osg::ref_ptr<osg::PositionAttitudeTransform> RenderModel::attachGeode(int label, osg::ref_ptr<osg::Geode> geode) {

	osg::ref_ptr<osg::PositionAttitudeTransform> pat(
			new osg::PositionAttitudeTransform());
	pat->addChild(geode);

	this->primitives_->addChild(pat.get());
	pat->setUpdateCallback(new BodyCallback(this->getModel(), label));
	activateTransparency(geode->getOrCreateStateSet());
	return pat;

}

std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > RenderModel::attachGeoms() {
	std::vector<osg::Vec4> colors;
	colors.push_back(osg::Vec4(1, 1, 1, 0.5));
	colors.push_back(osg::Vec4(0, 1, 0, 0.5));
	colors.push_back(osg::Vec4(0, 0, 1, 0.5));
	colors.push_back(osg::Vec4(1, 0, 0, 0.5));
	return this->attachGeoms(colors);
}


std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > RenderModel::attachGeoms(std::vector<osg::Vec4> colors) {
	std::vector<int> ids = this->getModel()->getIDs();
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pats;

	int count = 0;


	// iterate over the SimpleBodies, each will have a single geom
	for (unsigned int i=0; i<ids.size(); i++) {
		dGeomID g = this->getModel()->getBody(ids[i])->getGeom();

		int gclass = dGeomGetClass(g);


		switch (gclass) {
			case dBoxClass:
			{
				dVector3 lengths;
				dGeomBoxGetLengths(g, lengths);
				osg::ref_ptr<osg::PositionAttitudeTransform> pat =
						this->attachBox(
								ids[i],lengths[0], lengths[1],
								lengths[2], colors[count % colors.size()]);
				pats.push_back(pat);
				break;
			}

			case dCylinderClass :
			{
				dReal radius, length;
				dGeomCylinderGetParams(g, &radius, &length);
				osg::ref_ptr<osg::PositionAttitudeTransform> pat =
						this->attachGeode(ids[i],this->getCylinder(fromOde(radius),
								fromOde(length), colors[count % colors.size()]));
				pats.push_back(pat);
				break;
			}

			default:
			{
				std::cout << "not a box or cylinder," <<
						" not yet configured to draw!" << std::endl;
			}
		}

		count++;

	}
	return pats;


}


}
