/*
 * @(#) ActiveWhegRenderModel.cpp   1.0   Feb 27, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2016 Andrea Maesani, Joshua Auerbach
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
#include <osgDB/ReadFile>

#include "render/callback/BodyCallback.h"
#include "render/components/ActiveWhegRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

ActiveWhegRenderModel::ActiveWhegRenderModel(
      boost::shared_ptr<ActiveWhegModel> model) :
      RenderModel(model) {
   this->partA_.reset(new Mesh());
   this->partB_.reset(new Mesh());
}

ActiveWhegRenderModel::~ActiveWhegRenderModel() {

}

bool ActiveWhegRenderModel::initRenderModel() {

   bool meshLoadingA = this->partA_->loadMesh(
		   RobogenUtils::getMeshFile(this->getModel(),
			  ActiveWhegModel::B_SLOT_ID));

   if (!meshLoadingA) {
      std::cerr << "[ActiveWhegRenderModel] Error loading model" << std::endl;
      return false;
   }

   bool meshLoadingB = this->partB_->loadMesh(
		   RobogenUtils::getMeshFile(this->getModel(),
			  ActiveWhegModel::B_WHEG_BASE));

   if (!meshLoadingB) {
      std::cerr << "[ActiveWhegRenderModel] Error loading model" << std::endl;
      return false;
   }

   if (isDebugActive()) {
      this->showDebugView();
   }

   partA_->setColor(osg::Vec4(1, 0, 0, 0.5));
   partB_->setColor(osg::Vec4(0, 1, 0, 0.5));

   // SLOT
   osg::ref_ptr<osg::PositionAttitudeTransform> slot = this->partA_->getMesh();
   slot->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
			  ActiveWhegModel::B_SLOT_ID));

   slot->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
		   ActiveWhegModel::B_SLOT_ID));

   osg::ref_ptr<osg::PositionAttitudeTransform> patSlot(
         new osg::PositionAttitudeTransform());
   patSlot->addChild(slot);

   this->getMeshes()->addChild(patSlot.get());
   patSlot->setUpdateCallback(
         new BodyCallback(this->getModel(), ActiveWhegModel::B_SLOT_ID));

   // WHEG
   osg::ref_ptr<osg::PositionAttitudeTransform> wheg = this->partB_->getMesh();


   wheg->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
      			  ActiveWhegModel::B_WHEG_BASE));
   wheg->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
   			ActiveWhegModel::B_WHEG_BASE));


   // We need to rescale the wheel
   static const float BASE_RADIUS = 38;
   float radius = fromOde(
         boost::dynamic_pointer_cast < ActiveWhegModel
               > (this->getModel())->getRadius());

   float scale = radius / BASE_RADIUS;
   partB_->rescaleMesh(scale, scale, 1);

   wheg = partB_->getMesh();

   osg::ref_ptr<osg::PositionAttitudeTransform> patWheg(
         new osg::PositionAttitudeTransform());
   patWheg->addChild(wheg.get());

   this->getMeshes()->addChild(patWheg.get());
   patWheg->setUpdateCallback(
         new BodyCallback(this->getModel(), ActiveWhegModel::B_WHEG_BASE));

   if(isDebugActive()) {
	   this->activateTransparency(patSlot->getOrCreateStateSet());
	   this->activateTransparency(patWheg->getOrCreateStateSet());
	   attachAxis(patWheg);
   }
   return true;

}

void ActiveWhegRenderModel::showDebugView() {
	this->attachGeoms();
}

void ActiveWhegRenderModel::setColor(osg::Vec4 color) {
   this->partA_->setColor(color);
   this->partB_->setColor(color);
}

}
