/*
 * @(#) PassiveWheelRenderModel.cpp   1.0   Feb 13, 2013
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
#include <osgDB/ReadFile>

#include "render/callback/BodyCallback.h"
#include "render/components/PassiveWheelRenderModel.h"
#include "render/Mesh.h"

namespace robogen {

PassiveWheelRenderModel::PassiveWheelRenderModel(
      boost::shared_ptr<PassiveWheelModel> model) :
      RenderModel(model) {
   this->partA_.reset(new Mesh());
   this->partB_.reset(new Mesh());
}

PassiveWheelRenderModel::~PassiveWheelRenderModel() {

}

bool PassiveWheelRenderModel::initRenderModel() {

   bool meshLoadingA = this->partA_->loadMesh(
         "../models/PassiveRotation_Frame.stl");

   if (!meshLoadingA) {
      std::cerr << "[PassiveWheelRenderModel] Error loading model" << std::endl;
      return false;
   }

   bool meshLoadingB = this->partB_->loadMesh(
         "../models/PassiveRotation_Wheel.stl");

   if (!meshLoadingB) {
      std::cerr << "[PassiveWheelRenderModel] Error loading model" << std::endl;
      return false;
   }

   if (isDebugActive()) {
      this->showDebugView();
      return true;
   }

   // We need to rescale the wheel
   static const float BASE_RADIUS = 40;
   float radius = fromOde(
         boost::dynamic_pointer_cast < PassiveWheelModel
               > (this->getModel())->getRadius());

   float scale = radius/BASE_RADIUS;
   partB_->rescaleMesh(scale, scale, 1);

   partA_->setColor(osg::Vec4(1, 0, 0, 1));
   partB_->setColor(osg::Vec4(0, 1, 0, 1));

   // SLOT
   osg::ref_ptr<osg::PositionAttitudeTransform> slot = this->partA_->getMesh();
   slot->setAttitude(osg::Quat(osg::inDegrees(90.0), osg::Vec3(0, 1, 0)));

   osg::ref_ptr<osg::PositionAttitudeTransform> patSlot(
         new osg::PositionAttitudeTransform());
   patSlot->addChild(slot);

   this->getRootNode()->addChild(patSlot.get());
   patSlot->setUpdateCallback(
         new BodyCallback(this->getModel(), PassiveWheelModel::B_SLOT_ID));

   // WHEEL
   osg::ref_ptr<osg::PositionAttitudeTransform> wheel = this->partB_->getMesh();

   osg::ref_ptr<osg::PositionAttitudeTransform> patWheel(
         new osg::PositionAttitudeTransform());
   patWheel->addChild(wheel.get());

   this->getRootNode()->addChild(patWheel.get());
   patWheel->setUpdateCallback(
         new BodyCallback(this->getModel(), PassiveWheelModel::B_WHEEL_ID));

   return true;

}

void PassiveWheelRenderModel::showDebugView() {

   this->attachBox(PassiveWheelModel::B_SLOT_ID,
         PassiveWheelModel::SLOT_THICKNESS, PassiveWheelModel::SLOT_WIDTH,
         PassiveWheelModel::SLOT_WIDTH);

   osg::ref_ptr<osg::Geode> wheel = this->getCylinder(
         fromOde(
               boost::dynamic_pointer_cast < PassiveWheelModel
                     > (this->getModel())->getRadius()),
         fromOde(PassiveWheelModel::WHEEL_THICKNESS));

   // Wheel rotation
   osg::ref_ptr<osg::PositionAttitudeTransform> wheelRotation(
         new osg::PositionAttitudeTransform());
   wheelRotation->addChild(wheel);

   // WHEEL
   osg::ref_ptr<osg::PositionAttitudeTransform> patWheel(
         new osg::PositionAttitudeTransform());
   patWheel->addChild(wheelRotation);

   this->getRootNode()->addChild(patWheel.get());
   patWheel->setUpdateCallback(
         new BodyCallback(this->getModel(), PassiveWheelModel::B_WHEEL_ID));

   attachAxis(patWheel);

}

void PassiveWheelRenderModel::setColor(osg::Vec4 color) {
   this->partA_->setColor(color);
   this->partB_->setColor(color);
}

}
