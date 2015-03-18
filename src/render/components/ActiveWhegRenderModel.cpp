/*
 * @(#) ActiveWhegRenderModel.cpp   1.0   Feb 27, 2013
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
      return true;
   }

   partA_->setColor(osg::Vec4(1, 0, 0, 1));
   partB_->setColor(osg::Vec4(0, 1, 0, 1));

   float slotCorrectionZ = inMm(0); //not needed with new model

   // SLOT
   osg::ref_ptr<osg::PositionAttitudeTransform> slot = this->partA_->getMesh();
   slot->setAttitude(osg::Quat(osg::inDegrees(90.0), osg::Vec3(0, 1, 0)));
   slot->setPosition(
         fromOde(
               osg::Vec3(ActiveWhegModel::X_SERVO +
            		   ActiveWhegModel::WHEG_THICKNESS/2
            		   - ActiveWhegModel::SEPARATION,
            		   0,slotCorrectionZ)));
   //attachAxis(slot);
   osg::ref_ptr<osg::PositionAttitudeTransform> patSlot(
         new osg::PositionAttitudeTransform());
   patSlot->addChild(slot);

   this->getRootNode()->addChild(patSlot.get());
   patSlot->setUpdateCallback(
         new BodyCallback(this->getModel(), ActiveWhegModel::B_SLOT_ID));


   // WHEG
   osg::ref_ptr<osg::PositionAttitudeTransform> wheg = this->partB_->getMesh();


   wheg->setAttitude(osg::Quat(osg::inDegrees(90.0), osg::Vec3(0, 0, 1))
   	   	   	   	   	* osg::Quat(osg::inDegrees(180.0), osg::Vec3(0, 1, 0)));

   wheg->setPosition(osg::Vec3(11.5, 0, -2.5));

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

   this->getRootNode()->addChild(patWheg.get());
   patWheg->setUpdateCallback(
         new BodyCallback(this->getModel(), ActiveWhegModel::B_WHEG_BASE));

   return true;

}

void ActiveWhegRenderModel::showDebugView() {

   this->attachBox(ActiveWhegModel::B_SLOT_ID, ActiveWhegModel::SLOT_THICKNESS,
         ActiveWhegModel::SLOT_WIDTH, ActiveWhegModel::SLOT_WIDTH);

   this->attachBox(ActiveWhegModel::B_SERVO_ID, ActiveWhegModel::SERVO_LENGTH,
         ActiveWhegModel::SERVO_WIDTH, ActiveWhegModel::SERVO_HEIGHT);


   this->attachBox(ActiveWhegModel::B_WHEG_SPOKE_1,
         ActiveWhegModel::WHEG_THICKNESS, ActiveWhegModel::WHEG_WIDTH,
         boost::dynamic_pointer_cast < ActiveWhegModel
               > (this->getModel())->getRadius());

   this->attachBox(ActiveWhegModel::B_WHEG_SPOKE_2,
         ActiveWhegModel::WHEG_THICKNESS, ActiveWhegModel::WHEG_WIDTH,
         boost::dynamic_pointer_cast < ActiveWhegModel
               > (this->getModel())->getRadius());

   this->attachBox(ActiveWhegModel::B_WHEG_SPOKE_3,
         ActiveWhegModel::WHEG_THICKNESS, ActiveWhegModel::WHEG_WIDTH,
         boost::dynamic_pointer_cast < ActiveWhegModel
               > (this->getModel())->getRadius());


   osg::ref_ptr<osg::Geode> wheel = this->getCylinder(
         fromOde(ActiveWhegModel::WHEG_BASE_RADIUS),
         fromOde(ActiveWhegModel::WHEG_THICKNESS),
         osg::Vec4(1,0,0,1));

   // Wheel rotation
   osg::ref_ptr<osg::PositionAttitudeTransform> wheelRotation(
         new osg::PositionAttitudeTransform());
   wheelRotation->addChild(wheel);

   osg::Quat rotateWheel;
   rotateWheel.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));


   // WHEEL
   osg::ref_ptr<osg::PositionAttitudeTransform> patWheel(
         new osg::PositionAttitudeTransform());
   patWheel->addChild(wheelRotation);

   this->getRootNode()->addChild(patWheel.get());
   patWheel->setUpdateCallback(
         new BodyCallback(this->getModel(), ActiveWhegModel::B_WHEG_BASE));

   //attachAxis(patWheel);

}

void ActiveWhegRenderModel::setColor(osg::Vec4 color) {
   this->partA_->setColor(color);
   this->partB_->setColor(color);
}

}
