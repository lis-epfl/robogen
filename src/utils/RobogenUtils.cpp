/*
 * @(#) RobogenUtils.cpp   1.0   Feb 17, 2013
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

#include <cmath>
#include <iostream>

#include "utils/RobogenUtils.h"
#include "PartList.h"

namespace robogen {

template<typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>& operator<<(
		std::basic_ostream<CharT, Traits>& out, osg::Vec3& o) {
	out << "(" << o.x() << ", " << o.y() << ", " << o.z() << ")";
	return out;
}

template<typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>& operator<<(
		std::basic_ostream<CharT, Traits>& out, osg::Quat& o) {
	out << "(" << o.w() << "|" << o.x() << ", " << o.y() << ", " << o.z()
			<< ")";
	return out;
}

const double RobogenUtils::OSG_EPSILON = 1e-7;
const double RobogenUtils::OSG_EPSILON_2 = 1e-6;

RobogenUtils::RobogenUtils() {

}

RobogenUtils::~RobogenUtils() {

}

void RobogenUtils::connect(boost::shared_ptr<Model> a, unsigned int slotA,
		boost::shared_ptr<Model> b, unsigned int slotB, float orientation,
		dJointGroupID connectionJointGroup, dWorldID odeWorld) {

	// Mandatory debug output
	#if 0
	// TODO make debug output based on something else
	std::cout << "Connecting " << a->getId() << " to " << b->getId()
			<< std::endl;
	#endif
	// 1) Rotate slotAxis of B such that we obtain a normal pointing inward the body
	osg::Vec3 bSlotAxis = b->getSlotAxis(slotB);
	osg::Vec3 bSlotAxisInv = -bSlotAxis;

	// 2) Find quaternion to rotate the vector pointing from the a root to the slot
	// and align it with B slot inward axis
	osg::Vec3 aCenterAxis = a->getSlotAxis(slotA); //aSlotPos - aCenter;

	osg::Quat rotAxisQuat; //= RobogenUtils::makeRotate(aCenterAxis, bSlotAxisInv);
	rotAxisQuat.makeRotate(aCenterAxis, bSlotAxisInv);
	a->setRootAttitude(rotAxisQuat);

	// 3) Compute A new center and translate it
	osg::Vec3 bSlotPos = b->getSlotPosition(slotB);
	osg::Vec3 aSlotNewPos = bSlotPos;
	osg::Vec3 aSlotPos = a->getSlotPosition(slotA);
	osg::Vec3 aTranslation = aSlotNewPos - aSlotPos;
	osg::Vec3 aCenter = a->getRootPosition();
	a->setRootPosition(aCenter + aTranslation);

	if (!RobogenUtils::areAxisParallel(a->getSlotAxis(slotA),
			-b->getSlotAxis(slotB))) {
		std::cout << "ALERT1!!!!!! axis not parallel" << std::endl;
	}

	// 4) At this point we need to orient the slots to the "zero" orientation

	// Basically, we could instantiate a Quat that aligns the two orientation
	// vectors, however, there is the very unfortunate special case when they are
	// antiparallel - in which it is not guaranteed that the axis is parallel to
	// the slot axis. What we do thus is extracting the angle and then rotating
	// about the slot axis ourselves.
	osg::Vec3 bSlotOrientation = b->getSlotOrientation(slotB);
	osg::Vec3 aSlotOrientation = a->getSlotOrientation(slotA);
	osg::Vec3 aSlotAxis = a->getSlotAxis(slotA);
	double angle;
	osg::Vec3 rotAxis;
	osg::Quat alignRot;
	alignRot.makeRotate(aSlotOrientation, bSlotOrientation);
	alignRot.getRotate(angle, rotAxis);
	// note: in quat, angles are always positive, axis direction determines
	// direction
	if (RobogenUtils::areAxisParallel(aSlotAxis, -rotAxis)) {
		angle *= -1;
	}
	//std::cout << "Angle: " << angle * 180 / M_PI << std::endl;
	osg::Quat slotAlignRotation;
	slotAlignRotation.makeRotate(angle, aSlotAxis);

	//std::cout << "bSlotOrientation: " << bSlotOrientation << std::endl;
	//std::cout << "aSlotOrientation: " << aSlotOrientation << std::endl;
	//std::cout << "slotAlignRotation: " << slotAlignRotation << std::endl;

	// ...and has the correct orientation
	if (abs(orientation) > 1e-6) {

		osg::Quat rotOrientationQuat;
		rotOrientationQuat.makeRotate(osg::inDegrees(orientation), aSlotAxis);
		a->setRootAttitude(slotAlignRotation * rotOrientationQuat);

	} else {

		a->setRootAttitude(slotAlignRotation);

	}

	// Final check, if the axis are not parallel, something is very wrong
	if (!RobogenUtils::areAxisParallel(a->getSlotAxis(slotA),
			-b->getSlotAxis(slotB))) {
		std::cout << "ALERT2!!!!!! axis not parallel" << std::endl;
	}

	// Create a joint to hold pieces in position

	dJointID joint = dJointCreateFixed(odeWorld, connectionJointGroup);
	dJointAttach(joint, a->getSlot(slotA), b->getSlot(slotB));
	dJointSetFixed(joint);

}

boost::shared_ptr<Model> RobogenUtils::createModel(
		const robogenMessage::BodyPart& bodyPart, dWorldID odeWorld,
		dSpaceID odeSpace) {

	// get part id
	const std::string &id = bodyPart.id();

	boost::shared_ptr<Model> model;
	if (bodyPart.type().compare(PART_TYPE_CORE_COMPONENT) == 0) {

		model.reset(new CoreComponentModel(odeWorld, odeSpace, id, true));

	} else if (bodyPart.type().compare(PART_TYPE_FIXED_BRICK) == 0) {

		model.reset(new CoreComponentModel(odeWorld, odeSpace, id, false));

	} else if (bodyPart.type().compare(PART_TYPE_PARAM_JOINT) == 0) {

		if (bodyPart.evolvableparam_size() != 3) {
			std::cerr
					<< "The parametric brick does not encode 3 parameters. Exiting."
					<< std::endl;
			return boost::shared_ptr<Model>();
		}

		model.reset(
				new ParametricBrickModel(odeWorld, odeSpace, id,
						bodyPart.evolvableparam(0).paramvalue(),
						bodyPart.evolvableparam(1).paramvalue(),
						bodyPart.evolvableparam(2).paramvalue()));

	} else if (bodyPart.type().compare(PART_TYPE_ROTATOR) == 0) {

		model.reset(new RotateJointModel(odeWorld, odeSpace, id));

	} else if (bodyPart.type().compare(PART_TYPE_PASSIVE_HINGE) == 0) {

		model.reset(new HingeModel(odeWorld, odeSpace, id));

	} else if (bodyPart.type().compare(PART_TYPE_ACTIVE_HINGE) == 0) {

		model.reset(new ActiveHingeModel(odeWorld, odeSpace, id));

	} else if (bodyPart.type().compare(PART_TYPE_PASSIVE_CARDAN) == 0) {

		model.reset(new CardanModel(odeWorld, odeSpace, id));

	} else if (bodyPart.type().compare(PART_TYPE_ACTIVE_CARDAN) == 0) {

		model.reset(new ActiveCardanModel(odeWorld, odeSpace, id));

	} else if (bodyPart.type().compare(PART_TYPE_PASSIVE_WHEEL) == 0) {

		// Read radius
		if (bodyPart.evolvableparam_size() != 1) {
			std::cerr
					<< "The passive wheel does not encode 1 parameter. Exiting."
					<< std::endl;
			return boost::shared_ptr<Model>();
		}

		model.reset(
				new PassiveWheelModel(odeWorld, odeSpace, id,
						bodyPart.evolvableparam(0).paramvalue()));

	} else if (bodyPart.type().compare(PART_TYPE_ACTIVE_WHEEL) == 0) {

		if (bodyPart.evolvableparam_size() != 1) {
			std::cerr
					<< "The active wheel does not encode 1 parameter. Exiting."
					<< std::endl;
			return boost::shared_ptr<Model>();
		}

		model.reset(
				new ActiveWheelModel(odeWorld, odeSpace, id,
						bodyPart.evolvableparam(0).paramvalue()));

	} else if (bodyPart.type().compare(PART_TYPE_ACTIVE_WHEG) == 0) {

		if (bodyPart.evolvableparam_size() != 1) {
			std::cerr << "The active wheg does not encode 1 parameter. Exiting."
					<< std::endl;
			return boost::shared_ptr<Model>();
		}

		model.reset(
				new ActiveWhegModel(odeWorld, odeSpace, id,
						bodyPart.evolvableparam(0).paramvalue()));

	} else if (bodyPart.type().compare(PART_TYPE_TOUCH_SENSOR) == 0) {

		model.reset(new TouchSensorModel(odeWorld, odeSpace, id));

	} else if (bodyPart.type().compare(PART_TYPE_LIGHT_SENSOR) == 0) {

		model.reset(new LightSensorModel(odeWorld, odeSpace, id, false));

	}

	// set orientation at slot to parent
	if (!model->setOrientationToParentSlot(bodyPart.orientation())) {
		std::cout << "Problem when setting orientation to parent slot of " << id
				<< std::endl;
	}

	return model;

}

boost::shared_ptr<RenderModel> RobogenUtils::createRenderModel(
		boost::shared_ptr<Model> model) {

	if (boost::dynamic_pointer_cast<CoreComponentModel>(model)) {

		return boost::shared_ptr<CoreComponentRenderModel>(
				new CoreComponentRenderModel(
						boost::dynamic_pointer_cast<CoreComponentModel>(model)));

	} else if (boost::dynamic_pointer_cast<ActiveCardanModel>(model)) {

		return boost::shared_ptr<ActiveCardanRenderModel>(
				new ActiveCardanRenderModel(
						boost::dynamic_pointer_cast<ActiveCardanModel>(model)));

	} else if (boost::dynamic_pointer_cast<ActiveHingeModel>(model)) {

		return boost::shared_ptr<ActiveHingeRenderModel>(
				new ActiveHingeRenderModel(
						boost::dynamic_pointer_cast<ActiveHingeModel>(model)));

	} else if (boost::dynamic_pointer_cast<ActiveWheelModel>(model)) {

		return boost::shared_ptr<ActiveWheelRenderModel>(
				new ActiveWheelRenderModel(
						boost::dynamic_pointer_cast<ActiveWheelModel>(model)));

	} else if (boost::dynamic_pointer_cast<CardanModel>(model)) {

		return boost::shared_ptr<CardanRenderModel>(
				new CardanRenderModel(
						boost::dynamic_pointer_cast<CardanModel>(model)));

	} else if (boost::dynamic_pointer_cast<HingeModel>(model)) {

		return boost::shared_ptr<HingeRenderModel>(
				new HingeRenderModel(
						boost::dynamic_pointer_cast<HingeModel>(model)));

	} else if (boost::dynamic_pointer_cast<ParametricBrickModel>(model)) {

		return boost::shared_ptr<ParametricBrickRenderModel>(
				new ParametricBrickRenderModel(
						boost::dynamic_pointer_cast<ParametricBrickModel>(
								model)));

	} else if (boost::dynamic_pointer_cast<PassiveWheelModel>(model)) {

		return boost::shared_ptr<PassiveWheelRenderModel>(
				new PassiveWheelRenderModel(
						boost::dynamic_pointer_cast<PassiveWheelModel>(model)));

	} else if (boost::dynamic_pointer_cast<RotateJointModel>(model)) {

		return boost::shared_ptr<RotateJointRenderModel>(
				new RotateJointRenderModel(
						boost::dynamic_pointer_cast<RotateJointModel>(model)));

	} else if (boost::dynamic_pointer_cast<ActiveWhegModel>(model)) {

		return boost::shared_ptr<ActiveWhegRenderModel>(
				new ActiveWhegRenderModel(
						boost::dynamic_pointer_cast<ActiveWhegModel>(model)));

	} else if (boost::dynamic_pointer_cast<TouchSensorModel>(model)) {

		return boost::shared_ptr<TouchSensorRenderModel>(
				new TouchSensorRenderModel(
						boost::dynamic_pointer_cast<TouchSensorModel>(model)));

	} else if (boost::dynamic_pointer_cast<LightSensorModel>(model)) {

		return boost::shared_ptr<LightSensorRenderModel>(
				new LightSensorRenderModel(
						boost::dynamic_pointer_cast<LightSensorModel>(model),
						boost::dynamic_pointer_cast<LightSensorModel>(model)->isInternal()));

	}
	return boost::shared_ptr<RenderModel>();
}

osg::Quat RobogenUtils::makeRotate(const osg::Vec3& from, const osg::Vec3& to) {
	osg::Quat rotation;
	rotation.makeRotate(from, to);

	// Check for opposite vectors
	// Same epsilon used in osg
	if ((rotation.w() < OSG_EPSILON) && (rotation.w() > -OSG_EPSILON)) {

		osg::Vec3 fromInverse = -from;
		if (areAxisParallel(fromInverse, to)) {

			// The inverse are parallel => they are pointing to the opposite directions

			// 1) Find an arbitrary perpendicular vector. We need to get the small element
			osg::Vec3 cardinalBasis;
			double x = fabs(from.x());
			double y = fabs(from.y());
			double z = fabs(from.z());
			if (x <= y && x <= z) {
				cardinalBasis.set(1, 0, 0);
			} else if (y <= x && y <= z) {
				cardinalBasis.set(0, 1, 0);
			} else {
				cardinalBasis.set(0, 0, 1);
			}
			osg::Vec3 perpendicular = from ^ cardinalBasis;
			rotation.makeRotate(osg::inDegrees(180.0), perpendicular);

		}

	}

	return rotation;
}

bool RobogenUtils::areAxisParallel(const osg::Vec3& a, const osg::Vec3& b) {

	if (fabs(a * b - a.length() * b.length()) < OSG_EPSILON) {
		return true;
	} else {
		return false;
	}
}

double RobogenUtils::getAngle(const osg::Vec3& a, const osg::Vec3& b) {

	if (areAxisParallel(a, -b)) {
		return 180.0;
	} else if (areAxisParallel(a, b)) {
		return 0.0;
	} else {
		return osg::RadiansToDegrees(acos((a * b) / (a.length() * b.length())));
	}
}

}
