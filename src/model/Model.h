/*
 * @(#) Model.h   1.0   Feb 8, 2013
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
#ifndef ROBOGEN_MODEL_H_
#define ROBOGEN_MODEL_H_

#include <osg/PositionAttitudeTransform>
#include <iostream>
#include <map>
#include <stdexcept>

#include "Robogen.h"
#include "render/RenderModel.h"

namespace robogen {

/**
 * A slot center must always be place on the external surface of the slot
 */
class Model {

public:

	/**
	 * Constructor
	 */
	Model(dWorldID odeWorld, dSpaceID odeSpace, std::string id);

	/**
	 * Destructor
	 */
	virtual ~Model();

	/**
	 * @return true if the model initialization completed successfully
	 */
	virtual bool initModel() = 0;

	/**
	 * @return id string of the part
	 */
	const std::string &getId();

	/**
	 * @return the body corresponding to the selected slot
	 */
	virtual dBodyID getSlot(unsigned int i) = 0;

	/**
	 * @return the slot position, in world coordinates
	 */
	virtual osg::Vec3 getSlotPosition(unsigned int i) = 0;

	/**
	 * @return the slot axis (normal to the slot)
	 */
	virtual osg::Vec3 getSlotAxis(unsigned int i) = 0;

	/**
	 * Tangent to the slot surface, defines the zero rotation orientation
	 * on that slot
	 */
	virtual osg::Vec3 getSlotOrientation(unsigned int i) = 0;

	/**
	 * @return the root body
	 */
	virtual dBodyID getRoot() = 0;

	/**
	 * @return the position of the root part
	 */
	osg::Vec3 getRootPosition();

	/**
	 * @return the attitude of the root part (a quaternion)
	 */
	osg::Quat getRootAttitude();

	/**
	 * @return the position of the specified body
	 */
	osg::Vec3 getBodyPosition(int id);

	/**
	 * @return the attitude of the specified body
	 */
	osg::Quat getBodyAttitude(int id);

	/**
	 * Sets the position of the root part.
	 * Will force all the other bodies to translate accordingly.
	 */
	void setRootPosition(const osg::Vec3& pos);

	/**
	 * Translate the root part of the specified amount
	 * Will force all the other bodies to translate accordingly.
	 */
	void translateRootPosition(const osg::Vec3& translation);

	/**
	 * Sets the attitude of the root part (a quaternion).
	 * Will force all the other bodies to rotate accordingly.
	 */
	void setRootAttitude(const osg::Quat& quat);

	/**
	 * @return the position of the given body
	 */
	osg::Vec3 getPosition(dBodyID body);

	/**
	 * @return the attitude of the given body
	 */
	osg::Quat getAttitude(dBodyID body);

	/**
	 * @return the physics world
	 */
	dWorldID getPhysicsWorld();

	/**
	 * @return the collision space
	 */
	dSpaceID getCollisionSpace();

	/**
	 * @return the specified body
	 */
	dBodyID getBody(int id);

	/**
	 * @return all the bodies belonging to this model
	 */
	std::vector<dBodyID> getBodies();

	/**
	 * Create the specified body
	 * @param body label. If the label is negative, does not register the body in the list of bodies of this model.
	 */
	dBodyID createBody(int label);
	dBodyID createBody();

	/**
	 * Create a box geometry for the body
	 * @param body
	 * @param mass
	 * @param pos
	 * @param lengthX
	 * @param lengthY
	 * @param lengthZ
	 */
	dxGeom* createBoxGeom(dBodyID body, float mass, const osg::Vec3& pos,
			float lengthX, float lengthY, float lengthZ);

	/**
	 * Create a capsule geometry for the body
	 * @param body
	 * @param mass
	 * @param pos
	 * @param direction
	 * @param radius
	 * @param height
	 */
	dxGeom* createCapsuleGeom(dBodyID body, float mass, const osg::Vec3& pos,
			int direction, float radius, float height);

	/**
	 * Create a cylinder geometry for the body
	 * @param body
	 * @param mass
	 * @param pos
	 * @param direction
	 * @param radius
	 * @param height
	 */
	dxGeom* createCylinderGeom(dBodyID body, float mass, const osg::Vec3& pos,
			int direction, float radius, float height);

	/**
	 * Fix bodies together
	 * @param b1 first body
	 * @param b2 second body
	 * @param axis the axis along which the bodies will be aligned
	 * @param a slider joint
	 */
	dJointID fixBodies(dBodyID b1, dBodyID b2, const osg::Vec3& axis);

	/**
	 * Set orientation to parent slot with increments of 90 degrees
	 * @param orientation integer between 0 and 3, specifying the amount of
	 * additional 90 degree increments after attaching the part to its parent.
	 */
	bool setOrientationToParentSlot(int orientation);

	/**
	 * Get orientation to parent slot.
	 * @return a number specifying the amount of additional 90 degree increments
	 * after attaching the part to its parent.
	 */
	int getOrientationToParentSlot();

protected:

	/**
	 * Add a body to the model
	 * @param body
	 */
	void addBody(dBodyID body, int id);

private:

	/**
	 * ODE world
	 */
	dWorldID odeWorld_;

	/**
	 * ODE collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * User-defined identifier of the part
	 */
	const std::string id_;

	/**
	 * ODE Bodies composing the model
	 */
	std::map<int, dBodyID> bodies_;

	/**
	 * Orientation at parent slot: 0-3, where the number stands for
	 * increments of 90 degrees when attaching to the parent part.
	 */
	int orientationToParentSlot_;

};

}

#endif /* ROBOGEN_MODEL_H_ */
