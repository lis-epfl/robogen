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

#define NO_FIXED_JOINTS

#include "model/Model.h"
#include <stdexcept>
#include <sstream>

#include "CompositeBody.h"

namespace robogen {

Model::Model(dWorldID odeWorld, dSpaceID odeSpace, std::string id) :
		odeWorld_(odeWorld), odeSpace_(odeSpace), id_(id),
		orientationToParentSlot_(0), orientationToRoot_(0) {
}

Model::~Model() {

}

const std::string &Model::getId(){
	return id_;
}

dWorldID Model::getPhysicsWorld() {
	return this->odeWorld_;
}

dSpaceID Model::getCollisionSpace() {
	return this->odeSpace_;
}

osg::Vec3 Model::getRootPosition() {
	return this->getRoot()->getPosition();
}

osg::Quat Model::getRootAttitude() {
	return this->getRoot()->getAttitude();
}

std::set<boost::shared_ptr<AbstractBody> > Model::bodiesToMove() {
	std::set<boost::shared_ptr<AbstractBody> > rootBodies;

	for(size_t i=0; i<bodies_.size(); ++i) {
		boost::shared_ptr<AbstractBody> bodyRoot = bodies_[i]->getRoot();
		boost::shared_ptr<CompositeBody> composite =
						boost::dynamic_pointer_cast<CompositeBody>(bodyRoot);
		if( !(composite && composite->isMultiModel()) ) {
			rootBodies.insert(bodyRoot);
		}
	}
	return rootBodies;
}


void Model::setRootPosition(const osg::Vec3& pos) {

	osg::Vec3 curPosition = this->getRootPosition();
	osg::Vec3 translation = pos - curPosition;

	std::set<boost::shared_ptr<AbstractBody> > rootBodies = bodiesToMove();

	for(std::set<boost::shared_ptr<AbstractBody> >::iterator it =
			rootBodies.begin(); it!=rootBodies.end(); ++it) {

		osg::Vec3 curBodyPos = (*it)->getPosition();

		curBodyPos += translation;

		(*it)->setPosition(curBodyPos);
	}
}

void Model::translateRootPosition(const osg::Vec3& translation) {

	osg::Vec3 newPosition = this->getRootPosition() + translation;
	this->setRootPosition(newPosition);

}

void Model::setRootAttitude(const osg::Quat& quat) {

	osg::Vec3 rootPosition = this->getRootPosition();

	std::set<boost::shared_ptr<AbstractBody> > rootBodies = bodiesToMove();

	for(std::set<boost::shared_ptr<AbstractBody> >::iterator it =
			rootBodies.begin(); it!=rootBodies.end(); ++it) {
		//osg::Vec3 curPosition = it->second->getPosition();
		//const dReal *position = dBodyGetPosition((*it)->getBody());
		osg::Vec3 curPosition = (*it)->getPosition();//osg::Vec3(position[0], position[1], position[2]);
		osg::Vec3 relPosition = curPosition - rootPosition;

		// Rotate relPosition
		osg::Vec3 newPosition = quat * relPosition;
		(*it)->setPosition(newPosition);

		osg::Quat curBodyAttitude = (*it)->getAttitude();
		curBodyAttitude *= quat;

		(*it)->setAttitude(curBodyAttitude);
	}

	this->setRootPosition(rootPosition);

}

osg::Vec3 Model::getBodyPosition(int id) {
	return this->getBody(id)->getPosition();
}

osg::Quat Model::getBodyAttitude(int id) {
	return this->getBody(id)->getAttitude();
}

boost::shared_ptr<SimpleBody> Model::getBody(int id) {
	std::map<int, boost::shared_ptr<SimpleBody> >::iterator it = this->bodies_.find(id);
	if (it == this->bodies_.end()) {
		std::cout
				<< "[Model] Error: The specified body does not exists in this "
				<< " model" << std::endl;
		assert(it != this->bodies_.end());
		return boost::shared_ptr<SimpleBody>();
	}
	return bodies_[id];
}

std::vector<boost::shared_ptr<SimpleBody> > Model::getBodies() {

	std::vector<boost::shared_ptr<SimpleBody> > bodies;
	std::map<int, boost::shared_ptr<SimpleBody> >::iterator it = this->bodies_.begin();
	for (; it != this->bodies_.end(); ++it) {
		bodies.push_back(it->second);
	}
	return bodies;

}

std::vector<int> Model::getIDs() {
	std::vector<int> bodies;
	std::map<int, boost::shared_ptr<SimpleBody> >::iterator it = this->bodies_.begin();
	for (; it != this->bodies_.end(); ++it) {
		bodies.push_back(it->first);
	}
	return bodies;

}

void Model::addBody(boost::shared_ptr<SimpleBody> body, int id) {
	this->bodies_.insert(std::pair<int, boost::shared_ptr<SimpleBody> >(id, body));
}

boost::shared_ptr<SimpleBody> Model::addBox(float mass,
		const osg::Vec3& pos, float lengthX, float lengthY, float lengthZ,
		int label) {

	dMass massOde;
	dMassSetBoxTotal(&massOde, mass, lengthX, lengthY, lengthZ);
	dxGeom* g = dCreateBox(this->getCollisionSpace(), lengthX, lengthY,
							lengthZ);
	boost::shared_ptr<SimpleBody> body(new SimpleBody(shared_from_this(),
			massOde, g, pos));
	this->addBody(body, label);
	return body;
}

boost::shared_ptr<SimpleBody> Model::addCylinder(float mass,
		const osg::Vec3& pos, int direction, float radius, float height,
		int label) {

	dMass massOde;
	dMassSetCylinderTotal(&massOde, mass, direction, radius, height);
	dxGeom* g = dCreateCylinder(this->getCollisionSpace(), radius, height);
	osg::Quat rotateCylinder;
	if (direction == 1) {
		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
	} else if (direction == 2) {
		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
	}

	boost::shared_ptr<SimpleBody> body(new SimpleBody(shared_from_this(),
				massOde, g, pos, rotateCylinder));
	this->addBody(body, label);
	return body;

}

boost::shared_ptr<SimpleBody> Model::addCapsule(float mass,
		const osg::Vec3& pos, int direction, float radius, float height,
		int label) {

	dMass massOde;
	dMassSetCapsuleTotal(&massOde, mass, direction, radius, height);
	dxGeom* g = dCreateCapsule(this->getCollisionSpace(), radius, height);

	osg::Quat rotateCapsule;

	if (direction == 1) {
		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
	} else if (direction == 2) {
		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
	}

	boost::shared_ptr<SimpleBody> body(new SimpleBody(shared_from_this(),
					massOde, g, pos, rotateCapsule));
	this->addBody(body, label);
	return body;
}

void Model::fixBodies(std::vector<boost::shared_ptr<AbstractBody> > bodies) {
	boost::shared_ptr<CompositeBody> composite(new CompositeBody());
	// shared_ptr is maintained on the child bodies
	composite->init(bodies,this->getPhysicsWorld());
}

void Model::fixBodies(boost::shared_ptr<SimpleBody> b1,
						  boost::shared_ptr<SimpleBody> b2) {

#ifdef NO_FIXED_JOINTS
	std::vector<boost::shared_ptr<AbstractBody> > bodies;
	bodies.push_back(b1);
	bodies.push_back(b2);
	fixBodies(bodies);


#else
	boost::shared_ptr<Joint> joint(new Joint());
	joint->createFixed(this->getPhysicsWorld(),b1, b2);
	joints_.push_back(joint);
#endif

}

boost::shared_ptr<Joint> Model::attachWithHinge(
		boost::shared_ptr<SimpleBody> b1, boost::shared_ptr<SimpleBody> b2,
		osg::Vec3 axis, osg::Vec3 anchor) {
	boost::shared_ptr<Joint> joint(new Joint());
	joint->createHinge(this->getPhysicsWorld(), b1, b2, axis, anchor);
	joints_.push_back(joint);
	return joint;
}

boost::shared_ptr<Joint> Model::attachWithUniversal(
		boost::shared_ptr<SimpleBody> b1, boost::shared_ptr<SimpleBody> b2,
		osg::Vec3 axis1, osg::Vec3 axis2, osg::Vec3 anchor) {
	boost::shared_ptr<Joint> joint(new Joint());
	joint->createUniversal(this->getPhysicsWorld(), b1, b2, axis1, axis2,
			anchor);
	joints_.push_back(joint);
	return joint;
}

bool Model::setOrientationToParentSlot(int orientation){
	if (orientation < 0 || orientation > 3){
		std::cout << "Specified orientation to parent slot is not"\
				" between 0 and 3." << std::endl;
		return false;
	}
	int deltaOrientation = orientation - this->orientationToParentSlot_ ;
	this->orientationToParentSlot_ = orientation;
	this->orientationToRoot_ = modulo(this->orientationToRoot_ +
										deltaOrientation, 4);
	return true;
}

int Model::getOrientationToParentSlot(){
	return this->orientationToParentSlot_;
}

bool Model::setParentOrientation(int orientation) {
	if (orientation < 0 || orientation > 3){
		std::cout << "Specified parent orientation is not"\
				" between 0 and 3." << std::endl;
		return false;
	}
	this->orientationToRoot_ = modulo(this->getOrientationToParentSlot() +
								orientation, 4);
	return true;
}

int Model::getOrientationToRoot() {
	return this->orientationToRoot_;
}

void Model::removeJoint(boost::shared_ptr<Joint> joint) {
	joints_.erase(std::remove(joints_.begin(), joints_.end(), joint),
			joints_.end());
}


}
