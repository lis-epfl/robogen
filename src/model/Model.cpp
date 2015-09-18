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
#include "model/Model.h"
#include <stdexcept>
#include <sstream>

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

void Model::setRootPosition(const osg::Vec3& pos) {

	osg::Vec3 curPosition = this->getRootPosition();
	osg::Vec3 translation = pos - curPosition;

	std::map<int, boost::shared_ptr<SimpleBody> >::iterator it = this->bodies_.begin();
	for (; it != this->bodies_.end(); ++it) {
		osg::Vec3 curBodyPos = it->second->getPosition();
		curBodyPos += translation;
		dBodySetPosition(it->second->getBody(), curBodyPos.x(), curBodyPos.y(),
				curBodyPos.z());
	}
}

void Model::translateRootPosition(const osg::Vec3& translation) {

	osg::Vec3 newPosition = this->getRootPosition() + translation;
	this->setRootPosition(newPosition);

}

void Model::setRootAttitude(const osg::Quat& quat) {

	osg::Vec3 rootPosition = this->getRootPosition();

	std::map<int, boost::shared_ptr<SimpleBody> >::iterator it =
			this->bodies_.begin();
	for (; it != this->bodies_.end(); ++it) {

		osg::Vec3 curPosition = it->second->getPosition();
		osg::Vec3 relPosition = curPosition - rootPosition;

		// Rotate relPosition
		osg::Vec3 newPosition = quat * relPosition;
		dBodySetPosition(it->second->getBody(), newPosition.x(), newPosition.y(),
				newPosition.z());

		osg::Quat curBodyAttitude = it->second->getAttitude();
		curBodyAttitude *= quat;

		//curBodyAttitude = quat;

		dQuaternion quatOde;
		quatOde[0] = curBodyAttitude.w();
		quatOde[1] = curBodyAttitude.x();
		quatOde[2] = curBodyAttitude.y();
		quatOde[3] = curBodyAttitude.z();

		dBodySetQuaternion(it->second->getBody(), quatOde);
	}

	this->setRootPosition(rootPosition);

}

//osg::Vec3 Model::getPosition(dBodyID body) {
//	const dReal* boxVec = dBodyGetPosition(body);
//	return osg::Vec3(boxVec[0], boxVec[1], boxVec[2]);
//}

//osg::Quat Model::getAttitude(dBodyID body) {
//	const dReal* boxQuat = dBodyGetQuaternion(body);
//	return (osg::Quat(boxQuat[1], boxQuat[2], boxQuat[3], boxQuat[0]));
//}

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

/*
dBodyID Model::createBody(int label) {
	dBodyID b = dBodyCreate(this->getPhysicsWorld());
	if (label >= 0) {
		this->addBody(b, label);
	}
	return b;
}

dBodyID Model::createBody() {
	return this->createBody(-1);
}
*/

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

boost::shared_ptr<Joint> Model::fixBodies(boost::shared_ptr<SimpleBody> b1,
						  boost::shared_ptr<SimpleBody> b2) {

	return boost::shared_ptr<Joint>(new Joint(this->getPhysicsWorld(),
			b1, b2));
}

boost::shared_ptr<Joint> Model::attachWithHinge(
		boost::shared_ptr<SimpleBody> b1, boost::shared_ptr<SimpleBody> b2,
		osg::Vec3 axis, osg::Vec3 anchor) {

	return boost::shared_ptr<Joint>(new Joint(this->getPhysicsWorld(),
			b1, b2, axis, anchor));
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

}
