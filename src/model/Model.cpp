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
		odeWorld_(odeWorld), odeSpace_(odeSpace), id_(id) {
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
	return this->getPosition(this->getRoot());
}

osg::Quat Model::getRootAttitude() {
	return this->getAttitude(this->getRoot());
}

void Model::setRootPosition(const osg::Vec3& pos) {

	osg::Vec3 curPosition = this->getRootPosition();
	osg::Vec3 translation = pos - curPosition;

	std::map<int, dBodyID>::iterator it = this->bodies_.begin();
	for (; it != this->bodies_.end(); ++it) {
		osg::Vec3 curBodyPos = this->getPosition(it->second);
		curBodyPos += translation;
		dBodySetPosition(it->second, curBodyPos.x(), curBodyPos.y(),
				curBodyPos.z());
	}
}

void Model::translateRootPosition(const osg::Vec3& translation) {

	osg::Vec3 newPosition = this->getRootPosition() + translation;
	this->setRootPosition(newPosition);

}

void Model::setRootAttitude(const osg::Quat& quat) {

	osg::Vec3 rootPosition = this->getRootPosition();

	std::map<int, dBodyID>::iterator it = this->bodies_.begin();
	for (; it != this->bodies_.end(); ++it) {

		osg::Vec3 curPosition = this->getPosition(it->second);
		osg::Vec3 relPosition = curPosition - rootPosition;

		// Rotate relPosition
		osg::Vec3 newPosition = quat * relPosition;
		dBodySetPosition(it->second, newPosition.x(), newPosition.y(),
				newPosition.z());

		osg::Quat curBodyAttitude = this->getAttitude(it->second);
		curBodyAttitude *= quat;

		//curBodyAttitude = quat;

		dQuaternion quatOde;
		quatOde[0] = curBodyAttitude.w();
		quatOde[1] = curBodyAttitude.x();
		quatOde[2] = curBodyAttitude.y();
		quatOde[3] = curBodyAttitude.z();

		dBodySetQuaternion(it->second, quatOde);
	}

	this->setRootPosition(rootPosition);

}

osg::Vec3 Model::getPosition(dBodyID body) {
	const dReal* boxVec = dBodyGetPosition(body);
	return osg::Vec3(boxVec[0], boxVec[1], boxVec[2]);
}

osg::Quat Model::getAttitude(dBodyID body) {
	const dReal* boxQuat = dBodyGetQuaternion(body);
	return (osg::Quat(boxQuat[1], boxQuat[2], boxQuat[3], boxQuat[0]));
}

osg::Vec3 Model::getBodyPosition(int id) {
	return this->getPosition(this->getBody(id));
}

osg::Quat Model::getBodyAttitude(int id) {
	return this->getAttitude(this->getBody(id));
}

dBodyID Model::getBody(int id) {
	std::map<int, dBodyID>::iterator it = this->bodies_.find(id);
	if (it == this->bodies_.end()) {
		std::cout
				<< "[Model] Error: The specified body does not exists in this model"
				<< std::endl;
		assert(it != this->bodies_.end());
		return NULL;
	}
	return bodies_[id];
}

std::vector<dBodyID> Model::getBodies() {

	std::vector<dBodyID> bodies;
	std::map<int, dBodyID>::iterator it = this->bodies_.begin();
	for (; it != this->bodies_.end(); ++it) {
		bodies.push_back(it->second);
	}
	return bodies;

}

void Model::addBody(dBodyID body, int id) {
	this->bodies_.insert(std::pair<int, dBodyID>(id, body));
}

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

dxGeom* Model::createBoxGeom(dBodyID body, float mass, const osg::Vec3& pos,
		float lengthX, float lengthY, float lengthZ) {

	dMass massOde;
	dMassSetBoxTotal(&massOde, mass, lengthX, lengthY, lengthZ);
	dBodySetMass(body, &massOde);
	dxGeom* g = dCreateBox(this->getCollisionSpace(), lengthX, lengthY,
			lengthZ);
	dBodySetPosition(body, pos.x(), pos.y(), pos.z());
	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());
	dGeomSetBody(g, body);
	return g;

}

dxGeom* Model::createCylinderGeom(dBodyID body, float mass,
		const osg::Vec3& pos, int direction, float radius, float height) {

	dMass massOde;
	dMassSetCylinderTotal(&massOde, mass, direction, radius, height);
	dBodySetMass(body, &massOde);
	dxGeom* g = dCreateCylinder(this->getCollisionSpace(), radius, height);
	dBodySetPosition(body, pos.x(), pos.y(), pos.z());
	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());

	if (direction == 1) {

		osg::Quat rotateCylinder;
		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
		dQuaternion quatOde;
		quatOde[0] = rotateCylinder.w();
		quatOde[1] = rotateCylinder.x();
		quatOde[2] = rotateCylinder.y();
		quatOde[3] = rotateCylinder.z();
		dBodySetQuaternion(body, quatOde);

	} else if (direction == 2) {

		osg::Quat rotateCylinder;
		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
		dQuaternion quatOde;
		quatOde[0] = rotateCylinder.w();
		quatOde[1] = rotateCylinder.x();
		quatOde[2] = rotateCylinder.y();
		quatOde[3] = rotateCylinder.z();
		dBodySetQuaternion(body, quatOde);

	}

	dGeomSetBody(g, body);

	return g;

}

dxGeom* Model::createCapsuleGeom(dBodyID body, float mass, const osg::Vec3& pos,
		int direction, float radius, float height) {

	dMass massOde;
	dMassSetCapsuleTotal(&massOde, mass, direction, radius, height);
	dBodySetMass(body, &massOde);
	dxGeom* g = dCreateCapsule(this->getCollisionSpace(), radius, height);
	dBodySetPosition(body, pos.x(), pos.y(), pos.z());
	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());

	if (direction == 1) {

		osg::Quat rotateCapsule;
		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
		dQuaternion quatOde;
		quatOde[0] = rotateCapsule.w();
		quatOde[1] = rotateCapsule.x();
		quatOde[2] = rotateCapsule.y();
		quatOde[3] = rotateCapsule.z();
		dBodySetQuaternion(body, quatOde);

	} else if (direction == 2) {

		osg::Quat rotateCapsule;
		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
		dQuaternion quatOde;
		quatOde[0] = rotateCapsule.w();
		quatOde[1] = rotateCapsule.x();
		quatOde[2] = rotateCapsule.y();
		quatOde[3] = rotateCapsule.z();
		dBodySetQuaternion(body, quatOde);

	}

	dGeomSetBody(g, body);

	return g;

}

dJointID Model::fixBodies(dBodyID b1, dBodyID b2, const osg::Vec3& /*axis*/) {
	dJointID joint = dJointCreateFixed(this->getPhysicsWorld(), 0);
	dJointAttach(joint, b1, b2);
	dJointSetFixed(joint);
	return joint;

}

bool Model::setOrientationToParentSlot(int orientation){
	if (orientation < 0 || orientation > 3){
		std::cout << "Specified orientation to parent slot is not"\
				" between 0 and 3." << std::endl;
		return false;
	}
	this->orientationToParentSlot_ = orientation;
	return true;
}

int Model::getOrientationToParentSlot(){
	return this->orientationToParentSlot_;
}

}
