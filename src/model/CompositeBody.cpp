/*
 * @(#) CompositeBody.cpp   1.0   September 16, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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
#include <sstream>

#include "Model.h"
#include "CompositeBody.h"
#include "SimpleBody.h"

//#define DEBUG_MERGE
//#define DEBUG_MERGE_ADVANCED
namespace robogen {

void CompositeBody::init(std::vector<boost::shared_ptr<AbstractBody> >
							subBodies, dWorldID world, bool multiModel) {

	multiModel_ = multiModel;

	// want to keep insertion order concstent, so use vector, but use set to get unique elements
	std::vector<boost::shared_ptr<AbstractBody> > rootBodies;
	std::set<boost::shared_ptr<AbstractBody> > uniqueRoots;

	for(size_t i=0; i<subBodies.size(); ++i) {
#ifdef DEBUG_MERGE
		if (i==0) {
			std::cout <<  ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> "
					<< "merging >>>>>>>>>>>>> " << std::endl;
		}
		std::cout << "\t" << subBodies[i] << " (" << subBodies[i]->getBody() << ")"
			<< " with root "
			<< subBodies[i]->getRoot() << " ("
			<< subBodies[i]->getRoot()->getBody() << ")"<< std::endl;
#endif
		if (uniqueRoots.count(subBodies[i]->getRoot()) == 0) {
			uniqueRoots.insert(subBodies[i]->getRoot());
			rootBodies.push_back(subBodies[i]->getRoot());
		}
	}

	if(rootBodies.size() < 2) {
		std::cerr << "Trying to create composite from less than root 2 bodies!"
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	body_ = dBodyCreate(world);
	dMassSetZero(&compositeMass_);

	bodiesToDestroy_.clear();
	rotations_.clear();
#ifdef DEBUG_MERGE
	std::cout << "Adding " << subBodies.size() << " sub bodies." << std::endl;
	std::cout << "  already " << subBodies_.size() << " sub bodies" << std::endl;
#endif
	for(size_t i=0; i<rootBodies.size(); ++i) {
		this->addSubBody(rootBodies[i]);
	}
#ifdef DEBUG_MERGE
	std::cout << subBodies_.size() << " sub bodies added " << std::endl;
#endif
	// finalize
	std::vector<boost::weak_ptr<SimpleBody> > simpleBodies =
			this->flattenSubBodies();
#ifdef DEBUG_MERGE
	std::cout << "COMPOSITE BODY WITH " << simpleBodies.size() << " geoms" << std::endl;
#endif
	for (size_t i=0; i<simpleBodies.size(); ++i) {
		osg::Vec3 currentPosition = simpleBodies[i].lock()->getPosition();
		osg::Quat currentAttitude = simpleBodies[i].lock()->getAttitude();
#ifdef DEBUG_MERGE_ADVANCED
		std::cout << "PRE SETTING OFFSET " <<
				currentPosition[0] << " " << currentPosition[1] << " " <<
				currentPosition[2] << std::endl;
#endif


#ifdef DEBUG_MERGE_ADVANCED
		osg::Vec3 specifiedPosition = simpleBodies[i].lock()->getSpecifiedPosition();
		std::cout << "SPECIFIED " <<
				specifiedPosition[0] << " " << specifiedPosition[1] << " " <<
				specifiedPosition[2] << std::endl;
#endif
		simpleBodies[i].lock()->setSpecifiedPosition(currentPosition);
		simpleBodies[i].lock()->setSpecifiedAttitude(currentAttitude);

		dGeomClearOffset(simpleBodies[i].lock()->getGeom());
		dGeomSetBody(simpleBodies[i].lock()->getGeom(), body_);
//		const osg::Vec3 specifiedPosition = simpleBodies[i].lock()->getSpecifiedPosition();
//		const osg::Quat specifiedAttitude = simpleBodies[i].lock()->getSpecifiedAttitude();
		dGeomSetOffsetPosition(simpleBodies[i].lock()->getGeom(),
				currentPosition[0]-compositeMass_.c[0],
				currentPosition[1]-compositeMass_.c[1],
				currentPosition[2]-compositeMass_.c[2]);
#ifdef DEBUG_MERGE_ADVANCED
		std::cout << "-------------GEOM ROTATION-----------------------\n";
		std::cout << "\t\t"
				<< currentAttitude[0] <<  " " <<
				currentAttitude[1] << " " << currentAttitude[2] << " "<<
				currentAttitude[3] << std::endl;
#endif
		dMatrix3 rotationMatrixOde;
		getRotationMatrixOde(/*currentAttitude*/ rotations_[simpleBodies[i].lock()->getGeom()], rotationMatrixOde);
		dGeomSetOffsetRotation(simpleBodies[i].lock()->getGeom(),
				rotationMatrixOde);
		currentPosition = simpleBodies[i].lock()->getPosition();
#ifdef DEBUG_MERGE_ADVANCED
		std::cout << "POST SETTING OFFSET " <<
						currentPosition[0] << " " << currentPosition[1] << " " <<
						currentPosition[2] << std::endl;

		//dGeomSetOffsetWorldPosition(simpleBodies[i].lock()->getGeom(),
		//		currentPosition.x(), currentPosition.y(), currentPosition.z());
		std::cout << simpleBodies[i].lock() << std::endl;
#endif
	}

	// update all joint axis and anchors before destroying bodies
	// so that they are correctly set below when reconnected
	std::vector<boost::shared_ptr<Joint> > allJoints = this->getAllJoints();
	for(size_t i = 0; i < allJoints.size(); ++i) {
		allJoints[i]->updateAxisAndAngle();
	}

	// set all descendants to point to body of this composite
	updateDescendantBodies();

	// destroy all bodies that are no longer needed
	for(std::set<dBodyID>::iterator
			it=bodiesToDestroy_.begin(); it!=bodiesToDestroy_.end(); ++it) {
		dBodyDestroy((*it));
	}

	// center mass
	dMassTranslate (&compositeMass_,-compositeMass_.c[0],-compositeMass_.c[1],
			-compositeMass_.c[2]);
	dBodySetMass (body_, &compositeMass_);

	// finally we move the body so that geom_[0] is at its offset
	{
		const osg::Vec3 specifiedPos = simpleBodies[0].lock()->getSpecifiedPosition();
		const dReal *bodyPos = dBodyGetPosition(body_);
		const dReal *pos = dGeomGetPosition(simpleBodies[0].lock()->getGeom());

#ifdef DEBUG_MERGE
		printf("body position  % 1.7f % 1.7f % 1.7f\n", bodyPos[0], bodyPos[1], bodyPos[2]);
		printf("geom position  % 1.7f % 1.7f % 1.7f\n", pos[0], pos[1], pos[2]);
		printf("requested position % 1.7f % 1.7f % 1.7f\n",
			specifiedPos[0],
			specifiedPos[1],
			specifiedPos[2]);
#endif
		dBodySetPosition(body_, bodyPos[0]-pos[0] + specifiedPos[0],
								bodyPos[1]-pos[1] + specifiedPos[1],
								bodyPos[2]-pos[2] + specifiedPos[2]);

	}

#ifdef DEBUG_MERGE
	{
		const dReal *pos = dGeomGetPosition(simpleBodies[0].lock()->getGeom());
		printf("new position  % 1.7f % 1.7f % 1.7f\n", pos[0], pos[1], pos[2]);
	}
#endif



	// reconnect all joints
	//std::vector<boost::shared_ptr<Joint> > allJoints = this->getAllJoints();

#ifdef DEBUG_MERGE
	std::cout << "Reconnecting " << allJoints.size() << " joints" << std::endl;
#endif
	for(size_t i = 0; i < allJoints.size(); ++i) {
		// need to update axis
		allJoints[i]->reconnect();

	}
#ifdef DEBUG_MERGE
	std::cout << std::endl;
#endif
	//for (unsigned int i=0; i<geoms_.size(); ++i) {
	//		const dReal *pos = dGeomGetPosition(geoms_[i]);
	//		printf("position %02d % 1.7f % 1.7f % 1.7f\n", i, pos[0], pos[1], pos[2]);
	//	}

}



CompositeBody::~CompositeBody() {

}


void CompositeBody::updateDescendantBodies() {
	for(size_t i=0; i<subBodies_.size(); ++i) {
		boost::shared_ptr<AbstractBody> subBody = subBodies_[i].lock();
		subBody->setBody(body_);
		if(boost::shared_ptr<CompositeBody> composite =
				boost::dynamic_pointer_cast<CompositeBody>(subBody)) {
			composite->updateDescendantBodies();
		}
	}
}

void CompositeBody::addSubBody(boost::shared_ptr<AbstractBody> subBody,
		bool directDescendant) {


	if(boost::shared_ptr<CompositeBody> composite =
			boost::dynamic_pointer_cast<CompositeBody>(subBody)) {
#ifdef DEBUG_MERGE
		std::cout << "Adding composite! " << composite << std::endl;
#endif
		for(size_t i=0; i<composite->subBodies_.size(); ++i) {
			this->addSubBody(composite->subBodies_[i].lock(),
					false);
		}
	} else if(boost::shared_ptr<SimpleBody> simple =
			boost::dynamic_pointer_cast<SimpleBody>(subBody)) {
#ifdef DEBUG_MERGE
		std::cout << "Adding simple! " << simple << std::endl;
#endif
		dMass componentMass;
		dMassSetZero(&componentMass);
		dMassAdd(&componentMass, &simple->getMass());
#ifdef DEBUG_MERGE
		std::cout << "================================\n";
		std::cout << "\tspecified: " << simple->getSpecifiedPosition()[0] << " "
				 << simple->getSpecifiedPosition()[1] << " "
				 << simple->getSpecifiedPosition()[2] << std::endl;
		std::cout << "\tactual: " << simple->getPosition()[0] << " "
						 << simple->getPosition()[1] << " "
						 << simple->getPosition()[2] << std::endl;
		std::cout << "================================\n";
#endif

		// do these operations as the same order as demo_boxstack:
		// dMassRotate then dMassTranslate then dMassAdd

		const osg::Vec3 currentPosition = simple->getPosition();//getSpecifiedPosition();
		const osg::Quat currentRotation = simple->getAttitude();
		dMatrix3 rotationMatrixOde;
		getRotationMatrixOde(currentRotation, rotationMatrixOde);
		dMassRotate(&componentMass, rotationMatrixOde);

		rotations_[simple->getGeom()] = currentRotation;

		dMassTranslate (&componentMass,currentPosition[0],
				currentPosition[1], currentPosition[2]);

		dMassAdd (&compositeMass_,&componentMass);


#ifdef DEBUG_MERGE_ADVANCED
		std::cout << "-------------MASS ROTATION-----------------------\n";
		std::cout << "\t\t" << currentRotation[0] << " " << currentRotation[1]
		        << " " << currentRotation[2]
		        << " " << currentRotation[3] << std::endl;
#endif


	} else {
		std::cerr << "Invalid Body Type" << std::endl;
		exitRobogen(EXIT_FAILURE);
	}

	//don't update the body yet, that will happen after
	bodiesToDestroy_.insert(subBody->getBody());


	if(directDescendant) {
		subBody->setParent(boost::dynamic_pointer_cast<CompositeBody>(shared_from_this()));
		subBodies_.push_back(subBody);
	}

#ifdef DEBUG_MERGE
	std::cout << subBody->getJoints().size() << " JOINTS" << std::endl;


	for(size_t i = 0; i < subBody->getJoints().size(); ++i) {
		std::cout << subBody->getJoints()[i] << " " << subBody->getJoints()[i]->getBodyA().lock()
				<< " " << subBody->getJoints()[i]->getBodyB().lock() << std::endl;
	}
#endif

}

std::vector<boost::weak_ptr<SimpleBody> > CompositeBody::flattenSubBodies() {
	std::vector<boost::weak_ptr<SimpleBody> > children;
	for (size_t i=0; i<subBodies_.size(); ++i) {
		if(boost::shared_ptr<CompositeBody> composite =
				boost::dynamic_pointer_cast<CompositeBody>(subBodies_[i].lock())) {
			std::vector<boost::weak_ptr<SimpleBody> > grandChildren =
					composite->flattenSubBodies();
#ifdef DEBUG_MERGE
			std::cout << "composite " << shared_from_this() << " has " << children.size() <<
					" geoms" << std::endl;
#endif
			children.insert(children.end(), grandChildren.begin(),
					grandChildren.end());
		} else if(boost::shared_ptr<SimpleBody> simple =
				boost::dynamic_pointer_cast<SimpleBody>(subBodies_[i].lock())) {
			children.push_back(simple);
		}
	}
	return children;
}

osg::Vec3 CompositeBody::getPosition() {
	const dReal* pos = dBodyGetPosition(body_);
	return osg::Vec3(pos[0], pos[1], pos[2]);
}



osg::Quat CompositeBody::getAttitude() {
	const dReal* quat = dBodyGetQuaternion(body_);
	return (osg::Quat(quat[1], quat[2], quat[3], quat[0]));

}

std::vector<boost::shared_ptr<Joint> > CompositeBody::getAllJoints() {
	std::vector<boost::shared_ptr<Joint> > allJoints;
	std::vector<boost::weak_ptr<SimpleBody> > children = flattenSubBodies();
	for(size_t i=0; i<children.size(); ++i) {
		allJoints.insert(allJoints.end(), children[i].lock()->getJoints().begin(),
				children[i].lock()->getJoints().end());
	}
	return allJoints;
}


std::string posString(AbstractBody* body) {
	std::stringstream ss;
	ss << body->getPosition()[0] << ", " << body->getPosition()[1] << ", " <<
			body->getPosition()[2];
	return ss.str();
}


std::string CompositeBody::str(int indent) {
	std::stringstream ss;

	for(int i=0; i<indent; ++i)
		ss << "\t";

	ss << "body: " << body_ << " (" << subBodies_.size() << " sub bodies) ";
	ss << posString(this);
	ss << std::endl;
	for (size_t i=0; i<subBodies_.size(); ++i) {
		if(boost::shared_ptr<CompositeBody> composite =
				boost::dynamic_pointer_cast<CompositeBody>(subBodies_[i].lock())) {
			ss << composite->str(indent + 1);
		} else if(boost::shared_ptr<SimpleBody> simple =
					boost::dynamic_pointer_cast<SimpleBody>(subBodies_[i].lock())) {
			for(int j=0; j<indent+1; ++j)
					ss << "\t";
			ss << simple->getModel().lock()->getId() << " " ;
			ss << posString(simple.get());
			ss << std::endl;
		}
	}


	return ss.str();
}


}
