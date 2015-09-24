/*
 * Body.cpp
 *
 *  Created on: Sep 13, 2015
 *      Author: auerbach
 */

#include <sstream>

#include "Model.h"
#include "CompositeBody.h"
#include "SimpleBody.h"

namespace robogen {

void CompositeBody::init(std::vector<boost::shared_ptr<PhysicalBody> >
							subBodies, dWorldID world) {


	if(subBodies.size() < 2) {
		std::cerr << "Trying to create composite from less than 2 bodies!"
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	}
	body_ = dBodyCreate(world);
	dMassSetZero(&compositeMass_);

	std::cout << "Adding " << subBodies.size() << " sub bodies." << std::endl;

	for(size_t i=0; i<subBodies.size(); ++i) {
		this->addSubBody(subBodies[i]);
	}

	std::cout << subBodies_.size() << " sub bodies added " << std::endl;

	// finalize
	std::vector<boost::weak_ptr<SimpleBody> > simpleBodies =
			this->flattenSubBodies();

	for (size_t i=0; i<simpleBodies.size(); ++i) {
		dGeomSetBody(simpleBodies[i].lock()->getGeom(), body_);
		const osg::Vec3 specifiedPosition = simpleBodies[i].lock()->getSpecifiedPosition();
		dGeomSetOffsetPosition(simpleBodies[i].lock()->getGeom(),
				specifiedPosition[0]-compositeMass_.c[0],
				specifiedPosition[1]-compositeMass_.c[1],
				specifiedPosition[2]-compositeMass_.c[2]);
		//dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);


	}

	std::cout << "COMPOSITE BODY WITH " << simpleBodies.size() << " geoms" << std::endl;

	dMassTranslate (&compositeMass_,-compositeMass_.c[0],-compositeMass_.c[1],
			-compositeMass_.c[2]);
	dBodySetMass (body_, &compositeMass_);

	// finally we move the body so that geom_[0] is at its offset
	{
		const osg::Vec3 specifiedPos = simpleBodies[0].lock()->getSpecifiedPosition();
		const dReal *bodyPos = dBodyGetPosition(body_);

		printf("position  % 1.7f % 1.7f % 1.7f\n", bodyPos[0], bodyPos[1], bodyPos[2]);
		const dReal *pos = dGeomGetPosition(simpleBodies[0].lock()->getGeom());
		printf("position  % 1.7f % 1.7f % 1.7f\n", pos[0], pos[1], pos[2]);
		printf("requested position % 1.7f % 1.7f % 1.7f\n",
			specifiedPos[0],
			specifiedPos[1],
			specifiedPos[2]);

		dBodySetPosition(body_, bodyPos[0]-pos[0] + specifiedPos[0],
								bodyPos[1]-pos[1] + specifiedPos[1],
								bodyPos[2]-pos[2] + specifiedPos[2]);

	}

	{
		const dReal *pos = dGeomGetPosition(simpleBodies[0].lock()->getGeom());
		printf("new position  % 1.7f % 1.7f % 1.7f\n", pos[0], pos[1], pos[2]);
	}
	//for (unsigned int i=0; i<geoms_.size(); ++i) {
	//		const dReal *pos = dGeomGetPosition(geoms_[i]);
	//		printf("position %02d % 1.7f % 1.7f % 1.7f\n", i, pos[0], pos[1], pos[2]);
	//	}

}



CompositeBody::~CompositeBody() {

}




void CompositeBody::addSubBody(boost::shared_ptr<PhysicalBody> subBody) {

	if(boost::shared_ptr<CompositeBody> composite =
			boost::dynamic_pointer_cast<CompositeBody>(subBody)) {
		std::cout << "Adding composite!" << std::endl;
		dBodyDestroy(subBody->getBody());
		for(size_t i=0; i<composite->subBodies_.size(); ++i) {
			this->addSubBody(composite->subBodies_[i].lock());
		}
	} else if(boost::shared_ptr<SimpleBody> simple =
			boost::dynamic_pointer_cast<SimpleBody>(subBody)) {
		std::cout << "Adding simple!" << std::endl;
		dMass componentMass;
		dMassSetZero(&componentMass);
		dMassAdd(&componentMass, &simple->getMass());
		const osg::Vec3 specifiedPosition = simple->getSpecifiedPosition();
		dMassTranslate (&componentMass,specifiedPosition[0],
				specifiedPosition[1], specifiedPosition[2]);
		dMassAdd (&compositeMass_,&componentMass);
		// if it is a sub body of some composite than the body will have
		// already been destroyed
		if(!simple->getParent()) {
			dBodyDestroy(simple->getBody());
		}
		simple->setBody(body_);
	} else {
		std::cout << "WTF!!!!" << std::endl;
	}
	subBody->setParent(shared_from_this());
	subBodies_.push_back(subBody);

}

std::vector<boost::weak_ptr<SimpleBody> > CompositeBody::flattenSubBodies() {
	std::vector<boost::weak_ptr<SimpleBody> > children;
	for (size_t i=0; i<subBodies_.size(); ++i) {
		if(boost::shared_ptr<CompositeBody> composite =
				boost::dynamic_pointer_cast<CompositeBody>(subBodies_[i].lock())) {
			std::vector<boost::weak_ptr<SimpleBody> > grandChildren =
					composite->flattenSubBodies();
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

std::string posString(PhysicalBody* body) {
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
			for(int i=0; i<indent+1; ++i)
					ss << "\t";
			ss << simple->getModel().lock()->getId() << " " ;
			ss << posString(simple.get());
			ss << std::endl;
		}
	}


	return ss.str();
}


}
