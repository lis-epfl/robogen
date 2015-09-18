/*
 * Body.cpp
 *
 *  Created on: Sep 13, 2015
 *      Author: auerbach
 */

#include "Model.h"
#include "CompositeBody.h"
#include "SimpleBody.h"

namespace robogen {

CompositeBody::CompositeBody(std::vector<boost::shared_ptr<PhysicalBody> >
							subBodies, dWorldID world) {

	if(subBodies.size() < 2) {
		std::cerr << "Trying to create composite from less than 2 bodies!"
				<< std::endl;
		exitRobogen(EXIT_FAILURE);
	}
	body_ = dBodyCreate(world);
	dMassSetZero(&compositeMass_);

	for(size_t i; i<subBodies.size(); ++i) {
		this->addSubBody(subBodies[i]);
	}


	// finalize
	std::vector<boost::shared_ptr<SimpleBody> > simpleBodies =
			this->flattenSubBodies();

	for (unsigned int i=0; i<simpleBodies.size(); ++i) {
		dGeomSetBody(simpleBodies[i]->getGeom(), body_);
		const osg::Vec3 specifiedPosition = simpleBodies[i]->getSpecifiedPosition();
		dGeomSetOffsetPosition(simpleBodies[i]->getGeom(),
				specifiedPosition[0]-compositeMass_.c[0],
				specifiedPosition[1]-compositeMass_.c[1],
				specifiedPosition[2]-compositeMass_.c[2]);
		//dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);


	}
	dMassTranslate (&compositeMass_,-compositeMass_.c[0],-compositeMass_.c[1],
			-compositeMass_.c[2]);
	dBodySetMass (body_, &compositeMass_);

	//for (unsigned int i=0; i<geoms_.size(); ++i) {
	//	const dReal *pos = dGeomGetPosition(geoms_[i]);
	//	printf("position %02d % 1.7f % 1.7f % 1.7f\n", i, pos[0], pos[1], pos[2]);
	//}

	// finally we move the body so that geom_[0] is at (0,0,0)

	//const dReal *bodyPos = dBodyGetPosition(body_);
	//const dReal *pos = dGeomGetPosition(geoms_[0]);
	//dBodySetPosition(body_, bodyPos[0]-pos[0], bodyPos[1]-pos[1], bodyPos[2]-pos[2]);
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
		for(size_t i=0; i<composite->subBodies_.size(); ++i) {
			this->addSubBody(composite->subBodies_[i]);
		}
	} else if(boost::shared_ptr<SimpleBody> simple =
			boost::dynamic_pointer_cast<SimpleBody>(subBody)) {
		dMass componentMass;
		dMassSetZero(&componentMass);
		dMassAdd(&componentMass, &simple->getMass());
		const osg::Vec3 specifiedPosition = simple->getSpecifiedPosition();
		dMassTranslate (&componentMass,specifiedPosition[0],
				specifiedPosition[1], specifiedPosition[2]);
		dMassAdd (&compositeMass_,&componentMass);
	}
	if(subBody->getBody()) {
		dBodyDestroy(subBody->getBody());
		subBody->setBody(body_);
	}
}

std::vector<boost::shared_ptr<SimpleBody> > CompositeBody::flattenSubBodies() {
	std::vector<boost::shared_ptr<SimpleBody> > children;
	for (size_t i=0; i<subBodies_.size(); ++i) {
		if(boost::shared_ptr<CompositeBody> composite =
				boost::dynamic_pointer_cast<CompositeBody>(subBodies_[i])) {
			std::vector<boost::shared_ptr<SimpleBody> > grandChildren =
					composite->flattenSubBodies();
			children.insert(children.end(), grandChildren.begin(),
					grandChildren.end());
		} else if(boost::shared_ptr<SimpleBody> simple =
				boost::dynamic_pointer_cast<SimpleBody>(subBodies_[i])) {
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

}
