/*
 * Body.h
 *
 *  Created on: Sep 13, 2015
 *      Author: auerbach
 */

#ifndef ROBOGEN_SIMPLE_BODY_H_
#define ROBOGEN_SIMPLE_BODY_H_

#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "PhysicalBody.h"


namespace robogen {

class Model;

class SimpleBody : public PhysicalBody {

public:
	SimpleBody(boost::shared_ptr<Model> model, dMass mass,
			dGeomID geom, const osg::Vec3& pos,
			const osg::Quat& attitude = osg::Quat());

	inline ~SimpleBody() {
		/*if(geom_) {
			printf("destroying geom!!!\n");
			std::cout << geom_ << std::endl;
			dGeomDestroy(geom_);
			geom_ = NULL;
		}*/
	}


	osg::Vec3 getPosition();
	osg::Quat getAttitude();

	osg::Vec3 getLocalPosition();
	osg::Quat getLocalAttitude();

	inline const dGeomID& getGeom() {
		return geom_;
	}

	inline const dMass& getMass() {
		return mass_;
	}

	const boost::weak_ptr<Model>& getModel();

	inline const osg::Vec3& getSpecifiedPosition() {
		return specifiedPosition_;
	}

	inline const osg::Quat& getSpecifiedAttitude() {
		return specifiedAttitude_;
	}

	inline void setSpecifiedPosition(osg::Vec3 specifiedPosition) {
		specifiedPosition_ = specifiedPosition;
	}

	inline void setSpecifiedAttitude(osg::Quat specifiedAttitude) {
		specifiedAttitude_ = specifiedAttitude;
	}



private:

	boost::weak_ptr<Model> model_;
	dMass mass_;
	dGeomID geom_;

	osg::Vec3 specifiedPosition_;
	osg::Quat specifiedAttitude_;
};

}

#endif /* ROBOGEN_SUBMODEL_H_ */
