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
	SimpleBody(boost::weak_ptr<Model> model, dMass mass,
			dGeomID geom, const osg::Vec3& pos,
			const osg::Quat& attitude = osg::Quat());


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

	inline const boost::weak_ptr<Model>& getModel() {
		return model_;
	}

	inline const osg::Vec3& getSpecifiedPosition() {
		return specifiedPosition_;
	}

	inline const osg::Quat& getSpecifiedAttitude() {
		return specifiedAttitude_;
	}


private:

	dGeomID geom_;
	dMass mass_;
	boost::weak_ptr<Model> model_;
	osg::Vec3 specifiedPosition_;
	osg::Quat specifiedAttitude_;
};

}

#endif /* ROBOGEN_SUBMODEL_H_ */
