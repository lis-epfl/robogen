/*
 * @(#) SimpleBody.h   1.0   September 16, 2015
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

#ifndef ROBOGEN_SIMPLE_BODY_H_
#define ROBOGEN_SIMPLE_BODY_H_

#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "AbstractBody.h"

namespace robogen {

class Model;

class SimpleBody : public AbstractBody {

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
		joints_.clear();
	}

	void addJoint(boost::shared_ptr<Joint> joint);
	void clearJoints();

	void removeJoint(boost::shared_ptr<Joint> joint);

	inline const std::vector<boost::shared_ptr<Joint> > &getJoints() {
		return joints_;
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

	//overwrite parent methods to also update specified position / attitude
	inline void setPosition(osg::Vec3 position) {
		AbstractBody::setPosition(position);
		setSpecifiedPosition(position);
	}

	inline void setAttitude(osg::Quat attitude) {
		AbstractBody::setAttitude(attitude);
		setSpecifiedAttitude(attitude);
	}



private:

	boost::weak_ptr<Model> model_;
	dMass mass_;
	dGeomID geom_;

	osg::Vec3 specifiedPosition_;
	osg::Quat specifiedAttitude_;

	std::vector<boost::shared_ptr<Joint> > joints_;
};

}

#endif /* ROBOGEN_SIMPLE_BODY_H_ */
