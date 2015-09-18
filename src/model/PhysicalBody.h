/*
 * Body.h
 *
 *  Created on: Sep 13, 2015
 *      Author: auerbach
 */

#ifndef ROBOGEN_PHYSICAL_BODY_H_
#define ROBOGEN_PHYSICAL_BODY_H_

#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>


#include "Joint.h"

namespace robogen {

class Model;

class PhysicalBody : public boost::enable_shared_from_this<PhysicalBody> {

public:
	inline dBodyID getBody() { return body_; }
	inline void setBody(dBodyID body) { body_ = body; }
	inline virtual ~PhysicalBody() {
		dBodyDestroy(body_);
		joints.clear();
	}
	inline void addJoint(boost::shared_ptr<Joint> joint) {
		joints.push_back(joint);
	}
	virtual osg::Vec3 getPosition() = 0;
	virtual osg::Quat getAttitude() = 0;

protected:
	dBodyID body_;
	std::vector<boost::shared_ptr<Joint> > joints;
};

}

#endif /* ROBOGEN_SUBMODEL_H_ */
