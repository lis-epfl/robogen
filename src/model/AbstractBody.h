/*
 * AbstractBody.h
 *
 *  Created on: Sep 13, 2015
 *      Author: auerbach
 */

#ifndef ROBOGEN_PHYSICAL_BODY_H_
#define ROBOGEN_PHYSICAL_BODY_H_

#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <boost/enable_shared_from_this.hpp>

#include "Joint.h"

namespace robogen {

class Model;
class CompositeBody;

class AbstractBody : public boost::enable_shared_from_this<AbstractBody> {

public:
	inline dBodyID getBody() { return body_; }
	inline void setBody(dBodyID body) { body_ = body; }
	inline virtual ~AbstractBody() {
		/*if(body_) {
			printf("destroying body!!!\n");
			std::cout << body_ << std::endl;
			dBodyDestroy(body_);
			body_ = NULL;
		}*/
		joints_.clear();
	}
	inline void addJoint(boost::shared_ptr<Joint> joint) {
		joints_.push_back(joint);
	}

	inline void clearJoints() {
		joints_.clear();
	}

	void removeJoint(boost::shared_ptr<Joint> joint);

	inline const std::vector<boost::shared_ptr<Joint> > &getJoints() {
		return joints_;
	}


	virtual osg::Vec3 getPosition() = 0;
	virtual osg::Quat getAttitude() = 0;

	void setPosition(osg::Vec3 position);
	void setAttitude(osg::Quat attitude);

	inline void setParent(boost::shared_ptr<CompositeBody> parent) {
		parent_ = parent;
	}

	inline boost::shared_ptr<CompositeBody> getParent() {
		return parent_;
	}

	boost::shared_ptr<AbstractBody> getRoot();


protected:
	dBodyID body_;
	std::vector<boost::shared_ptr<Joint> > joints_;
	boost::shared_ptr<CompositeBody> parent_;
};

}

#endif /* ROBOGEN_SUBMODEL_H_ */
