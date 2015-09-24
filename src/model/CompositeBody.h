/*
 * CompositeBody.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: auerbach
 */

#if 0

#ifndef ROBOGEN_COMPOSITEBODY_CPP_
#define ROBOGEN_COMPOSITEBODY_CPP_


#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>


#include "PhysicalBody.h"
#include "SimpleBody.h"

namespace robogen {

class CompositeBody : public PhysicalBody,
	public boost::enable_shared_from_this<CompositeBody> {

public:
	//errorless constructor
	CompositeBody() { }

	void init(std::vector<boost::shared_ptr<PhysicalBody> > subBodies,
			dWorldID world);

	virtual ~CompositeBody();

	virtual osg::Vec3 getPosition();
	virtual osg::Quat getAttitude();

	inline const std::vector<boost::weak_ptr<PhysicalBody> > &getSubBodies() {
		return subBodies_;
	}

	std::string str(int indent=0);

private:

	void addSubBody(boost::shared_ptr<PhysicalBody> subBody);
	std::vector<boost::weak_ptr<SimpleBody> > flattenSubBodies();

	dMass compositeMass_;
	std::vector<boost::weak_ptr<PhysicalBody> > subBodies_;



};

}

#endif /* COMPOSITEBODY_CPP_ */

#endif
