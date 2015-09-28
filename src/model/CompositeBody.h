/*
 * CompositeBody.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: auerbach
 */



#ifndef ROBOGEN_COMPOSITEBODY_CPP_
#define ROBOGEN_COMPOSITEBODY_CPP_


#include "Robogen.h"
#include <vector>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>



#include "AbstractBody.h"
#include "SimpleBody.h"

namespace robogen {

class CompositeBody : public AbstractBody {

public:
	//errorless constructor
	CompositeBody() { }

	void init(std::vector<boost::shared_ptr<AbstractBody> > subBodies,
			dWorldID world, bool multiModel = false);

	virtual ~CompositeBody();

	virtual osg::Vec3 getPosition();
	virtual osg::Quat getAttitude();

	inline const std::vector<boost::weak_ptr<AbstractBody> > &getSubBodies() {
		return subBodies_;
	}

	inline bool isMultiModel() {
		return multiModel_;
	}

	std::vector<boost::shared_ptr<Joint> > getAllJoints();

	std::string str(int indent=0);

private:

	void addSubBody(boost::shared_ptr<AbstractBody> subBody,
			bool directDescendant = true);
	void updateDescendantBodies();

	std::vector<boost::weak_ptr<SimpleBody> > flattenSubBodies();


	dMass compositeMass_;
	std::vector<boost::weak_ptr<AbstractBody> > subBodies_;
	bool multiModel_;
	std::set<dBodyID> bodiesToDestroy_;
	std::map<dGeomID, osg::Quat> rotations_;


};

}

#endif /* COMPOSITEBODY_CPP_ */


