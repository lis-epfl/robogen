/*
 * PartRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include <iostream>
#include <sstream>
#include <math.h>

#include "evolution/representation/PartRepresentation.h"
#include "PartList.h"


namespace robogen {

PartRepresentation::PartRepresentation(std::string id, unsigned int orientation,
		unsigned int arity, const std::string& type,
		const std::vector<double>& params,
		const std::vector<std::string>& motors,
		const std::vector<std::string>& sensors) :
		id_(id), orientation_(orientation), arity_(arity), type_(type),
		parent_(NULL), params_(params), motors_(motors), sensors_(sensors) {

	children_.resize(arity_, boost::shared_ptr<PartRepresentation>());
}

PartRepresentation::~PartRepresentation() {

}

const std::string &PartRepresentation::getId() {
	return id_;
}

void PartRepresentation::setId(std::string newid) {
	id_ = newid;
}

unsigned int PartRepresentation::getOrientation() {
	return orientation_;
}

void PartRepresentation::setOrientation(unsigned int orientation) {
	orientation_ = orientation;
}

unsigned int PartRepresentation::getArity() {
	return arity_;
}

bool PartRepresentation::setArity(unsigned int arity, std::string partType){

//check if the bodyPartType as the right to mutate its connection
	if(!PART_TYPE_IS_VARIABLE_ARITY_MAP.at(partType))
		return false;
//check if the newArity is in the range	
	std::pair<unsigned int, unsigned int> range = 
		PART_TYPE_VARIABLE_ARITY_RANGE_MAP.at(partType);
	if(arity<range.first || arity>range.second)
		return false;

	arity_ = arity;
	return true;
}

unsigned int PartRepresentation::numDescendants() {

	int descendants = 0;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		// child and all its children
		if (children_[i].get()) {
			++descendants += children_[i]->numDescendants();
		}
	}
	return descendants;

}

unsigned int PartRepresentation::getChildrenCount() {

	int count = 0;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		if (children_[i].get()) {
			count++;
		}
	}
	return count;

}

std::vector<unsigned int> PartRepresentation::getFreeSlots() {

	std::vector<unsigned int> freeSlots;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		if (!children_[i].get()) {
			freeSlots.push_back(i);
		}
	}
	return freeSlots;

}

const std::string &PartRepresentation::getType() {
	return type_;
}

boost::shared_ptr<PartRepresentation> PartRepresentation::getChild(unsigned int n) {
	if (n  >= arity_ ) {
		std::cout << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_ << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}
	return children_[n];
}

bool PartRepresentation::setChild(unsigned int n,
		boost::shared_ptr<PartRepresentation> part) {

	if (n  >= arity_ ) {
		std::cout << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with arity " << arity_ << std::endl;
		return false;
	}
	// don't try to access part if void
	if (part) {
		part->setParent(this);
		part->setPosition(n);
	}
	children_[n] = part;
	return true;

}

bool PartRepresentation::setChildren(std::vector<boost::shared_ptr<PartRepresentation>> children){
	
	unsigned int childrenSize = children.size();
	boost::shared_ptr<PartRepresentation> child;

	// check if the partType has the right to use this function
	if(PART_TYPE_IS_VARIABLE_ARITY_MAP.at(this->getType()) == false){
		std::cout 	<< "The part "
					<< this->getType()
					<< " hasn't the right to use the function" 
					<< " PartRepresentation::setChildren"
					<< std::endl;
		return false;
	}
	//check if the vector children size = arity
	if(children.size() != this->getArity()){
		std::cout	<< "RobotRepresentation::setChildPosition: "
					<< "children vector size is "
					<< children.size()
					<< " and must be the same of the arity, so "
					<< this->getArity()
					<< std::endl;
		return false;
	}
//Gael Debug*************************************************************************
	std::cout 	<< "Children = [";
	for(int i=0; i<children.size(); i++){
		if(children[i] != NULL)
			std::cout 	<< " "
						<< i;
		else
			std::cout 	<< " NULL";
	}
	std::cout 	<< " ]"
				<< std::endl;
//***********************************************************************************

	children_.resize(childrenSize, boost::shared_ptr<PartRepresentation>());
	for(int i=0; i<childrenSize; i++){
		// don't try to access part if void
		child = children[i];
		if (child) {
			child->setParent(this);
			child->setPosition(i);
		}
		children_[i] = child;	
	}

//Gael Debug*************************************************************************
	std::cout 	<< "Children_ = [";
	for(int i=0; i<children.size(); i++){
		if(children_[i] != NULL)
			std::cout 	<< " "
						<< i;
		else
			std::cout 	<< " NULL";
	}
	std::cout 	<< " ]"
				<< std::endl;
//***********************************************************************************

std::cout << "*******************setChildrenEND********************"
			<<std::endl;
	return true;
}

boost::shared_ptr<PartRepresentation> PartRepresentation::create(char type,
		std::string id, unsigned int orientation, std::vector<double> params) {
	int arity;
std::cout << "CREATE patram size " << params.size() << std::endl;
	if (PART_TYPE_MAP.count(type) == 0) {
		std::cout << "Unknown part type '" << type << "'" << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}

	std::string partType = PART_TYPE_MAP.at(type);
	if (params.size() != PART_TYPE_PARAM_COUNT_MAP.at(partType)) {
		std::cout << "The parameter count (" << params.size()
				<< ") does not equal the requested parameter count ("
				<< PART_TYPE_PARAM_COUNT_MAP.at(partType) << ") for the part: '"
				<< id << "'" << std::endl;
		return boost::shared_ptr<PartRepresentation>();
	}

	// In order to save the compability with Mutator::mutateParams
	// we will keep only the params that this function needs in the vector Params
	if(PART_TYPE_IS_VARIABLE_ARITY_MAP.at(partType)){
		if(	partType == PART_TYPE_PARAM_PRISM_CORE ||
			partType == PART_TYPE_PARAM_PRISM_CORE_NO_IMU)
				arity =params.at(0);
		else
			arity = params.at(0)-1; // remove the Parent Connection
		params.erase(params.begin());
	}
	else
	{
		arity = PART_TYPE_ARITY_MAP.at(partType);
	}

	return boost::shared_ptr<PartRepresentation>(
			new PartRepresentation(id, orientation,
					arity, partType, params,
					PART_TYPE_MOTORS_MAP.at(partType),
					PART_TYPE_SENSORS_MAP.at(partType)));

}

void PartRepresentation::addSubtreeToBodyMessage(
		robogenMessage::Body *bodyMessage, bool amIRoot) {

	// first, insert self
	robogenMessage::BodyPart* serialization = bodyMessage->add_part(); 

	// required string id = 1;
	serialization->set_id(id_);

	// required string type = 2;
	serialization->set_type(this->getType());

	// required bool root = 3;
	serialization->set_root(amIRoot);

	// BASIL
	/*
	* TODO: if isTheCore put param->set_paramvalue(arity_)
	*/
	unsigned i0param = 0;
	if(PART_TYPE_IS_VARIABLE_ARITY_MAP.at(this->getType())){
		robogenMessage::EvolvableParameter *param =
					serialization->add_evolvableparam();
		if(	this->getType() == PART_TYPE_PARAM_PRISM_CORE ||
			this->getType() == PART_TYPE_PARAM_PRISM_CORE_NO_IMU)
			param->set_paramvalue(arity_);
		else
			param->set_paramvalue(arity_ + 1);
		i0param = 1;
	}
	// repeated EvolvableParameter evolvableParam = 4;
	// The first parameter can be a non-normalized Arity map
	for (unsigned int i = i0param; i < params_.size(); ++i) {
		robogenMessage::EvolvableParameter *param =
				serialization->add_evolvableparam();

		//convert parameters from [0,1] back to valid range
		std::pair<double, double> ranges = PART_TYPE_PARAM_RANGE_MAP.at(
				std::make_pair(this->getType(), i));
		double paramValue = (fabs(ranges.first - ranges.second) < 1e-6)
								? ranges.first
								: (params_[i] * (ranges.second - ranges.first))
								  + ranges.first;
		param->set_paramvalue(paramValue);
	}

	// required int32 orientation = 5;
	serialization->set_orientation(orientation_);

	// treat children, including connection
	for (unsigned int i = 0; i < arity_; i++) {
		if (this->getChild(i)) {
			robogenMessage::BodyConnection *connection =
					bodyMessage->add_connection();
			connection->set_src(id_);

			if (isCore(this->getType())) {
				connection->set_srcslot(i);
			} else {
				connection->set_srcslot(i+1);
			}
			connection->set_dest(this->getChild(i)->getId());
			connection->set_destslot(0);
			this->getChild(i)->addSubtreeToBodyMessage(bodyMessage, false);
		}
	}

}

std::vector<std::string> PartRepresentation::getAncestorsIds() {

	std::vector<std::string> ids;
	if (parent_) {
		ids.push_back(parent_->getId());
		std::vector<std::string> tmp = parent_->getAncestorsIds();
		ids.insert(ids.end(), tmp.begin(), tmp.end());
	}
	return ids;

}

std::vector<std::string> PartRepresentation::getDescendantsIds() {

	std::vector<std::string> ids;
	for (unsigned int i = 0; i < children_.size(); ++i) {
		// child and all its children
		if (children_[i].get()) {

			// Add children ID
			ids.push_back(children_[i]->getId());

			// Add all the descendants ids
			std::vector<std::string> tmp = children_[i]->getDescendantsIds();
			ids.insert(ids.end(), tmp.begin(), tmp.end());
		}
	}
	return ids;

}

boost::shared_ptr<PartRepresentation> PartRepresentation::cloneSubtree() {

	boost::shared_ptr<PartRepresentation> theClone(
			new PartRepresentation(this->getId(), this->getOrientation(),
					this->getArity(), this->getType(), this->getParams(),
					this->getMotors(), this->getSensors()));
	// deep copy all children
	for (unsigned int i = 0; i < this->getArity(); i++) {
		if (this->getChild(i)) {
			theClone->setChild(i, this->getChild(i)->cloneSubtree());
		}
	}
	return theClone;

}

std::vector<double> &PartRepresentation::getParams() {
	return params_;
}

void PartRepresentation::setParent(PartRepresentation* parent) {
	parent_ = parent;
}

PartRepresentation* PartRepresentation::getParent() {
	return parent_;
}

void PartRepresentation::setPosition(int position) {
	position_ = position;
}

int PartRepresentation::getPosition() {
	return position_;
}

std::vector<std::string> PartRepresentation::getMotors() {
	return motors_;
}

std::vector<std::string> PartRepresentation::getSensors() {
	return sensors_;
}

void PartRepresentation::toString(std::stringstream& str, unsigned int depth) {

	// Print out current childrens and recursively call on them
	for (unsigned int i = 0; i < arity_; ++i) {

		for (unsigned int j = 0; j < depth; ++j) {
			str << "\t";
		}

		if (this->getChild(i)) {
			str << " -> [" << this->getChild(i)->getId() << " | " << this->getChild(i)->getType() << "]" << std::endl;
			this->getChild(i)->toString(str, depth+1);
		} else {
			str << " -> NULL" << std::endl;
		}

	}

}

} /* namespace robogen */
