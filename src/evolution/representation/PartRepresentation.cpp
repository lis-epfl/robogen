/*
 * PartRepresentation.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#include <iostream>
#include <sstream>

#include "evolution/representation/PartRepresentation.h"
#include "PartList.h"

namespace robogen {

PartRepresentation::PartRepresentation(std::string id, unsigned int orientation,
		unsigned int numSlots, const std::string& type, const std::vector<double>& params,
		const std::vector<std::string>& motors,
		const std::vector<std::string>& sensors) :
		id_(id), orientation_(orientation), numSlots_(numSlots), type_(type), parent_(NULL), params_(
				params), motors_(motors), sensors_(sensors) {

	slots_.resize(numSlots_, boost::weak_ptr<PartRepresentation>());
}

PartRepresentation::~PartRepresentation() {

}

std::string &PartRepresentation::getId() {
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

unsigned int PartRepresentation::getNumSlots() {
	return numSlots_;
}

unsigned int PartRepresentation::numDescendants() {

	int descendants = 0;
	for (unsigned int i = 0; i < slots_.size(); ++i) {
		// child and all its children
		if (slots_[i].lock() != NULL && slots_[i] != parent_) {
			descendants += (1 + slots_[i].lock()->numDescendants());
		}
	}
	return descendants;

}

std::vector<boost::weak_ptr<PartRepresentation> > PartRepresentation::
		getChildren() {
	std::vector<boost::weak_ptr<PartRepresentation> > children;
	for (unsigned int i = 0; i < slots_.size(); ++i) {
		// child and all its children
		if (slots_[i].lock() != NULL && slots_[i] != parent_) {
			children.push_back(slots_[i]);
		}
	}
	return children;
}

unsigned int PartRepresentation::getChildrenCount() {

	return this->getChildren().size();

}

std::vector<unsigned int> PartRepresentation::getFreeSlots() {

	std::vector<unsigned int> freeSlots;
	for (unsigned int i = 0; i < slots_.size(); ++i) {
		if (slots_[i].lock() == NULL) {
			freeSlots.push_back(i);
		}
	}
	return freeSlots;

}

std::string &PartRepresentation::getType() {
	return type_;
}

bool PartRepresentation::setSlot(unsigned int n,
		boost::shared_ptr<PartRepresentation> part) {

	if (n  >= numSlots_ ) {
		std::cout << "Attempt to access non-existing slot " << n << " of part "
				<< this->getId() << " with " << numSlots_ << " slots"
				<< std::endl;
		return false;
	}
	// don't try to access part if void
	if (part) {
		part->setParent(this);
		part->setPosition(n);
	}
	slots_[n] = part;
	return true;

}

boost::shared_ptr<PartRepresentation> PartRepresentation::create(char type,
		std::string id, int orientation, std::vector<double> params) {

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

	return boost::shared_ptr<PartRepresentation>(
			new PartRepresentation(id, orientation,
					PART_TYPE_ARITY_MAP.at(partType), partType, params,
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

	// repeated EvolvableParameter evolvableParam = 4;
	for (unsigned int i = 0; i < params_.size(); ++i) {
		robogenMessage::EvolvableParameter *param =
				serialization->add_evolvableparam();

		//convert parameters from [0,1] back to valid range
		std::pair<double, double> ranges = PART_TYPE_PARAM_RANGE_MAP.at(
				std::make_pair(this->getType(), i));
		double paramValue = (params_[i] * (ranges.second - ranges.first)) +
				ranges.first;
		param->set_paramvalue(paramValue);
	}

	// required int32 orientation = 5;
	serialization->set_orientation(orientation_);

	// treat children, including connection
	std::vector<boost::weak_ptr<PartRepresentation> >children =
			this->getChildren();
	for (unsigned int i = 0; i < children.size(); i++) {
		if (children[i].lock() != NULL) {
			robogenMessage::BodyConnection *connection =
					bodyMessage->add_connection();
			connection->set_src(id_);
			connection->set_srcslot(i);
			connection->set_dest(children[i].lock()->getId());
			connection->set_destslot(0);
			children[i].lock()->addSubtreeToBodyMessage(bodyMessage, false);
		}
	}

}

std::vector<std::string> PartRepresentation::getAncestorsIds() {

	std::vector<std::string> ids;
	if (parent_) {
		ids.push_back(parent_.lock()->getId());
		std::vector<std::string> tmp = parent_.lock()->getAncestorsIds();
		ids.insert(ids.end(), tmp.begin(), tmp.end());
	}
	return ids;

}

std::vector<std::string> PartRepresentation::getDescendantsIds() {

	std::vector<std::string> ids;
	std::vector<boost::weak_ptr<PartRepresentation> > children =
				this->getChildren();
	for (unsigned int i = 0; i < children.size(); ++i) {
		// child and all its children
		if (children[i].lock() != NULL) {

			// Add children ID
			ids.push_back(children[i].lock()->getId());

			// Add all the descendants ids
			std::vector<std::string> tmp = children[i].lock()
					->getDescendantsIds();
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
	std::vector<boost::weak_ptr<PartRepresentation> > children =
			this->getChildren();
	for (unsigned int i = 0; i < children.size(); i++) {
		if (children[i].lock() != NULL) {
			theClone->setChild(i,children[i].lock()->cloneSubtree());
		}
	}
	return theClone;

}

std::vector<double> PartRepresentation::getParams() {
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

} /* namespace robogen */
