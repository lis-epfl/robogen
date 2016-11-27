/*
 * Grammar.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: lis
 */

#include <iostream>
#include <sstream>
#include <math.h>

#include "evolution/representation/Grammar.h"

using namespace robogen;

Grammar::Axiom::Axiom(boost::shared_ptr<PartRepresentation> r){
    this->tree_ = r;
}

boost::shared_ptr<PartRepresentation> Grammar::Axiom::getAxiomClone(){
	return this->tree_->cloneSubtree();
}

std::string Grammar::Axiom::generateUniqueIdFromSomeId() {
	std::stringstream ss;
	ss << "myid" << maxid_;

	std::string newUniqueId = ss.str();
	maxid_++;
	return newUniqueId;
}

bool Grammar::Axiom::insertPart(const std::string& parentPartId,
		unsigned int parentPartSlot,
		boost::shared_ptr<PartRepresentation> newPart,
		unsigned int newPartSlot,
		unsigned int motorNeuronType, bool printErrors) {

	// Set new ID for the inserted node
	std::string newUniqueId = this->generateUniqueIdFromSomeId();
	newPart->setId(newUniqueId);

	// create Neurons in NeuralNetwork
	std::vector<std::string> sensors = newPart->getSensors();
	for (unsigned int i = 0; i < sensors.size(); ++i) {
		neuralNetwork_->insertNeuron(ioPair(newPart->getId(), i),
				NeuronRepresentation::INPUT, NeuronRepresentation::SIMPLE);
	}
	std::vector<std::string> motors = newPart->getMotors();
	for (unsigned int i = 0; i < motors.size(); ++i) {
		neuralNetwork_->insertNeuron(
				ioPair(newPart->getId(), sensors.size() + i),
				NeuronRepresentation::OUTPUT, motorNeuronType);
	}

	// find dst part by id
	boost::shared_ptr<PartRepresentation> parentPart =
			idToPart_[parentPartId].lock();
	boost::shared_ptr<PartRepresentation> childPart = parentPart->getChild(
			parentPartSlot);

	// Check the arity of the new part
	if (childPart != NULL && newPart->getArity() < 1) {
		if (printErrors) {
			std::cerr << "Cannot insert 0-arity part when there is already a"
					" part present at its location" << std::endl;
		}
		return false;
	}

	parentPart->setChild(parentPartSlot, newPart);

	if (childPart != NULL)
		newPart->setChild(newPartSlot, childPart);

	// Add to the map
	idToPart_[newUniqueId] = boost::weak_ptr<PartRepresentation>(newPart);

	return true;

}

bool Grammar::Axiom::removePart(const std::string& partId,
		bool printErrors) {

	boost::shared_ptr<PartRepresentation> nodeToRemove =
			idToPart_[partId].lock();

	// If root node, return
	if (nodeToRemove->getId().compare(this->tree_->getId()) == 0) {
		if (printErrors) {
			std::cerr << "Cannot remove root" << std::endl;
		}
		return false;
	}

	// Get parent of node to be removed
	PartRepresentation* parent = nodeToRemove->getParent();

	// this should be handled by check above, but just to be safe
	if (parent == NULL) {
		if (printErrors) {
				std::cerr << "Cannot remove root" << std::endl;
			}
		return false;
	}

	// Add one since will be freeing a slot when this node is removed
	unsigned int nFreeSlots = parent->getFreeSlots().size() + 1;
	if (nFreeSlots < nodeToRemove->getChildrenCount()) {
		if (printErrors) {
			std::cerr << "Not enough free slots on parent to remove part."
					<< "Need " << nodeToRemove->numDescendants()
					<< ", but only have " << nFreeSlots
					<< std::endl;
		}
		return false;
	}

	// remove neurons and all their connections
	neuralNetwork_->removeNeurons(partId);

	// Find child part that will be removed
	for (unsigned int i = 0; i < parent->getArity(); i++) {
		if (parent->getChild(i) != NULL &&
				parent->getChild(i)->getId().compare(nodeToRemove->getId())
				== 0) {
			parent->setChild(i, boost::shared_ptr<PartRepresentation>());
			break;
		}
	}
	idToPart_.erase(nodeToRemove->getId());

	if ( nodeToRemove->getArity() > 0 ) {
		unsigned int indx = 0;
		for (unsigned int i = 0; i < parent->getArity(); i++) {

			if (parent->getChild(i) == NULL) {

				while (nodeToRemove->getChild(indx) == NULL) {
					indx++;
					if (indx >= nodeToRemove->getArity()) {
						return true;
					}
				}

				parent->setChild(i, nodeToRemove->getChild(indx));

				// Increment current index
				indx++;

				if (indx >= nodeToRemove->getArity()) {
					return true;
				}

			}
		}
	}
	return true;
}

bool Grammar::Axiom::check() {

	// 1. Check that every body part in the body tree is in the idBodyPart map
	// and there are no dangling references
	std::vector<std::string> bodyPartIdsFromMap;
	for (std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator
			it = idToPart_.begin(); it != idToPart_.end(); ++it) {
		bodyPartIdsFromMap.push_back(it->first);
	}

	std::vector<std::string> bodyIds = this->tree_->getDescendantsIds();
	bodyIds.push_back(this->tree_->getId());

	std::sort(bodyPartIdsFromMap.begin(), bodyPartIdsFromMap.end());
	std::sort(bodyIds.begin(), bodyIds.end());

	bool idsMatch = true;
	if (bodyPartIdsFromMap.size() != bodyIds.size()) {
		std::cout << "Error: bodyPartIdsFromMap has " <<
				bodyPartIdsFromMap.size() << " elements, but bodyIds has " <<
				bodyIds.size() << " elements\n";
		idsMatch = false;
	} else {
		for (unsigned int i = 0; i < bodyPartIdsFromMap.size(); ++i) {
			if (bodyPartIdsFromMap[i].compare(bodyIds[i]) != 0) {
				std::cerr << "Error: bodyPartIdsFromMap does not match bodyIds "
						<< "at "
						<< "position " << i << " " << bodyPartIdsFromMap[i]
						<< " " << bodyIds[i] << "\n";
				idsMatch = false;
			}
		}
	}

	if (!idsMatch) {
		std::cerr << "bodyPartIdsFromMap:";
		for (unsigned int i = 0; i < bodyPartIdsFromMap.size(); ++i) {
			std::cerr << " " << bodyPartIdsFromMap[i] ;
		}
		std::cerr << "\nbodyIds:";
		for (unsigned int i = 0; i < bodyIds.size(); ++i) {
			std::cerr << " " << bodyIds[i] ;
		}
		std::cerr << "\n";

		return false;
	}

	// 2. Check that each neuron has an associated body part
	std::map<std::string, int> netInputs;
	std::map<std::string, int> netOutputs;
	for (unsigned int i = 0; i < bodyIds.size(); ++i) {

		bool hasNeurons = true;
		boost::weak_ptr<PartRepresentation> part = idToPart_[bodyIds[i]];
		if (part.lock()->getSensors().size() > 0) {
			netInputs[bodyIds[i]] = part.lock()->getSensors().size();
		}

		if (part.lock()->getMotors().size() > 0) {
			netOutputs[bodyIds[i]] = part.lock()->getMotors().size();
		}

		if (hasNeurons) {

			std::vector<boost::weak_ptr<NeuronRepresentation> > neurons =
					neuralNetwork_->getBodyPartNeurons(bodyIds[i]);
			int totInputs = 0;
			int totOutputs = 0;
			for (unsigned int j = 0; j < neurons.size(); ++j) {
				unsigned int layer = neurons[j].lock()->getLayer();
				if (layer == NeuronRepresentation::INPUT) {
					totInputs++;
				} else if (layer == NeuronRepresentation::OUTPUT) {
					totOutputs++;
				}
			}

			if (totInputs != netInputs[bodyIds[i]] ||
					totOutputs != netOutputs[bodyIds[i]]) {
				return false;
			}
		}

	}

	// TODO Consistency check is not complete, neural representation is
	// only partially checked
	return true;

}

Grammar::Grammar(boost::shared_ptr<PartRepresentation> bodytree,
                boost::shared_ptr<NeuralNetworkRepresentation> neuralNetwork,
				int maxid){

    this->axiom_ = boost::shared_ptr<Axiom>(new Axiom(bodytree->cloneSubtree()));

	this->maxid_ = maxid;

	std::cout<< "Creating grammar" << std::endl;
	this->pBodyTree_=bodytree;
    this->pNeuralNetwork_.reset(new NeuralNetworkRepresentation(*(neuralNetwork.get())));

    //Hardcoded Rule:
    boost::shared_ptr<PartRepresentation> searchPattern = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_FIXED_BRICK),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());

    boost::shared_ptr<PartRepresentation> replacePattern = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_FIXED_BRICK),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());
    this->rules_.push_back( boost::shared_ptr<Rule>(new Rule(2, searchPattern, replacePattern)));
}

Grammar::Rule::Rule(int iterations, boost::shared_ptr<PartRepresentation> predecessor,
                        boost::shared_ptr<PartRepresentation> successor){
    this->iterations_ = iterations;
    this->predecessor_ = predecessor;
    this->successor_ = successor;
}

boost::shared_ptr<PartRepresentation> Grammar::buildTree(void){

	//this->pBodyTree_ = this->axiom_->getAxiomClone();

	pIdToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(this->pBodyTree_);
	while (!q.empty()) {
		boost::shared_ptr<PartRepresentation> cur = q.front();
		q.pop();
		//Using at instead of []
		pIdToPart_[cur->getId()] = boost::weak_ptr<PartRepresentation>(cur);
		for (unsigned int i = 0; i < cur->getArity(); ++i) {
			if (cur->getChild(i)) {
				q.push(cur->getChild(i));
			}
		}
	}

	typedef IdPartMap::iterator it_type;

	std::cout << "%%%%%%%%%%%%% MUTATING %%%%%%%%%%%%%%%" << std::endl;

	std::cout << "Starting with " << this->pBodyTree_->numDescendants() << std::endl;

	IdPartMap bot = pIdToPart_;

	std::cout << "Equivalent to " << bot.size() << std::endl;

	for(it_type iterator = bot.begin(); iterator != bot.end(); iterator++) {
		// iterator->first = key
		// iterator->second = value

		if(iterator->second.lock()->getType() == PART_TYPE_FIXED_BRICK){

			if(iterator->second.lock()->getChildrenCount()==0){

				char type = 'F';

				unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type));
				std::vector<double> parameters;
				for (unsigned int i = 0; i < nParams; ++i) {
					parameters.push_back(0.1f);
				}

				boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
					type,
					"",
					0, //orientation
					parameters);

				this->insertPart(iterator->first,
					0, //Parent slot
					newPart,
					0, //newPartSlot
					0 ? NeuronRepresentation::OSCILLATOR :
							NeuronRepresentation::SIGMOID,
							false);
			}

		}
	}

	std::cout << "Finishing with " << this->pBodyTree_->numDescendants() << std::endl;

	std::cout << "Equivalent to " << bot.size() << std::endl;

	// Generate a core component

	boost::shared_ptr<PartRepresentation> corePart = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_CORE_COMPONENT),
			PART_TYPE_CORE_COMPONENT, 0, std::vector<double>());
    return corePart;
}

std::string Grammar::generateUniqueIdFromSomeId() {
	std::stringstream ss;
	ss << "myid" << maxid_;

	std::string newUniqueId = ss.str();
	maxid_++;
	return newUniqueId;
}

bool Grammar::insertPart(const std::string& parentPartId,
		unsigned int parentPartSlot,
		boost::shared_ptr<PartRepresentation> newPart,
		unsigned int newPartSlot,
		unsigned int motorNeuronType, bool printErrors) {

	// Set new ID for the inserted node
	std::string newUniqueId = this->generateUniqueIdFromSomeId();
	newPart->setId(newUniqueId);

	// create Neurons in NeuralNetwork
	std::vector<std::string> sensors = newPart->getSensors();
	for (unsigned int i = 0; i < sensors.size(); ++i) {
		pNeuralNetwork_->insertNeuron(ioPair(newPart->getId(), i),
				NeuronRepresentation::INPUT, NeuronRepresentation::SIMPLE);
	}
	
	std::vector<std::string> motors = newPart->getMotors();
	for (unsigned int i = 0; i < motors.size(); ++i) {
		pNeuralNetwork_->insertNeuron(
				ioPair(newPart->getId(), sensors.size() + i),
				NeuronRepresentation::OUTPUT, motorNeuronType);
	}

	// find dst part by id
	boost::shared_ptr<PartRepresentation> parentPart =
			pIdToPart_[parentPartId].lock();
	boost::shared_ptr<PartRepresentation> childPart = parentPart->getChild(
			parentPartSlot);

	// Check the arity of the new part
	if (childPart != NULL && newPart->getArity() < 1) {
		if (printErrors) {
			std::cerr << "Cannot insert 0-arity part when there is already a"
					" part present at its location" << std::endl;
		}
		return false;
	}

	parentPart->setChild(parentPartSlot, newPart);

	if (childPart != NULL)
		newPart->setChild(newPartSlot, childPart);

	// Add to the map
	std::cout << "New piece with id " << newUniqueId << std::endl;
	pIdToPart_[newUniqueId] = boost::weak_ptr<PartRepresentation>(newPart);

	return true;

}

bool Grammar::check() {

	// 1. Check that every body part in the body tree is in the idBodyPart map
	// and there are no dangling references
	std::vector<std::string> bodyPartIdsFromMap;
	for (std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator
			it = pIdToPart_.begin(); it != pIdToPart_.end(); ++it) {
		bodyPartIdsFromMap.push_back(it->first);
	}

	std::vector<std::string> bodyIds = this->pBodyTree_->getDescendantsIds();
	bodyIds.push_back(this->pBodyTree_->getId());

	std::sort(bodyPartIdsFromMap.begin(), bodyPartIdsFromMap.end());
	std::sort(bodyIds.begin(), bodyIds.end());

	bool idsMatch = true;
	if (bodyPartIdsFromMap.size() != bodyIds.size()) {
		std::cout << "Error: bodyPartIdsFromMap has " <<
				bodyPartIdsFromMap.size() << " elements, but bodyIds has " <<
				bodyIds.size() << " elements\n";
		idsMatch = false;
	} else {
		for (unsigned int i = 0; i < bodyPartIdsFromMap.size(); ++i) {
			if (bodyPartIdsFromMap[i].compare(bodyIds[i]) != 0) {
				std::cerr << "Error: bodyPartIdsFromMap does not match bodyIds "
						<< "at "
						<< "position " << i << " " << bodyPartIdsFromMap[i]
						<< " " << bodyIds[i] << "\n";
				idsMatch = false;
			}
		}
	}

	if (!idsMatch) {
		std::cerr << "bodyPartIdsFromMap:";
		for (unsigned int i = 0; i < bodyPartIdsFromMap.size(); ++i) {
			std::cerr << " " << bodyPartIdsFromMap[i] ;
		}
		std::cerr << "\nbodyIds:";
		for (unsigned int i = 0; i < bodyIds.size(); ++i) {
			std::cerr << " " << bodyIds[i] ;
		}
		std::cerr << "\n";

		return false;
	}

	// 2. Check that each neuron has an associated body part
	std::map<std::string, int> netInputs;
	std::map<std::string, int> netOutputs;
	for (unsigned int i = 0; i < bodyIds.size(); ++i) {

		bool hasNeurons = true;
		boost::weak_ptr<PartRepresentation> part = pIdToPart_[bodyIds[i]];
		if (part.lock()->getSensors().size() > 0) {
			netInputs[bodyIds[i]] = part.lock()->getSensors().size();
		}

		if (part.lock()->getMotors().size() > 0) {
			netOutputs[bodyIds[i]] = part.lock()->getMotors().size();
		}

		if (hasNeurons) {

			std::vector<boost::weak_ptr<NeuronRepresentation> > neurons =
					pNeuralNetwork_->getBodyPartNeurons(bodyIds[i]);
			int totInputs = 0;
			int totOutputs = 0;
			for (unsigned int j = 0; j < neurons.size(); ++j) {
				unsigned int layer = neurons[j].lock()->getLayer();
				if (layer == NeuronRepresentation::INPUT) {
					totInputs++;
				} else if (layer == NeuronRepresentation::OUTPUT) {
					totOutputs++;
				}
			}

			if (totInputs != netInputs[bodyIds[i]] ||
					totOutputs != netOutputs[bodyIds[i]]) {
				return false;
			}
		}

	}

	// TODO Consistency check is not complete, neural representation is
	// only partially checked
	return true;

}