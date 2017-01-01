#include"SubRobotRepresentation.h"

#include <stack>
#include <fstream>
#include <sstream>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

namespace robogen{

SubRobotRepresentation::SubRobotRepresentation() :
		maxid_(1000) {
}

SubRobotRepresentation::SubRobotRepresentation(const SubRobotRepresentation &r){
	// we need to handle bodyTree_, neuralNetwork_ and reservedIds_
	// for the brainevolver, we could theoretically keep the bodyTree_ pointing
	// to the same body, but that would be easy to miss when resuming to body
	// evolution, so we'll just do proper copying right away

	// special treatment for base-pointed instances of derived classes as are
	// our body parts
	this->bodyTree_ = r.bodyTree_->cloneSubtree();

	// neural network pointer needs to be reset to a copy-constructed instance
	neuralNetwork_.reset(
			new NeuralNetworkRepresentation(*(r.neuralNetwork_.get())));
	// rebuild ID to part map
	idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(bodyTree_);
	while (!q.empty()) {
		boost::shared_ptr<PartRepresentation> cur = q.front();
		q.pop();
		idToPart_[cur->getId()] = boost::weak_ptr<PartRepresentation>(cur);
		for (unsigned int i = 0; i < cur->getArity(); ++i) {
			if (cur->getChild(i)) {
				q.push(cur->getChild(i));
			}
		}
	}
	maxid_ = r.maxid_;
}

const SubRobotRepresentation::IdPartMap& SubRobotRepresentation::getBody() const {
	return idToPart_;
}

void SubRobotRepresentation::getBrainGenome(std::vector<double*> &weights,
		std::vector<unsigned int> &types,
		std::vector<double*> &params) {
	neuralNetwork_->getGenome(weights, types, params);
}

boost::shared_ptr<NeuralNetworkRepresentation> SubRobotRepresentation::getBrain(
		) const {
	return this->neuralNetwork_;
}

boost::shared_ptr<PartRepresentation> SubRobotRepresentation::getTree(
		) const {
	return this->bodyTree_;
}

int SubRobotRepresentation::getMaxId(){
	return this->maxid_;
}

robogenMessage::Robot SubRobotRepresentation::serialize() const {
	robogenMessage::Robot message;
	// id - this can probably be removed
	message.set_id(1);
	// body
	this->bodyTree_->addSubtreeToBodyMessage(message.mutable_body(), true);
	// brain
	*(message.mutable_brain()) = this->neuralNetwork_->serialize();
	return message;
}

void SubRobotRepresentation::rebuildBodyMap(){
	// rebuild ID to part map
	this->idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(this->bodyTree_);
	while (!q.empty()) {
		boost::shared_ptr<PartRepresentation> cur = q.front();
		q.pop();
		//Using at instead of []
		this->idToPart_[cur->getId()] = boost::weak_ptr<PartRepresentation>(cur);
		for (unsigned int i = 0; i < cur->getArity(); ++i) {
			if (cur->getChild(i)) {
				q.push(cur->getChild(i));
			}
		}
	}
}

SubRobotRepresentation &SubRobotRepresentation::operator=(
		const SubRobotRepresentation &r) {
	// same as copy constructor, see there for explanations
	this->bodyTree_ = r.bodyTree_->cloneSubtree();
	this->neuralNetwork_.reset(
			new NeuralNetworkRepresentation(*(r.neuralNetwork_.get())));
	// rebuild ID to part map
	this->idToPart_.clear();
	std::queue<boost::shared_ptr<PartRepresentation> > q;
	q.push(bodyTree_);
	while (!q.empty()) {
		boost::shared_ptr<PartRepresentation> cur = q.front();
		q.pop();
		this->idToPart_[cur->getId()] = boost::weak_ptr<PartRepresentation>(cur);
		for (unsigned int i = 0; i < cur->getArity(); ++i) {
			if (cur->getChild(i)) {
				q.push(cur->getChild(i));
			}
		}
	}
	maxid_ = r.maxid_;
	return *this;
}

bool SubRobotRepresentation::init() {

// Generate a core component
std::vector<double> params;
//by Default we have a prism with 4 faces because we can't have the seed so
//we can't create a random value
params.push_back(4);
	boost::shared_ptr<PartRepresentation> corePart = PartRepresentation::create(
			INVERSE_PART_TYPE_MAP.at(PART_TYPE_PARAM_PRISM_CORE),
			PART_TYPE_PARAM_PRISM_CORE, 0, params);
	if (!corePart) {
		std::cout << "Failed to create root node" << std::endl;
		return false;
	}
	bodyTree_ = corePart;
	idToPart_[PART_TYPE_PARAM_PRISM_CORE] = boost::weak_ptr<PartRepresentation>(
			corePart);
	// TODO abstract this to a different function so it doesn't
	// duplicate what we have below

	// process brain
	std::string from, to;
	// create neural network: create map from body id to ioId for all sensor and
	// motor body parts
	std::map<std::string, int> sensorMap, motorMap;
	for (std::map<std::string,
			boost::weak_ptr<PartRepresentation> >::iterator it =
			idToPart_.begin(); it != idToPart_.end(); it++) {

		// omitting weak pointer checks, as this really shouldn't go wrong here!
		if (it->second.lock()->getMotors().size()) {
			motorMap[it->first] = it->second.lock()->getMotors().size();
		}
		if (it->second.lock()->getSensors().size()) {
			sensorMap[it->first] = it->second.lock()->getSensors().size();
		}

	}

	neuralNetwork_.reset(new NeuralNetworkRepresentation(sensorMap, motorMap));

	return true;
}


bool SubRobotRepresentation::init(std::string robotTextFile) {

	// open file
	std::ifstream file;
	file.open(robotTextFile.c_str());
	if (!file.is_open()) {
		std::cout << "Could not open robot text file " << robotTextFile
				<< std::endl;
		return false;
	}

	// prepare body processing
	boost::shared_ptr<PartRepresentation> current;
	std::stack<boost::shared_ptr<PartRepresentation> > parentStack;
	unsigned int slot, orientation, indent;
	char type;
	std::string line, id;
	std::vector<double> params;

	// process root node

	if (!robotTextFileReadPartLine(file, indent, slot, type, id, orientation,
			params) || indent) {
		std::cout << "Robot text file contains no or"
				" poorly formatted root node" << std::endl;
		return false;
	}

	current = PartRepresentation::create(type, id, orientation, params);
	if (!current) {
		std::cout << "Failed to create root node" << std::endl;
		return false;
	}
	bodyTree_ = current;
	idToPart_[id] = boost::weak_ptr<PartRepresentation>(current);

	// process other body parts

	while (robotTextFileReadPartLine(file, indent, slot, type, id,
			orientation, params)) {
		if (!indent) {
			std::cout << "Attempt to create multiple root nodes!"
					<< std::endl;
			return false;
		}
		// indentation: Adding children to current
		if (indent > (parentStack.size())) {
			parentStack.push(current);
		}
		// indentation: done adding children to top of parent stack
		for (; indent < (parentStack.size());) {
			parentStack.pop();
		}
		current = PartRepresentation::create(type, id, orientation, params);
		if (!current) {
			std::cout << "Failed to create node." << std::endl;
			return false;
		}
		if (parentStack.top()->getChild(slot)) {
			std::cout << "Attempt to overwrite child "
					<< parentStack.top()->getChild(slot)->getId() << " of "
					<< parentStack.top()->getId() << " with "
					<< current->getId() << std::endl;
			return false;
		}
		if (!parentStack.top()->setChild(slot, current)) {
			std::cout << "Failed to set child." << std::endl;
			return false;
		}
		idToPart_[id] = boost::weak_ptr<PartRepresentation>(current);
	}

	// process brain
	std::string from, to;
	int fromIoId, toIoId;
	double value;
	// create neural network: create map from body id to ioId for all sensor and
	// motor body parts
	std::map<std::string, int> sensorMap, motorMap;
	for (std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator
			it = idToPart_.begin(); it != idToPart_.end(); it++) {

		// omitting weak pointer checks, as this really shouldn't go wrong here!
		if (it->second.lock()->getMotors().size()) {
			motorMap[it->first] = it->second.lock()->getMotors().size();
		}
		if (it->second.lock()->getSensors().size()) {
			sensorMap[it->first] = it->second.lock()->getSensors().size();
		}

	}

	neuralNetwork_.reset(new NeuralNetworkRepresentation(sensorMap, motorMap));
	unsigned int neuronType;
	// add new neurons

	while (robotTextFileReadAddNeuronLine(file, id, neuronType)) {
		if (neuralNetwork_->getNumHidden() < MAX_HIDDEN_NEURONS) {
			std::string neuronId = neuralNetwork_->insertNeuron(ioPair(id,
					neuralNetwork_->getBodyPartNeurons(id).size()),
					NeuronRepresentation::HIDDEN, neuronType);
			std::cout << "added hidden neuron "  << neuronId << " with type "
					<< neuronType << std::endl;
		} else {
			std::cerr << "The number of specified hidden neurons is more than "
					<< MAX_HIDDEN_NEURONS << ", which is the maximuma allowed."
					<< std::endl;
			return false;
		}
	}

	// weights
	while (robotTextFileReadWeightLine(file, from, fromIoId, to, toIoId,
			value)) {
		if (!neuralNetwork_->setWeight(from, fromIoId, to, toIoId, value)) {
			std::cerr << "Failed to set weight" << std::endl;
			return false;
		}
	}

	// params
	params.clear();

	while (robotTextFileReadParamsLine(file, to, toIoId, neuronType, params)) {
		if (!neuralNetwork_->setParams(to, toIoId, neuronType, params)) {
			std::cerr << "Failed to set neuron params" << std::endl;
			return false;
		}
		params.clear();
	}


	while(!file.eof()) {
		RobogenUtils::safeGetline(file, line);
		if(!isLineEmpty(line)) {
			std::cerr << std::endl << std::endl
					<< "The robot text file has non-empty lines after all "
					<< "body and brain lines have been parsed!" << std::endl
					<< "Are you sure the file is properly formatted??"
					<< std::endl << std::endl;
			exitRobogen(EXIT_FAILURE);
		}
	}


	file.close();

	maxid_ = 1000;

	// loop through existing ids to find what new maxid should be.
	// this is necessary when trying to seed evolution with a previously
	// evolved morphology
	for(IdPartMap::iterator i = idToPart_.begin(); i!= idToPart_.end(); ++i) {
		if(i->first.substr(0,4).compare("myid") == 0) {
			int idVal = atoi(i->first.substr(4).c_str());
			if (idVal >= maxid_) {
				maxid_ = idVal + 1;
			}
		}
	}

}


bool SubRobotRepresentation::trimBodyAt(const std::string& id, bool printErrors) {
	// kill all neurons and their weights
	recurseNeuronRemoval(idToPart_[id].lock());

	// thanks to shared pointer magic, we only need to reset the shared pointer
	// to the indicated body part
	PartRepresentation *parent = idToPart_[id].lock()->getParent();
	int position = idToPart_[id].lock()->getPosition();
	if (!parent) {
		if (printErrors) {
			std::cerr << "Trying to remove root body part!" << std::endl;
		}
		return false;
	}
	//std::cout << "Has references: " << idToPart_[id].lock().use_count()
	//		<< std::endl;
	if (!parent->setChild(position, boost::shared_ptr<PartRepresentation>())) {
		if (printErrors) {
			std::cerr << "Failed trimming robot body!" << std::endl;
		}
		return false;
	}
	if (!parent->getChild(position)) {
		//std::cout << "Successfully removed" << std::endl;
	}
	// need to update the id to body part map! Easily done with weak pointers
	for (IdPartMap::iterator it = idToPart_.begin(); it != idToPart_.end();) {
		if (!it->second.lock()) {
			idToPart_.erase(it++);
			//std::cout << "Had a part to erase! " << parent << std::endl;
		} else {
			++it;
		}
	}
	return true;

}

std::string SubRobotRepresentation::generateUniqueIdFromSomeId() {

	std::stringstream ss;
	ss << "myid" << maxid_;

	std::string newUniqueId = ss.str();
	maxid_++;
	return newUniqueId;

}

void SubRobotRepresentation::recurseNeuronRemoval(
		boost::shared_ptr<PartRepresentation> part) {
	neuralNetwork_->removeNeurons(part->getId());
	for (unsigned int i = 0; i < part->getArity(); i++) {
		if (part->getChild(i)) {
			this->recurseNeuronRemoval(part->getChild(i));
		}
	}
}

bool SubRobotRepresentation::addClonesToMap(
		boost::shared_ptr<PartRepresentation> part,
		std::map<std::string, std::string> &neuronReMapping) {
	std::string oldId = part->getId();
	std::string newUniqueId = this->generateUniqueIdFromSomeId();
	part->setId(newUniqueId);
	// insert part in map
	idToPart_[newUniqueId] = boost::weak_ptr<PartRepresentation>(part);
	// clone neurons, save mapping
	neuralNetwork_->cloneNeurons(oldId, part->getId(), neuronReMapping);

	for (unsigned int i = 0; i < part->getArity(); i++) {

		if (part->getChild(i)) {
			this->addClonesToMap(part->getChild(i), neuronReMapping);
		}

	}

	return true;

}

bool SubRobotRepresentation::duplicateSubTree(const std::string& subtreeRootPartId,
		const std::string& subtreeDestPartId, unsigned int slotId,
		bool printErrors) {

	// find src part and dest part by id
	boost::shared_ptr<PartRepresentation> src =
			idToPart_[subtreeRootPartId].lock();
	boost::shared_ptr<PartRepresentation> dst =
			idToPart_[subtreeDestPartId].lock();

	// If source is root node, then return
	if (src->getId().compare(bodyTree_->getId()) == 0) {
		if (printErrors) {
			std::cerr << "Cannot duplicate root node!" << std::endl;
		}
		return false;
	}

	boost::shared_ptr<PartRepresentation> clone = src->cloneSubtree();
	dst->setChild(slotId, clone);

	std::map<std::string, std::string> neuronReMapping;

	// insert clones into part map and generate neurons
	this->addClonesToMap(clone, neuronReMapping);

	// correctly generate the weights
	neuralNetwork_->generateCloneWeights(neuronReMapping);

	return true;

}

bool SubRobotRepresentation::swapSubTrees(const std::string& subtreeRoot1,
		const std::string& subtreeRoot2, bool printErrors) {

	// Get roots of the subtrees
	boost::shared_ptr<PartRepresentation> root1 =
			idToPart_[subtreeRoot1].lock();
	boost::shared_ptr<PartRepresentation> root2 =
			idToPart_[subtreeRoot2].lock();

	// Check none of them is the root node
	if (root1->getId().compare(bodyTree_->getId()) == 0
			|| root2->getId().compare(bodyTree_->getId()) == 0) {
		if (printErrors) {
			std::cerr << "Cannot swap root subtree" << std::endl;
		}
		return false;
	}

	// Get parents and slots of each subtree
	PartRepresentation* parentRoot1 = root1->getParent();
	PartRepresentation* parentRoot2 = root2->getParent();

	// Get the slots to which this nodes are connected
	unsigned int slotParentRoot1 = 1000000;

	for (unsigned int i = 0; i < parentRoot1->getArity(); ++i) {
		if (parentRoot1->getChild(i) != NULL) {
			if (parentRoot1->getChild(i)->getId().compare(root1->getId())
					== 0) {
				slotParentRoot1 = i;
				break;
			}
		}
	}

	unsigned int slotParentRoot2 = 0;
	for (unsigned int i = 0; i < parentRoot2->getArity(); ++i) {
		if (parentRoot2->getChild(i) != NULL) {
			if (parentRoot2->getChild(i)->getId().compare(root2->getId())
					== 0) {
				slotParentRoot2 = i;
				break;
			}
		}
	}

	// Swap the subtrees
	parentRoot2->setChild(slotParentRoot2, root1);
	parentRoot1->setChild(slotParentRoot1, root2);

	return true;

}

bool SubRobotRepresentation::insertPart(const std::string& parentPartId,
		unsigned int parentPartSlot,
		boost::shared_ptr<PartRepresentation> newPart,
		unsigned int newPartSlot,
		unsigned int motorNeuronType, bool printErrors) {

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

	parentPart->setChild(parentPartSlot, newPart);

	if (childPart != NULL)
		newPart->setChild(newPartSlot, childPart);

	// Add to the map
	idToPart_[newUniqueId] = boost::weak_ptr<PartRepresentation>(newPart);

	return true;

}

bool SubRobotRepresentation::removePart(const std::string& partId,
		bool printErrors) {

	if(idToPart_.find(partId)==idToPart_.end()){
		std::cout << "Trying to access non existing node" << std::endl;
	}

	boost::shared_ptr<PartRepresentation> nodeToRemove =
			idToPart_[partId].lock();

	// If root node, return
	if (nodeToRemove->getId().compare(bodyTree_->getId()) == 0) {
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

bool SubRobotRepresentation::setChildPosition(const std::string& partId,  
			std::vector<boost::shared_ptr<PartRepresentation>> children,
			bool printErrors){

	boost::shared_ptr<PartRepresentation> part = idToPart_[partId].lock();

//check the coherency of the vector children
	if(children.size() != part->getArity()){
		if(printErrors){
		std::cout	<< "RobotRepresentation::setChildPosition: "
					<< "children vector size is "
					<< children.size()
					<< " and must be the same of the arity, so "
					<< part->getArity()
					<< std::endl;
		}
		return false;
	}

//Set the children
	part -> setChildren(children);
	return true;
}

std::string SubRobotRepresentation::toString() {
	std::stringstream str;
	str << "[" << this->bodyTree_->getId() << " | " << this->bodyTree_->getType() << "]"
			<< std::endl;
	this->bodyTree_->toString(str, 0);
	str << "Network:" << std::endl;
	str << this->neuralNetwork_->toString();
	return str.str();
}

bool SubRobotRepresentation::check() {

	// 1. Check that every body part in the body tree is in the idBodyPart map
	// and there are no dangling references
	std::vector<std::string> bodyPartIdsFromMap;
	for (std::map<std::string, boost::weak_ptr<PartRepresentation> >::iterator
			it = idToPart_.begin(); it != idToPart_.end(); ++it) {
		bodyPartIdsFromMap.push_back(it->first);
	}

	std::vector<std::string> bodyIds = bodyTree_->getDescendantsIds();
	bodyIds.push_back(bodyTree_->getId());

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

boost::shared_ptr<PartRepresentation> SubRobotRepresentation::getNodeById(std::string id){
	return this->idToPart_[id].lock();
}

}