#include "IndirectMutator.h"

namespace robogen{

//helper function for mutations
//TODO move this somewhere else
double clip(double value, double min, double max) {
	if ( value < min )
		return min;
	if (value > max )
		return max;
	return value;
}

typedef bool (IndirectMutator::*IndMutationOperator)(boost::shared_ptr<RobotRepresentation>&);

typedef std::pair<IndMutationOperator, boost::random::bernoulli_distribution<double> > IndMutOpPair;

IndirectMutator::~IndirectMutator() {
}

IndirectMutator::IndirectMutator(boost::shared_ptr<EvolverConfiguration> conf,
		boost::random::mt19937 &rng) :
		conf_(conf), rng_(rng), brainMutate_(conf->pBrainMutate){

	addHiddenNeuronDist_ = boost::random::bernoulli_distribution<double>(
			conf->pAddHiddenNeuron);

	oscillatorNeuronDist_ = boost::random::bernoulli_distribution<double>(
			conf->pOscillatorNeuron);


	if (conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		suppressRuleDist_ =
				boost::random::bernoulli_distribution<double>(conf->pSuppressRule);
		createRuleDist_ =
				boost::random::bernoulli_distribution<double>(conf->pCreateRule);
		swapRulesDist_ =
				boost::random::bernoulli_distribution<double>(conf->pSwapRules);
		mutateRuleDist_ =
				boost::random::bernoulli_distribution<double>(conf->pMutateRule);
		mutateAxiomDist_ =
				boost::random::bernoulli_distribution<double>(conf->pMutateAxiom);
	}
}

void IndirectMutator::growBodyRandomly(boost::shared_ptr<RobotRepresentation>& robot) {
	boost::random::uniform_int_distribution<> dist(0,8);
	unsigned int numPartsToAdd = dist(rng_);

	for (unsigned int i = 0; i < numPartsToAdd; i++) {
		bool success = false;

		for (unsigned int attempt = 0;
				(attempt < conf_->maxBodyMutationAttempts); ++attempt) {

			boost::shared_ptr<RobotRepresentation> newBot = boost::shared_ptr<
					RobotRepresentation>(new RobotRepresentation(*robot.get()));

			boost::shared_ptr<SubRobotRepresentation> axiom = newBot->getGrammar()->getAxiom();
			success = this->insertNode(axiom, true);
			int errorCode;

			newBot->buildFromGrammar();

			std::vector<std::pair<std::string, std::string> > affectedBodyParts;
			if (success
					&& BodyVerifier::verify(*newBot.get(), errorCode,
							affectedBodyParts, PRINT_ERRORS)) {
				robot = newBot;
				robot->setDirty();
				break;
			}
		}
	}
}

bool IndirectMutator::insertNode(boost::shared_ptr<RobotRepresentation>& robot) {

	boost::shared_ptr<SubRobotRepresentation> axiom = robot->getGrammar()->getAxiom();

	const SubRobotRepresentation::IdPartMap& idPartMap = axiom->getBody();

	if ( idPartMap.size() >= conf_->maxBodyParts )
		return false;

	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);

	SubRobotRepresentation::IdPartMap::const_iterator parent;
	boost::shared_ptr<PartRepresentation> parentPart;

	// find a parent with arity > 0 (will exist, since at the very least will
	// be the core)

	do {
		parent = idPartMap.begin();
		std::advance(parent, dist(rng_));
		parentPart = parent->second.lock();
	} while (parentPart->getArity() == 0);

	// Sample a random slot
	boost::random::uniform_int_distribution<> slotDist(0,
												parentPart->getArity() - 1);
	unsigned int parentSlot = slotDist(rng_);


	// Select node type
	boost::random::uniform_int_distribution<> distType(0,
			conf_->allowedBodyPartTypes.size() - 1);
	char type = conf_->allowedBodyPartTypes[distType(rng_)];

	// Randomly generate node orientation
	boost::random::uniform_int_distribution<> orientationDist(0, 3);
	unsigned int curOrientation = orientationDist(rng_);

	// Randomly generate parameters
	unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type));
	std::vector<double> parameters;
	boost::random::uniform_01<double> paramDist;
	unsigned int i0Param = 0;
	//generate a random arity for the parts with variable arity
	if(PART_TYPE_IS_VARIABLE_ARITY_MAP.at(PART_TYPE_MAP.at(type))){
		std::pair<double, double> range = PART_TYPE_PARAM_RANGE_MAP.at(
					std::make_pair(PART_TYPE_MAP.at(type), 0));
		boost::random::uniform_int_distribution<> arityDist((int)range.first,(int)range.second);

		parameters.push_back(arityDist(rng_));
		i0Param = 1;
	}
	//generate random parameters that can be mutate with mutator::mutateParam
	for (unsigned int i = i0Param; i < nParams; ++i) {
		parameters.push_back(paramDist(rng_));
	}

	// Create the new part
	boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
			type, "", curOrientation, parameters);

	unsigned int newPartSlot = 0;
	if (newPart->getArity() > 0) {
		// Generate a random slot in the new node, if it has arity > 0
		// newPartSlot will have the number of the child like the .txt
		boost::random::uniform_int_distribution<> distNewPartSlot(0,
				newPart->getArity() - 1);
		newPartSlot = distNewPartSlot(rng_);
	}
	// otherwise just keep it at 0... inserting part will fail if arity is 0 and
	// there were previously parts attached to the parent's chosen slot

	return axiom->insertPart(parent->first, parentSlot, newPart, newPartSlot,
			oscillatorNeuronDist_(rng_) ? NeuronRepresentation::OSCILLATOR :
					NeuronRepresentation::SIGMOID, PRINT_ERRORS);
			//todo other neuron types?

}

bool IndirectMutator::removeNode(boost::shared_ptr<RobotRepresentation>& robot) {

	boost::shared_ptr<SubRobotRepresentation> axiom = robot->getGrammar()->getAxiom();


	// Select node for removal
	const SubRobotRepresentation::IdPartMap& idPartMap = axiom->getBody();
	boost::random::uniform_int_distribution<> dist(0, idPartMap.size() - 1);
	SubRobotRepresentation::IdPartMap::const_iterator partToRemove =
			idPartMap.begin();
	std::advance(partToRemove, dist(rng_));

	bool success= axiom->removePart(partToRemove->first, PRINT_ERRORS);
#ifdef DEBUG_MUTATE
	std::cout << "Removed " << partToRemove->first << " " << success << std::endl;
#endif
	return success;

}

bool IndirectMutator::insertNode(boost::shared_ptr<SubRobotRepresentation>& robot, bool isAxiom){
	const SubRobotRepresentation::IdPartMap& idPartMap = robot->getBody();

	if ( idPartMap.size() >= conf_->maxBodyParts )
		return false;

	int offset = 0;

	if(idPartMap.size() >1 && !isAxiom){
		offset=1;
	}

	boost::random::uniform_int_distribution<> dist(offset, idPartMap.size() - 1);

	SubRobotRepresentation::IdPartMap::const_iterator parent;
	boost::shared_ptr<PartRepresentation> parentPart;

	// find a parent with arity > 0 , give up after 100 attempts
	int attempt = 0;
	do {
		parent = robot->getBody().begin();
		std::advance(parent, dist(rng_));
		parentPart = parent->second.lock();
		attempt++;
	} while (parentPart->getArity() == 0 && attempt<100);

	if(attempt==100){
		//This means we can't really grow the predecessor
		return false;
	}

	// Sample a random slot
	boost::random::uniform_int_distribution<> slotDist(0,
												parentPart->getArity() - 1);
	unsigned int parentSlot = slotDist(rng_);

	if(idPartMap.size()==1 && !isAxiom){
		parentSlot = 0;
	}


	// Select node type
	boost::random::uniform_int_distribution<> distType(0,
			conf_->allowedBodyPartTypes.size() - 1);
	char type = conf_->allowedBodyPartTypes[distType(rng_)];

	// Randomly generate node orientation
	boost::random::uniform_int_distribution<> orientationDist(0, 3);
	unsigned int curOrientation = orientationDist(rng_);

	// Randomly generate parameters
	unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type));
	std::vector<double> parameters;
	boost::random::uniform_01<double> paramDist;
	unsigned int i0Param = 0;
	//generate a random arity for the parts with variable arity
	if(PART_TYPE_IS_VARIABLE_ARITY_MAP.at(PART_TYPE_MAP.at(type))){
		std::pair<double, double> range = PART_TYPE_PARAM_RANGE_MAP.at(
					std::make_pair(PART_TYPE_MAP.at(type), 0));
		boost::random::uniform_int_distribution<> arityDist((int)range.first,(int)range.second);

		parameters.push_back(arityDist(rng_));
		i0Param = 1;
	}
	//generate random parameters that can be mutate with mutator::mutateParam
	for (unsigned int i = i0Param; i < nParams; ++i) {
		parameters.push_back(paramDist(rng_));
	}

	// Create the new part
	boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
			type, "", curOrientation, parameters);

	unsigned int newPartSlot = 0;

	if (newPart->getArity() > 0) {
		// Generate a random slot in the new node, if it has arity > 0
		// newPartSlot will have the number of the child like the .txt
		boost::random::uniform_int_distribution<> distNewPartSlot(0,
				newPart->getArity() - 1);
		newPartSlot = distNewPartSlot(rng_);
	}
	// otherwise just keep it at 0... inserting part will fail if arity is 0 and
	// there were previously parts attached to the parent's chosen slot

	return robot->insertPart(parent->first, parentSlot, newPart, newPartSlot,
			oscillatorNeuronDist_(rng_) ? NeuronRepresentation::OSCILLATOR :
					NeuronRepresentation::SIGMOID, PRINT_ERRORS);
			//todo other neuron types?
}

void IndirectMutator::randomizeBrain(boost::shared_ptr<RobotRepresentation>& robot) {
	// randomize weights and biases randomly in valid range
	boost::random::uniform_01<double> distrib;

	std::vector<double*> weights;
	std::vector<double*> params;
	std::vector<unsigned int> types;
	robot->getBrainGenome(weights, types, params);

	// set weights
	for (unsigned int i = 0; i < weights.size(); ++i) {
		*weights[i] = distrib(rng_) * (conf_->maxBrainWeight -
				conf_->minBrainWeight) + conf_->minBrainWeight;
	}
	// set params (biases, etc)
	unsigned int paramCounter = 0;
	for (unsigned int i = 0; i < types.size(); ++i) {
		if(types[i] == NeuronRepresentation::SIGMOID) {
			*params[paramCounter] = distrib(rng_) * (conf_->maxBrainBias -
					conf_->minBrainBias) + conf_->minBrainBias;
			paramCounter+=1;
		} else if(types[i] == NeuronRepresentation::CTRNN_SIGMOID) {
			*params[paramCounter] = distrib(rng_) * (conf_->maxBrainBias -
					conf_->minBrainBias) + conf_->minBrainBias;
			*params[paramCounter + 1] = distrib(rng_) * (conf_->maxBrainTau -
					conf_->minBrainTau) + conf_->minBrainTau;
			paramCounter += 2;
		} else if(types[i] == NeuronRepresentation::OSCILLATOR) {
			*params[paramCounter] = distrib(rng_) * (conf_->maxBrainPeriod -
					conf_->minBrainPeriod) + conf_->minBrainPeriod;
			*params[paramCounter + 1] = distrib(rng_) * (conf_->maxBrainPhaseOffset -
					conf_->minBrainPhaseOffset) + conf_->minBrainPhaseOffset;
			*params[paramCounter + 2] = distrib(rng_) * (conf_->maxBrainAmplitude -
					conf_->minBrainAmplitude) + conf_->minBrainAmplitude;
			paramCounter += 3;
		} else {
			std::cout << "INVALID TYPE ENCOUNTERED " << types[i] << std::endl;
		}

	}
	robot->setDirty();
}

std::vector<boost::shared_ptr<RobotRepresentation> > IndirectMutator::createOffspring(
			boost::shared_ptr<RobotRepresentation> parent1,
			boost::shared_ptr<RobotRepresentation> parent2) {

	std::vector<boost::shared_ptr<RobotRepresentation> > offspring;

	offspring.push_back(boost::shared_ptr<RobotRepresentation>(new
			RobotRepresentation(*parent1.get())));

	// only allow crossover if doing just brain mutation
	if (conf_->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER
			&& parent2) {
		offspring.push_back(boost::shared_ptr<RobotRepresentation>(new
				RobotRepresentation(*parent2.get())));
		//this->crossover(offspring[0], offspring[1]);
	}

	// Mutate
	for(size_t i = 0; i < offspring.size(); ++i) {
		this->mutate(offspring[i]);
	}

	return offspring;
}

bool IndirectMutator::mutate(boost::shared_ptr<RobotRepresentation>& robot){
	bool mutated = false;

	// mutate brain TODO conf bits?

	if (conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		mutated = (this->mutateBody(robot) || mutated);
	}

	robot->buildFromGrammar();

	if (conf_->evolutionMode == EvolverConfiguration::BRAIN_EVOLVER
			|| conf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {
		mutated = (this->mutateBrain(robot) || mutated);
	}

	return mutated;
}

bool IndirectMutator::mutateBrain(boost::shared_ptr<RobotRepresentation>& robot){
	bool mutated = false;

	// first potentially add hidden neurons
	if ((robot->getBrain()->getNumHidden() < MAX_HIDDEN_NEURONS) &&
			addHiddenNeuronDist_(rng_)) {
		unsigned int neuronType = NeuronRepresentation::SIGMOID;
		if (oscillatorNeuronDist_(rng_)) {
			neuronType = NeuronRepresentation::OSCILLATOR;
		}
		std::string neuronId = robot->getBrain()->insertNeuron(
				ioPair(robot->getBodyRootId(),
										robot->getBrain()->getBodyPartNeurons(
												robot->getBodyRootId()).size()),
							NeuronRepresentation::HIDDEN, neuronType);
		mutated = true;
	}

	// TODO allow removing hidden neurons???


	std::vector<double*> weights;
	std::vector<double*> params;
	std::vector<unsigned int> types;
	robot->getBrainGenome(weights, types, params);

	// mutate weights
	for (unsigned int i = 0; i < weights.size(); ++i) {
		if (brainMutate_(rng_)) {
			mutated = true;
			*weights[i] += (normalDistribution_(rng_) *
					conf_->brainWeightSigma);
			*weights[i] = clip(*weights[i], conf_->minBrainWeight,
					conf_->maxBrainWeight);

		}
	}
	// mutate params (biases, etc)
	unsigned int paramCounter = 0;
	for (unsigned int i = 0; i < types.size(); ++i) {
		if(types[i] == NeuronRepresentation::SIGMOID) {
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter] += (normalDistribution_(rng_) *
						conf_->brainBiasSigma);
				*params[paramCounter] = clip(*params[paramCounter],
						conf_->minBrainBias, conf_->maxBrainBias);
			}
			paramCounter+=1;
		} else if(types[i] == NeuronRepresentation::CTRNN_SIGMOID) {
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter] += (normalDistribution_(rng_) *
						conf_->brainBiasSigma);
				*params[paramCounter] = clip(*params[paramCounter],
						conf_->minBrainBias, conf_->maxBrainBias);
			}
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter+1] += (normalDistribution_(rng_) *
						conf_->brainTauSigma);
				*params[paramCounter+1] = clip(*params[paramCounter+1],
						conf_->minBrainTau, conf_->maxBrainTau);
			}
			paramCounter += 2;
		} else if(types[i] == NeuronRepresentation::OSCILLATOR) {
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter] += (normalDistribution_(rng_) *
						conf_->brainPeriodSigma);
				*params[paramCounter] = clip(*params[paramCounter],
						conf_->minBrainPeriod, conf_->maxBrainPeriod);
			}
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter+1] += (normalDistribution_(rng_) *
						conf_->brainPhaseOffsetSigma);
				*params[paramCounter+1] = clip(*params[paramCounter+1],
						conf_->minBrainPhaseOffset, conf_->maxBrainPhaseOffset);
			}
			if (brainMutate_(rng_)) {
				mutated = true;
				*params[paramCounter+2] += (normalDistribution_(rng_) *
						conf_->brainAmplitudeSigma);
				*params[paramCounter+2] = clip(*params[paramCounter+2],
						conf_->minBrainAmplitude, conf_->maxBrainAmplitude);
			}
			paramCounter += 3;
		} else {
			std::cout << "INVALID TYPE ENCOUNTERED " << types[i] << std::endl;
		}

	}

	if (mutated) {
		robot->setDirty();
	}
	
	return mutated;
}

bool IndirectMutator::mutateBody(boost::shared_ptr<RobotRepresentation>& robot){
	bool mutated = false;
	IndMutOpPair mutOpPairs[] = { 
			std::make_pair(&IndirectMutator::suppressRule, suppressRuleDist_),
			std::make_pair(&IndirectMutator::createRule, createRuleDist_),
			std::make_pair(&IndirectMutator::swapRules, swapRulesDist_),
			std::make_pair(&IndirectMutator::mutateRule, mutateRuleDist_),
			std::make_pair(&IndirectMutator::mutateAxiom, mutateAxiomDist_)};

	int numOperators = sizeof(mutOpPairs) / sizeof(IndMutOpPair);
	for (int i = 0; i < numOperators; ++i) {

		IndMutationOperator mutOp = mutOpPairs[i].first;
		boost::random::bernoulli_distribution<double> dist =
				mutOpPairs[i].second;

		if (dist(rng_)) {
			boost::shared_ptr<RobotRepresentation> newBot =
					boost::shared_ptr<RobotRepresentation>(
							new RobotRepresentation(*robot.get()));

			bool mutationSuccess = (this->*mutOp)(newBot);

			int errorCode;
			std::vector<std::pair<std::string, std::string> > affectedBodyParts;
			if (mutationSuccess
					&& BodyVerifier::verify(*newBot.get(), errorCode,
							affectedBodyParts, PRINT_ERRORS)) {

				if (!newBot->check()) {
					std::cout << "Consistency check failed in mutation operator " << i << std::endl;
				}

				robot = newBot;
				robot->setDirty();
				mutated = true;
				break;

			}
		}
	}
}

bool IndirectMutator::createRule(boost::shared_ptr<RobotRepresentation> &robot){
	//We generate a new robot, a clone
	boost::shared_ptr<RobotRepresentation> finalBot = boost::shared_ptr<RobotRepresentation>(new RobotRepresentation(*robot.get()));

	//We get the grammar from this clone
	boost::shared_ptr<Grammar> tmpGrammar = finalBot->getGrammar();

	//Now we start to modify the grammar
	boost::shared_ptr<SubRobotRepresentation> predecessor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation());
	//We give a core to the predecessor, at least, to not be mean.
	predecessor->init();

	std::vector<double> parameters;

	boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
		INVERSE_PART_TYPE_MAP.at("FixedBrick"),
		"",
		0, //orientation
		parameters);

	SubRobotRepresentation::IdPartMap::const_iterator targetPart = predecessor->getBody().begin();

	predecessor->insertPart(targetPart->first,
			0, //Parent slot
			newPart,
			0, //newPartSlot
			0 ? NeuronRepresentation::OSCILLATOR :
			NeuronRepresentation::SIGMOID,
			false);

	int attempt=0;

	if(tmpGrammar->getNumberOfRules()<20){
		while(attempt<1000){
			tmpGrammar->addRule(boost::shared_ptr<Grammar::Rule>(new Grammar::Rule(1, generateRandomPredecessor(), this->rng_, this->conf_)));

			#ifdef DEBUG_MUTATE
			std::cout << "The predecessor is:\n";
			std::cout << predecessor->toString() << std::endl;
			std::cout << "The successor is:\n";
			std::cout << tmpGrammar->getRule(tmpGrammar->getNumberOfRules()-1)->getSuccessor()->toString() << std::endl;
			#endif

			bool success = finalBot->buildFromGrammar();

			attempt++;

			int errorCode;
			std::vector<std::pair<std::string, std::string> > affectedBodyParts;
			if (success && BodyVerifier::verify(*finalBot.get(), errorCode,
								affectedBodyParts, PRINT_ERRORS)) {

				if (!finalBot->check()) {
					std::cout << "Consistency check failed in mutation operator " << std::endl;
				}

				robot = finalBot;
				robot->setDirty();
				break;
			} else {
				tmpGrammar->popLastRule();
			}
		}
	}
}

bool IndirectMutator::swapRules(boost::shared_ptr<RobotRepresentation> &robot){
	boost::shared_ptr<Grammar> tmpGrammar = robot->getGrammar();

	if(tmpGrammar->getNumberOfRules()==0){
		return false;
	}

	boost::random::uniform_int_distribution<> dist(0, tmpGrammar->getNumberOfRules()-1);

	int first = dist(rng_);
	int second=0;

	int attempt=0;
	while(attempt<100){
		second = dist(rng_);
		if(second!=first){
			break;
		}
		attempt++;
	}

	if(attempt==100){ //Couldn't find a pair of rules
		return false;
	} else {
		return tmpGrammar->swapRules(first,second);
	}
}

bool IndirectMutator::suppressRule(boost::shared_ptr<RobotRepresentation> &robot){
	boost::shared_ptr<Grammar> tmpGrammar = robot->getGrammar();

	if(tmpGrammar->getNumberOfRules()==0){
		return false;
	} else {
		boost::random::uniform_int_distribution<> dist(0, tmpGrammar->getNumberOfRules()-1);
		return tmpGrammar->popRuleAt(dist(rng_));
	}
}

bool IndirectMutator::mutateRule(boost::shared_ptr<RobotRepresentation> &robot){
	boost::shared_ptr<Grammar> tmpGrammar = robot->getGrammar();

	if(tmpGrammar->getNumberOfRules()==0){
		return false;
	}
	boost::random::uniform_int_distribution<> dist(0, tmpGrammar->getNumberOfRules()-1);

	return tmpGrammar->getRule(dist(rng_))->mutate(this->rng_, this->conf_);
}

bool IndirectMutator::mutateAxiom(boost::shared_ptr<RobotRepresentation> &robot){
	boost::random::bernoulli_distribution<double> dist(0.5);
	if(dist(rng_)){
		this->insertNode(robot);
	} else {
		this->removeNode(robot);
	}
	return true;
}

boost::shared_ptr<SubRobotRepresentation> IndirectMutator::generateRandomPredecessor(){
	boost::random::uniform_int_distribution<> dist(1, conf_->maxPredecessorParts);
	unsigned int numPartsToAdd = dist(rng_);

	boost::shared_ptr<SubRobotRepresentation> predecessor;
	predecessor.reset(new SubRobotRepresentation());
	predecessor->init();

	for (unsigned int i = 0; i < numPartsToAdd; i++) {
		bool success = false;

		for (unsigned int attempt = 0;
				(attempt < 10); ++attempt) {

			boost::shared_ptr<SubRobotRepresentation> newBot = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*predecessor.get()));

			success = this->insertNode(newBot, false);
			if (success) {
				predecessor = newBot;
				break;
			}
		}
	}

	return predecessor;
}

} //robogen