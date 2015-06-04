/*
 * @(#) NeatContainer.cpp   1.0   Oct 30, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014
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
#include "evolution/engine/neat/NeatContainer.h"
#include <algorithm>
#include <queue>
#include "PartList.h"

//#define NEAT_DEBUG

namespace robogen {

NeatContainer::NeatContainer(boost::shared_ptr<EvolverConfiguration> &evoConf,
		boost::shared_ptr<Population> &population, unsigned int seed,
		boost::random::mt19937 &rng) :
			evoConf_(evoConf), rng_(rng) {

	if(evoConf_->neatMode == EvolverConfiguration::BRAIN_EVOLVER) {

		if (evoConf_->evolutionaryAlgorithm ==
				EvolverConfiguration::HYPER_NEAT) {
			// create CPPN with 7 inputs (x1, y1, io1, x2, y2, io2, bias)
			// and 5 outputs: connection exists, weight, params
			neatPopulation_.reset(new NEAT::Population(NEAT::Genome(0, 7, 0, 5,
									false, NEAT::UNSIGNED_SIGMOID,
									NEAT::UNSIGNED_SIGMOID, 0,
									evoConf->neatParams),
								  evoConf->neatParams, true, 1.0, seed));
		} else {
			//NEAT or FT_NEAT

			boost::shared_ptr<NeuralNetworkRepresentation> brain =
						population->at(0)->getBrain();

			std::cout << brain->getNumInputs() << " " << brain->getNumHidden()
					<< " " << brain->getNumOutputs() << std::endl;

			// add 1 to inputs for bias
			neatPopulation_.reset(new NEAT::Population(NEAT::Genome(0,
					brain->getNumInputs() + 1, brain->getNumHidden(),
					brain->getNumOutputs(), false, NEAT::UNSIGNED_SIGMOID,
											NEAT::UNSIGNED_SIGMOID, 0,
											evoConf->neatParams, true),
										  evoConf->neatParams, true,
										  evoConf_->maxBrainWeight, seed));
		}
	} else { // full evolution
		if (evoConf_->evolutionaryAlgorithm ==
				EvolverConfiguration::HYPER_NEAT) {
			// create CPPN with 7 inputs (x1, y1, io1, x2, y2, io2, bias)
			// and 5 outputs for connection exists, weight, params
			// and 7 outputs for part exists, orientation, slot, params,
			// motor neuron type
			// + one output per type of body part
			neatPopulation_.reset(new NEAT::Population(NEAT::Genome(0, 7, 0,
									12 + evoConf_->allowedBodyPartTypes.size(),
									false, NEAT::UNSIGNED_SIGMOID,
									NEAT::UNSIGNED_SIGMOID, 0,
									evoConf->neatParams),
								  evoConf->neatParams, true, 1.0, seed));

		} else {
			std::cerr << "Cannot use NEAT or FT_NEAT with full evolution" <<
					std::endl;
			error_ = true;
			return;

		}
	}
	unsigned int count = 0;
	for(unsigned int i=0; i < neatPopulation_->m_Species.size(); i++) {
		for(unsigned int j=0;
				j < neatPopulation_->m_Species[i].m_Individuals.size();
				j++) {
			unsigned int id =
					neatPopulation_->m_Species[i].m_Individuals[j].GetID();
			neatIdToGenomeMap_[id] =
					&neatPopulation_->m_Species[i].m_Individuals[j];
			neatIdToRobotMap_[id] = population->at(count);
			count++;
		}
	}
	error_ = false;

}

NeatContainer::~NeatContainer() {
}

bool NeatContainer::fillPopulation(boost::shared_ptr<Population> &population) {

	population->clear();
	for(NeatIdToGenomeMap::iterator i = neatIdToGenomeMap_.begin();
			i != neatIdToGenomeMap_.end(); i++) {
		unsigned int id = i->first;
		NEAT::Genome *genome = i->second;
		if(neatIdToRobotMap_.count(id) == 0) {
			std::cout << "No robot in map with id " << id << std::endl;
			return false;
		}
		boost::shared_ptr<RobotRepresentation> robot = neatIdToRobotMap_[id];
		if (evoConf_->evolutionaryAlgorithm ==
				EvolverConfiguration::HYPER_NEAT) {
			if(evoConf_->neatMode == EvolverConfiguration::FULL_EVOLVER) {
				if(!this->createBodyHyperNEAT(genome,robot)) {
					return false;
				}
			}
			if(!this->fillBrainHyperNEAT(genome, robot)) {
				return false;
			}
		} else if(evoConf_->evolutionaryAlgorithm ==
				EvolverConfiguration::FT_NEAT) {
			if(!this->fillBrainNEAT(genome, robot)) {
				return false;
			}
		} else {
			std::cout << "ONLY HyperNEAT and FT-NEAT implemented!" << std::endl;
			return false;
		}
		population->push_back(robot);

	}
	return true;
}

void NeatContainer::printCurrentIds() {

	for(NeatIdToGenomeMap::iterator i = neatIdToGenomeMap_.begin();
				i != neatIdToGenomeMap_.end(); i++) {
		std::cout << i->first << " ";
	}
	std::cout << std::endl;
	for(NeatIdToRobotMap::iterator i = neatIdToRobotMap_.begin();
				i != neatIdToRobotMap_.end(); i++) {
		std::cout << i->first << " ";
	}
	std::cout << std::endl;

}

bool NeatContainer::produceNextGeneration(boost::shared_ptr<Population>
		&population, boost::shared_ptr<Mutator> &mutator) {

	for(NeatIdToGenomeMap::iterator i = neatIdToGenomeMap_.begin();
				i != neatIdToGenomeMap_.end(); i++) {
		unsigned int id = i->first;
		NEAT::Genome *genome = i->second;
		boost::shared_ptr<RobotRepresentation> robot = neatIdToRobotMap_[id];
		if (!robot->isEvaluated()) {
			std::cout << "Population contains non-evaluated individuals," <<
					" cannot reproduce" << std::endl;
			return false;
		}
		// use e^f so fitness is always positive
		genome->SetFitness(exp(robot->getFitness()));
	}

	//std::cout << "before epoch size is " << neatIdToGenomeMap_.size()
	//		<< " " << neatIdToRobotMap_.size() << std::endl;
	//printCurrentIds();
	neatPopulation_->Epoch();
	std::vector<unsigned int> currentIds;
	std::vector<unsigned int> newIds;
	std::cout <<  neatPopulation_->m_Species.size() << " species" << std::endl;
	for(unsigned int i=0; i < neatPopulation_->m_Species.size(); i++) {
		for(unsigned int j=0;
				j < neatPopulation_->m_Species[i].m_Individuals.size();
				j++) {
			unsigned int id =
					neatPopulation_->m_Species[i].m_Individuals[j].GetID();

			if (std::find(currentIds.begin(), currentIds.end(), id) !=
					currentIds.end() ) {
				// not the most efficient way to do this, but
				// pop sizes will never be too huge ;-)
				std::cout << "duplicate id: " << id << " skipping" << std::endl;
			} else {
				currentIds.push_back(id);
				if (neatIdToGenomeMap_.count(id) == 0) {
					newIds.push_back(id);
				}
				neatIdToGenomeMap_[id] =
						&neatPopulation_->m_Species[i].m_Individuals[j];
			}
		}
	}

	while(currentIds.size() > evoConf_->mu) {
		std::cout << "NEAT Pop size has grown, will delete one new individual"
				<< std::endl;

		boost::random::uniform_int_distribution<> dist(0, newIds.size()-1);
		int toRemove = dist(rng_);
		std::cout << "To remove: "  << toRemove << " " << newIds.size() << std::endl;
		unsigned int id = newIds[toRemove];
		currentIds.erase(std::find(currentIds.begin(), currentIds.end(), id));
		newIds.erase(newIds.begin() + toRemove);
		neatIdToGenomeMap_.erase(id);
		std::cout << "deleted " << id << std::endl;
	}

	int numMissing = evoConf_->mu - currentIds.size();
	if(numMissing > 0) {
		std::cout << "NEAT Pop size has shrunk" << std::endl;
	}


	// first get all the old ids so not messing with the map while iterating
	std::vector<unsigned int> oldIds;
	for(NeatIdToGenomeMap::iterator i = neatIdToGenomeMap_.begin();
				i != neatIdToGenomeMap_.end(); i++) {
		oldIds.push_back(i->first);
	}

	for(unsigned int i=0; i<oldIds.size(); i++) {
		unsigned int id = oldIds[i];
		if(std::find(currentIds.begin(), currentIds.end(), id) ==
				currentIds.end()) {
			// not in currentIds, remove from maps
			neatIdToGenomeMap_.erase(id);
			unMappedRobots_.push_back(neatIdToRobotMap_[id]);
			neatIdToRobotMap_.erase(id);
		}
	}

	if (newIds.size() != (unMappedRobots_.size() - numMissing)) {
		std::cout << "Error: new genomes is of different size than the number "
				<< "of available robots " << newIds.size()
				<< " " << unMappedRobots_.size()
				<< std::endl;
		return false;
	}

	for(unsigned int i=0; i < newIds.size(); i++) {
		if(evoConf_->neatMode == EvolverConfiguration::BRAIN_EVOLVER &&
				evoConf_->evolutionMode == EvolverConfiguration::FULL_EVOLVER) {

			// if evolving full bodies, but just using NEAT for the brain,
			// then mutate the body
			neatIdToRobotMap_[newIds[i]] =
					mutator->mutate(unMappedRobots_[i], unMappedRobots_[i]);

		} else {
			neatIdToRobotMap_[newIds[i]] = unMappedRobots_[i];
		}
		neatIdToRobotMap_[newIds[i]]->setDirty();
	}
	unMappedRobots_.erase(unMappedRobots_.begin(),
			unMappedRobots_.begin() + newIds.size());

	// missing individuals will stay evaluted and stay in unMapped array


	return this->fillPopulation(population);
}

//helper function to query neighbors
// TODO don't duplicate code from brain stuff
std::vector<osg::Vec3> getNeighboringPositions(
		boost::shared_ptr<RobotRepresentation> &robotRepresentation,
		std::string id) {

	std::vector<osg::Vec3> positions;

	// FIRST NEED TO CREATE PHYSICAL ROBOT REP TO DETERMINE POSITIONS

	// parse robot message
	robogenMessage::Robot robotMessage = robotRepresentation->serialize();
	// parse robot
	boost::shared_ptr<Robot> robot(new Robot);
	// Initialize ODE
	dInitODE();
	dWorldID odeWorld = dWorldCreate();
	dWorldSetGravity(odeWorld, 0, 0, 0);
	dSpaceID odeSpace = dHashSpaceCreate(0);
	if (!robot->init(odeWorld, odeSpace, robotMessage)) {
		std::cout << "Problem when initializing robot in "
				<< "NeatContainer::getNeighboringPositions" << std::endl;
		return positions;
	}

	for (unsigned int i=0;
			i<robotRepresentation->getBody().at(id).lock()->getArity();
			i++) {
		positions.push_back(robot->getBodyPart(id)->getSlotPosition(i));
	}

	return positions;

}


bool NeatContainer::createBodyHyperNEAT(NEAT::Genome *genome,
		boost::shared_ptr<RobotRepresentation> &robotRepresentation) {

	NEAT::NeuralNetwork net;
    genome->BuildPhenotype(net);

    robotRepresentation.reset(new RobotRepresentation());
    robotRepresentation->init();

    std::queue<std::string> partQueue;
    partQueue.push(robotRepresentation->getBodyRootId());


    // will do a breadth first search, start by querying all neighboring
    // locations of the core

    while(!partQueue.empty()) {
		if ( robotRepresentation->getBody().size() >= evoConf_->maxBodyParts ) {
			break;
		}

		std::string currentId = partQueue.front();
    	partQueue.pop();

    	unsigned int arity = robotRepresentation->getBody().at(currentId
    												  ).lock()->getArity();
    	if( arity > 0 ) {
    		std::vector<osg::Vec3> positions =
    		    	    		getNeighboringPositions(robotRepresentation,
    		    	    				currentId);
    		if(positions.size() != arity) {
    			// something went wrong
    			return false;
    		}

    		for(unsigned int parentSlot = 0; parentSlot<positions.size();
    				++parentSlot) {
    			net.Flush();
    			std::vector<double> inputs;
    			// 7 inputs
    			// first the x,y coords of this connection point
    			inputs.push_back(positions[parentSlot].x() * 10.0);
    			inputs.push_back(positions[parentSlot].y() * 10.0);
    			// next 4 are 0
    			for(unsigned j=0; j<4; j++) inputs.push_back(0.0);
    			// finally, set bias
    			inputs.push_back(1.0);

    			net.Input(inputs);
				for(int t=0; t<10; t++) {
					net.Activate();
				}
				std::vector<double> outputs = net.Output();

				// 5 is does part exist
				if (outputs[5] >= 0.5) {
					// if first output is under threshold, no part here
					// so do nothing, otherwise we query for details

					// first, what part type
					unsigned int chosenPart = 0;
					double chosenPartValue = outputs[12];

					for(unsigned int j=1;
							j< evoConf_->allowedBodyPartTypes.size(); ++j) {
						if(outputs[12 + j] >= chosenPartValue) {
							chosenPartValue = outputs[12 + j];
							chosenPart = j;
						}
					}
					char chosenPartType = evoConf_->allowedBodyPartTypes[
					                                                chosenPart];

					// already know parent slot, but...
					// need to get orientation, parameters, and child slot

					// orientation will come from output 6

					unsigned int orientation = (unsigned int) (outputs[6] * 4);
					// should truncate to be in [0,1,2,3]
					if (orientation > 3) {
						std::cout << "Orientation greater than 3: " <<
								orientation << std::endl;
						orientation = 3;
					}

					// params will come from outputs 7,8,9

					unsigned int numParams = PART_TYPE_PARAM_COUNT_MAP.at(
							PART_TYPE_MAP.at(chosenPartType));
					std::vector<double> parameters;

					for (unsigned int i = 0; i < numParams; ++i) {
						// value in [0,1]
						parameters.push_back(outputs[7 + i]);
					}

					boost::shared_ptr<PartRepresentation> newPart =
							PartRepresentation::create(chosenPartType, "",
									orientation, parameters);

					// output 10 defines the slot on the new part

					unsigned int newPartSlot = 0;

					if (newPart->getArity() > 0) {
						// Generate a random slot in the new node, if it has arity > 0
						newPartSlot = (unsigned int) (outputs[6] *
								(newPart->getArity() - 1));
						if (newPartSlot > (newPart->getArity() - 1)) {
							std::cout << "newPartSlot greater than " <<
									(newPart->getArity() - 1) << ": " <<
									newPartSlot << std::endl;
							orientation = (newPart->getArity() - 1);
						}
					}

					boost::shared_ptr<RobotRepresentation> newBot =
							boost::shared_ptr<RobotRepresentation>(
									new RobotRepresentation(
											*robotRepresentation.get()
											)
							);

					// output 11 defines motor neuron type

					if (!newBot->insertPart(currentId, parentSlot,
							newPart, newPartSlot,
							(outputs[11] < evoConf_->pOscillatorNeuron)  ?
									NeuronRepresentation::OSCILLATOR :
									NeuronRepresentation::SIGMOID)) { //todo other types?
						return false;
					}
					partQueue.push(newPart->getId());

					int errorCode;
					std::vector<std::pair<std::string, std::string> >
														affectedBodyParts;

					if (BodyVerifier::verify(*newBot.get(), errorCode,
							affectedBodyParts)) {

						if (!newBot->check()) {
							std::cout << "Consistency check failed "
									<< std::endl;
							return false;
						}

						robotRepresentation = newBot;
						robotRepresentation->setDirty();

					} // otherwise we just skip adding this part and continue

				}
    		}
    	}
    }

    return true;

}

bool NeatContainer::fillBrainHyperNEAT(NEAT::Genome *genome,
		boost::shared_ptr<RobotRepresentation> &robotRepresentation) {

	NEAT::NeuralNetwork net;
    genome->BuildPhenotype(net);

    typedef std::map<std::string, boost::weak_ptr<NeuronRepresentation> >
    	NeuronMap;

    std::map<std::string, std::vector<double> > neuronToPositionMap;
    NeuronMap neuronMap;

    // FIRST NEED TO CREATE PHYSICAL ROBOT REP TO DETERMINE POSITIONS

    // parse robot message
	robogenMessage::Robot robotMessage = robotRepresentation->serialize();
	// parse robot
	boost::shared_ptr<Robot> robot(new Robot);
	// Initialize ODE
	dInitODE();
	dWorldID odeWorld = dWorldCreate();
	dWorldSetGravity(odeWorld, 0, 0, 0);
	dSpaceID odeSpace = dHashSpaceCreate(0);
	if (!robot->init(odeWorld, odeSpace, robotMessage)) {
		std::cout << "Problem when initializing robot in "
				<< "NeatContainer::fillBrainHyperNEAT!" << std::endl;
		return false;
	}

    RobotRepresentation::IdPartMap body = robotRepresentation->getBody();
    boost::shared_ptr<NeuralNetworkRepresentation> brain =
    		robotRepresentation->getBrain();

    // For each body part, get its position then create an entry for every
    // neuron by adding a 4th coordinate that is the neurons ioID.
#ifdef NEAT_DEBUG
    std::cout << "POSITIONS: " << std::endl;
#endif
    for(RobotRepresentation::IdPartMap::iterator i = body.begin();
    		i != body.end(); i++) {
    	std::string id = i->first;
    	boost::shared_ptr<PartRepresentation> part = i->second.lock();
    	std::vector<boost::weak_ptr<NeuronRepresentation> > neurons =
    			brain->getBodyPartNeurons(part->getId());
    	osg::Vec3 pos = robot->getBodyPart(id)->getRootPosition();


    	for (unsigned int j = 0; j<neurons.size(); j++) {
    		std::vector<double> position;
    		position.push_back(pos.x() * 10.0); //roughly something in [-1,1]
    		position.push_back(pos.y() * 10.0);
    		//position.push_back(pos.z());
    		float io = neurons[j].lock()->getIoPair().second;
    		position.push_back((io/10.0));
    		neuronToPositionMap[neurons[j].lock()->getId()] = position;
    		neuronMap[neurons[j].lock()->getId()] = neurons[j];
#ifdef NEAT_DEBUG
    		for(unsigned int cv = 0; cv<position.size(); cv++) {
    			std::cout << position[cv] << " ";
    		}
    		std::cout << std::endl;
#endif
    	}
#ifdef NEAT_DEBUG
    	std::cout << std::endl;
#endif



    }

    // Now go through all neurons and query for weights and params with
    // the obtained coordinates
#ifdef NEAT_DEBUG
    std::cout << "**************************";
#endif
    for(NeuronMap::iterator i = neuronMap.begin(); i != neuronMap.end(); i++) {
    	std::vector<double> positionI = neuronToPositionMap[i->first];
    	boost::shared_ptr<NeuronRepresentation> neuronI = i->second.lock();
    	for(NeuronMap::iterator j = neuronMap.begin(); j != neuronMap.end();
    			j++) {
    		std::vector<double> positionJ = neuronToPositionMap[j->first];
    		boost::shared_ptr<NeuronRepresentation> neuronJ = j->second.lock();

			if (brain->connectionExists(neuronI->getId(), neuronJ->getId())) {
    			// only set weights on existing connections
				net.Flush();
				std::vector<double> inputs;
				for (unsigned int k = 0; k < positionI.size(); k++) {
					inputs.push_back(positionI[k]);
				}
				for (unsigned int k = 0; k < positionJ.size(); k++) {
					inputs.push_back(positionJ[k]);
				}
				inputs.push_back(1.0); //bias

#ifdef NEAT_DEBUG
				std::cout << "INPUTS: ";
				for (unsigned int cv = 0; cv < inputs.size(); cv++) {
					std::cout << inputs[cv] << " ";
				}
				std::cout << std::endl;
#endif

				net.Input(inputs);
				for(int t=0; t<10; t++) {
					net.Activate();
				}
				std::vector<double> outputs = net.Output();

#ifdef NEAT_DEBUG
				std::cout << "OUTPUTS: ";
				for (unsigned int cv = 0; cv < outputs.size(); cv++) {
					std::cout << outputs[cv] << " ";
				}
				std::cout << std::endl;
#endif

				if (outputs[0] < 0.5) {
					// if first output is under threshold,
					// connection "does not exist" according to genome so set
					// weight to 0
					brain->setWeight(neuronI->getIoPair(),
							neuronJ->getIoPair(), 0.0);
				} else {
					// otherwise use the second output
					// translate from [0,1] to [min, max]
					double weight = outputs[1] * (evoConf_->maxBrainWeight -
							evoConf_->minBrainWeight) +
							evoConf_->minBrainWeight;
					brain->setWeight(neuronI->getIoPair(),
							neuronJ->getIoPair(), weight);
				}
    		}
    	}

    	// now query for parameters
    	if (neuronI->getLayer() != NeuronRepresentation::INPUT) {
			// input neurons don't have params
			net.Flush();
			std::vector<double> inputs;
			for (unsigned int k = 0; k < positionI.size(); k++) {
				inputs.push_back(positionI[k]);
			}
			for (unsigned int k = 0; k < positionI.size(); k++) {
				inputs.push_back(0.0); // 0 for second set of coords
			}
			inputs.push_back(1.0); //bias
			net.Input(inputs);
			for(int t=0; t<10; t++) {
				net.Activate();
			}
			std::vector<double> outputs = net.Output();
			std::vector<double> params;
			if(neuronI->getType() == NeuronRepresentation::SIGMOID ||
					neuronI->getType() == NeuronRepresentation::CTRNN_SIGMOID){
				// bias
				params.push_back(outputs[2] * (evoConf_->maxBrainBias -
						evoConf_->minBrainBias) + evoConf_->minBrainBias);
				if(neuronI->getType() == NeuronRepresentation::CTRNN_SIGMOID) {
					// tau
					params.push_back(outputs[3] * (evoConf_->maxBrainTau -
							evoConf_->minBrainTau) + evoConf_->minBrainTau);
				}
			} else if(neuronI->getType() == NeuronRepresentation::OSCILLATOR) {
				// period
				params.push_back( outputs[2] * (evoConf_->maxBrainPeriod -
						evoConf_->minBrainPeriod) + evoConf_->minBrainPeriod);
				// phase offset
				params.push_back( outputs[3] * (evoConf_->maxBrainPhaseOffset -
						evoConf_->minBrainPhaseOffset) +
						evoConf_->minBrainPhaseOffset);
				// amplitude
				params.push_back( outputs[4] * (evoConf_->maxBrainAmplitude -
						evoConf_->minBrainAmplitude) +
						evoConf_->minBrainAmplitude);
			} else {
				std::cout << "INVALID TYPE ENCOUNTERED " << neuronI->getType()
						<< std::endl;
			}
			neuronI->setParams(params);
    	}
    }
    return true;

}

bool NeatContainer::fillBrainNEAT(NEAT::Genome *genome,
		boost::shared_ptr<RobotRepresentation> &robotRepresentation) {

	typedef std::map<std::string, boost::weak_ptr<NeuronRepresentation> >
    	NeuronMap;

    NeuronMap neuronMap;

    std::vector<double*> weights;
    std::vector<double*> params;
    std::vector<unsigned int> types;
    robotRepresentation->getBrainGenome(weights, types, params);

    for(unsigned int i = 0; i < weights.size(); i++) {
    	*weights[i] = genome->GetLinkByIndex(i).GetWeight();
    }

    // get biases, assume they come after other links for now
    for(unsigned int i = 0; i < params.size(); i++) {
    	*params[i] = genome->GetLinkByIndex(i + weights.size()).GetWeight();
    }
    return true;

}



} /* namespace robogen */
