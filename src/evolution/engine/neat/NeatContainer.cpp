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

#include <boost/random/uniform_int_distribution.hpp>
#include <algorithm>

#include "evolution/engine/neat/NeatContainer.h"

//#define NEAT_CONTAINER_DEBUG

namespace robogen {

NeatContainer::NeatContainer(boost::shared_ptr<EvolverConfiguration> &evoConf,
		boost::shared_ptr<Population> &population, unsigned int seed,
		boost::random::mt19937 &rng) :
			evoConf_(evoConf), rng_(rng) {
	// create CPPN with 7 inputs (x1, y1, io1, x2, y2, io2, bias)
	// and 5 outputs: connection exists, weight, params
	neatPopulation_.reset(new NEAT::Population(NEAT::Genome(0, 7, 0, 5, false,
							NEAT::UNSIGNED_SIGMOID,NEAT::UNSIGNED_SIGMOID, 0,
							evoConf->neatParams),
						  evoConf->neatParams, true, 1.0, seed));
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

}

NeatContainer::~NeatContainer() {
}

bool NeatContainer::fillPopulationWeights(
		boost::shared_ptr<Population> &population) {

	for(NeatIdToGenomeMap::iterator i = neatIdToGenomeMap_.begin();
			i != neatIdToGenomeMap_.end(); i++) {
		unsigned int id = i->first;
		NEAT::Genome *genome = i->second;
		if(neatIdToRobotMap_.count(id) == 0) {
			std::cout << "No robot in map with id " << id << std::endl;
			return false;
		}
		boost::shared_ptr<RobotRepresentation> robot = neatIdToRobotMap_[id];
		if(!this->fillBrain(genome, robot)) {
			return false;
		}

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
		&population) {

	for(NeatIdToGenomeMap::iterator i = neatIdToGenomeMap_.begin();
				i != neatIdToGenomeMap_.end(); i++) {
		unsigned int id = i->first;
		NEAT::Genome *genome = i->second;
		boost::shared_ptr<RobotRepresentation> robot = neatIdToRobotMap_[id];
		if (!robot->isEvaluated()) {
			std::cout << "Population contains non-evaluated individuals," <<
					"cannot reproduce" << std::endl;
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
		neatIdToRobotMap_[newIds[i]] = unMappedRobots_[i];
		neatIdToRobotMap_[newIds[i]]->setDirty();
	}
	unMappedRobots_.erase(unMappedRobots_.begin(),
			unMappedRobots_.begin() + newIds.size());
	// missing individuals will stay evaluted and stay in unMapped array


	return this->fillPopulationWeights(population);
}

bool NeatContainer::fillBrain(NEAT::Genome *genome,
		boost::shared_ptr<RobotRepresentation> &robotRepresentation) {

	// Initialize ODE
	dInitODE();
	dWorldID odeWorld = dWorldCreate();
	dWorldSetGravity(odeWorld, 0, 0, 0);
	dSpaceID odeSpace = dHashSpaceCreate(0);

	bool returnValue = false;

	// code block to protect object for ODE cleanup
	// use for loop so can break out of it -- hack, I know
	for(unsigned int useless = 0; useless < 1; ++useless) {
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
		if (!robot->init(odeWorld, odeSpace, robotMessage)) {
			std::cout << "Problem when initializing robot in "
					<< "NeatContainer::fillBrain!" << std::endl;
			break;
		}

		RobotRepresentation::IdPartMap body = robotRepresentation->getBody();
		boost::shared_ptr<NeuralNetworkRepresentation> brain =
				robotRepresentation->getBrain();

		// For each body part, get its position then create an entry for every
		// neuron by adding a 4th coordinate that is the neurons ioID.

#ifdef NEAT_CONTAINER_DEBUG
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
#ifdef NEAT_CONTAINER_DEBUG
				for(unsigned int cv = 0; cv<position.size(); cv++) {
					std::cout << position[cv] << " ";
				}
				std::cout << std::endl;
#endif
			}
#ifdef NEAT_CONTAINER_DEBUG
			std::cout << std::endl;
#endif



		}

		// Now go through all neurons and query for weights and params with
		// the obtained coordinates
#ifdef NEAT_CONTAINER_DEBUG
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

#ifdef NEAT_CONTAINER_DEBUG
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

#ifdef NEAT_CONTAINER_DEBUG
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
		returnValue = true;
	}
	// Destroy ODE space
	dSpaceDestroy(odeSpace);

	// Destroy ODE world
	dWorldDestroy(odeWorld);

	// Destroy the ODE engine
	dCloseODE();

	return returnValue;

}




} /* namespace robogen */
