/*
 * @(#) Robot.cpp   1.0   Mar 4, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <string>

#include "utils/RobogenUtils.h"
#include "Models.h"
#include "Robot.h"
#include "model/Connection.h"

#include "arduino/ArduinoNNConfiguration.h"

namespace robogen {

/**
 * Visits a body tree and connects body parts accordingly
 */
class BodyConnectionVisitor: public boost::default_bfs_visitor {

public:

	BodyConnectionVisitor(dWorldID& odeWorld,
			dJointGroupID connectionJointGroup, Robot *robot) :
			odeWorld_(odeWorld), 
			connectionJointGroup_(connectionJointGroup), robot_(robot) {
	}

	/**
	 * From boost doc: This is invoked on each edge as it becomes a member of
	 * the edges that form the search tree.
	 */
	void tree_edge(BodyEdge v, const BodyGraph& g) const {
		boost::property_map<BodyGraph, BodyEdgeDescriptorTag>::const_type bodyConnectionMap =
				boost::get(BodyEdgeDescriptorTag(), g);

		boost::shared_ptr<Connection> c = boost::get(bodyConnectionMap, v);

		// This is the gist of the bfs visitor
		boost::shared_ptr<Joint> joint = RobogenUtils::connect(c->getTo(),
				c->getToSlot(), c->getFrom(), c->getFromSlot(),
				connectionJointGroup_, odeWorld_);

		robot_->addJoint(joint);

		return;
	}

private:

	dWorldID& odeWorld_;

	dJointGroupID connectionJointGroup_;

	Robot *robot_;

};

Robot::Robot() :
		connectionJointGroup_(NULL) {

}

bool Robot::init(dWorldID odeWorld, dSpaceID odeSpace,
		const robogenMessage::Robot& robotSpec,
		bool printInfo) {
	odeWorld_ = odeWorld;
	odeSpace_ = odeSpace;
	robotMessage_ = &robotSpec;
	printInfo_ = printInfo;

	connectionJointGroup_ = dJointGroupCreate(0);
	this->id_ = robotSpec.id();

	const robogenMessage::Body& body = robotSpec.body();
	const robogenMessage::Brain& brain = robotSpec.brain();
	if (!this->decodeBody(body)) {
		std::cout << "Cannot decode the body of the robot."
				<< std::endl;
		return false;
	}
	// decode brain needs to come after decode body, as IO reordering
	if (!this->decodeBrain(brain)) {
		std::cout << "Cannot decode the brain of the robot."
				<< std::endl;
		return false;
	}

	return true;
}

Robot::~Robot() {
	if (connectionJointGroup_) {
		dJointGroupDestroy(connectionJointGroup_);
	}
}

const std::vector<boost::shared_ptr<Sensor> >& Robot::getSensors() const {
	return sensors_;
}

const std::vector<boost::shared_ptr<Motor> >& Robot::getMotors() {
	return motors_;
}

const std::vector<boost::shared_ptr<Model> >& Robot::getBodyParts() {
	return bodyParts_;
}

const boost::shared_ptr<NeuralNetwork>& Robot::getBrain() const {
	return neuralNetwork_;
} // Decode the body connections

boost::shared_ptr<Model> Robot::getCoreComponent() {
	return coreComponent_;
}

bool Robot::decodeBody(const robogenMessage::Body& robotBody) {

	float x = 0;
	float y = 0;
	float z = 200;
	float spacing = 200;
	rootNode_ = -1;

	int numDigitalPins = 0, numAnalogPins = 0, numPwmPins = 0;

	for (int i = 0; i < robotBody.part_size(); ++i) {

		const robogenMessage::BodyPart& bodyPart = robotBody.part(i);
		boost::shared_ptr<Model> model = RobogenUtils::createModel(bodyPart,
				odeWorld_, odeSpace_);

		if (model == NULL) {
			std::cerr << "Unrecognized body part: " << bodyPart.id()
					<< "." << std::endl;
			return false;
		}

		model->initModel();
		model->setRootPosition(osg::Vec3(x, y, z));
		bodyParts_.push_back(model);
		bodyPartsMap_.insert(std::pair<std::string, int>(bodyPart.id(), i));

		if (boost::dynamic_pointer_cast<PerceptiveComponent>(model)) {
			std::vector<boost::shared_ptr<Sensor> > sensors;
			boost::dynamic_pointer_cast<PerceptiveComponent>(model)->getSensors(
					sensors);
			bodyPartsToSensors_.insert(
					std::pair<int, std::vector<boost::shared_ptr<Sensor> > >(i,
							sensors));

			sensors_.insert(sensors_.end(), sensors.begin(), sensors.end());


			for(unsigned int s=0; s<sensors.size(); s++) {
				if(boost::dynamic_pointer_cast<LightSensor>(sensors[s])) {
					// light sensors need analog pins
					numAnalogPins++;
				} else if(boost::dynamic_pointer_cast<TouchSensor>(
						sensors[s])) {
					// touch sensors need digital pins
					numDigitalPins++;
				} else if(boost::dynamic_pointer_cast<IrSensorElement>(
						sensors[s])) {
					numAnalogPins++;
				}
			}

		} else if (boost::dynamic_pointer_cast<ActuatedComponent>(model)) {
			std::vector<boost::shared_ptr<Motor> > motors;
			boost::dynamic_pointer_cast<ActuatedComponent>(model)->getMotors(
					motors);
			bodyPartsToMotors_.insert(
					std::pair<int, std::vector<boost::shared_ptr<Motor> > >(i,
							motors));

			motors_.insert(motors_.end(), motors.begin(), motors.end());

			for(int m = 0; m < motors.size(); ++m) {
				if(boost::dynamic_pointer_cast<ServoMotor>(motors[m]))
					numDigitalPins++;
				else {
					numDigitalPins++;
					numPwmPins++;
				}

			}

		}

		if (bodyPart.root()) {
			if (printInfo_)
				std::cout << "Root node! " << i << std::endl;
			rootNode_ = i;
			coreComponent_ = model;
		}

		if(printInfo_) {
			std::cout << "Node: " << i << " id: " << bodyPart.id() << " type: "
				<< bodyPart.type() << std::endl;
		}

		x += spacing;
		y += spacing;

	}

	if (numAnalogPins > MAX_ANALOG_PINS) {
		std::cout << "The number of analog pins required ("
				<< numAnalogPins
				<< ") is greater than the maximum allowed one ("
				<< MAX_ANALOG_PINS << ")" << std::endl;
		return false;
	}


	if (numDigitalPins > MAX_DIGITAL_PINS + (MAX_ANALOG_PINS - numAnalogPins)) {

		std::cout << "The number of digital pins required ("
				<< numDigitalPins
				<< ") is greater than the number available ("
				<< MAX_DIGITAL_PINS + (MAX_ANALOG_PINS - numAnalogPins)
				<< ")" << std::endl;
		return false;
	}

	if ((numPwmPins + 1) > MAX_PWM_PINS) { // need 1 for the neutral signal
		std::cout << "The number of PWM pins required ("
					<< (numPwmPins + 1)
					<< ") is greater than the number available ("
					<< MAX_PWM_PINS
					<< ")" << std::endl;
			return false;

	}



	// Look for the root node and modify its position to the origin
	bodyParts_[rootNode_]->setRootPosition(osg::Vec3(0, 0, 0));

	// get the connections
	bodyTree_.reset(new BodyGraph(robotBody.part_size()));

	bodyConnections_.reserve(robotBody.connection_size());
	for (int i = 0; i < robotBody.connection_size(); ++i) {
		bodyConnections_.push_back(
				boost::shared_ptr<Connection>(new Connection()));
		if (!bodyConnections_.back()->init(robotBody.connection(i),
				bodyPartsMap_, bodyParts_)) {
			std::cout << "Problem when connecting body parts!" << std::endl;
			return false;
		}
		boost::add_edge(bodyPartsMap_[robotBody.connection(i).src()],
				bodyPartsMap_[robotBody.connection(i).dest()],
				BodyEdgeProperty(bodyConnections_.back()), *bodyTree_);
	}

	// Let's run first a connectivity check
	std::vector<int> component(robotBody.part_size());

	BodyUndirectedGraph bodyTreeUndirected;
	boost::copy_graph(*bodyTree_, bodyTreeUndirected);

	int numComponents = boost::connected_components(bodyTreeUndirected,
			&component[0]);
	if (numComponents != 1) {
		std::cout
				<< "The robot body has some disconnected component (ConnComponents: "
				<< numComponents << ")" << std::endl;
		return false;
	}
	// End of connectivity check

	// We now have to connect them properly
	this->reconnect();

	if(printInfo_) {
		std::cout << "Sensors: " << sensors_.size() << ", motors: "
			<< motors_.size() << std::endl;
	}

	return true;

}

bool Robot::decodeBrain(const robogenMessage::Brain& robotBrain) {

	unsigned int nInputs = 0;
	unsigned int nOutputs = 0;
	unsigned int nHidden = 0;

	std::map<std::string, bool> isNeuronInput;

	std::map<std::string, unsigned int> inputNeuronIds;
	// maps from position in input array to body part's position in part vector
	std::vector<unsigned int> brainInputToBodyPart;
	std::vector<unsigned int> brainInputToIoId;

	std::map<std::string, unsigned int> outputNeuronIds;
	std::vector<unsigned int> brainOutputToBodyPart;
	std::vector<unsigned int> brainOutputToIoId;

	std::map<std::string, unsigned int> hiddenNeuronIds;
	//std::vector<unsigned int> brainHiddenToBodyPart;
	//std::vector<unsigned int> brainHiddenToIoId;

	float weight[(MAX_INPUT_NEURONS + MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)
	             * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];

	float params[MAX_PARAMS * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	unsigned int types[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];

	//temporary arrays for storing params of hidden neurons
	float hidden_params[MAX_PARAMS * MAX_HIDDEN_NEURONS];
	unsigned int hidden_types[MAX_HIDDEN_NEURONS];

	// Fill it with zeros
	memset(weight, 0, sizeof(weight));

	// Read neurons
	for (int i = 0; i < robotBrain.neuron_size(); ++i) {

		const robogenMessage::Neuron& neuron = robotBrain.neuron(i);
		if (neuron.layer().compare("input") == 0) {

			// Retrieve the body part id from the neuron part id
			const std::map<std::string, unsigned int>::iterator bodyPartId =
					bodyPartsMap_.find(neuron.bodypartid());

			if (bodyPartId == bodyPartsMap_.end()) {
				std::cout << "Cannot find a body part with id '"
						<< neuron.bodypartid()
						<< "' to be associated with neuron " << i << std::endl;
				return false;
			}

			// // +1 because we have not incremented the counter yet
			if ((nInputs + 1) > MAX_INPUT_NEURONS) {
				std::cout << "The number of input neurons (" << (nInputs + 1)
						<< ") is greater than the maximum allowed one ("
						<< MAX_INPUT_NEURONS << ")" << std::endl;
				return false;
			}

			brainInputToBodyPart.push_back(bodyPartId->second);
			brainInputToIoId.push_back(neuron.ioid());
			inputNeuronIds.insert(
					std::pair<std::string, unsigned int>(neuron.id(), nInputs));
			isNeuronInput.insert(
					std::pair<std::string, bool>(neuron.id(), true));

			nInputs++;

		} else if (neuron.layer().compare("output") == 0) {

			// Retrieve the body part id from the neuron part id
			const std::map<std::string, unsigned int>::iterator bodyPartId =
					bodyPartsMap_.find(neuron.bodypartid());

			if (bodyPartId == bodyPartsMap_.end()) {
				std::cout << "Cannot find a body part with id '"
						<< neuron.bodypartid()
						<< "' to be associated with neuron " << i << std::endl;
				return false;
			}

			// +1 because we have not incremented the counter yet
			if ((nOutputs + 1) > MAX_OUTPUT_NEURONS) {
				std::cout << "The number of output neurons (" << (nOutputs + 1)
						<< ") is greater than the maximum allowed one ("
						<< MAX_OUTPUT_NEURONS << ")" << std::endl;
				return false;
			}

			brainOutputToBodyPart.push_back(bodyPartId->second);
			brainOutputToIoId.push_back(neuron.ioid());
			outputNeuronIds.insert(
					std::pair<std::string, unsigned int>(neuron.id(),
							nOutputs));
			isNeuronInput.insert(
					std::pair<std::string, bool>(neuron.id(), false));



			if (neuron.type().compare("sigmoid") == 0) {
				params[nOutputs * MAX_PARAMS] = neuron.bias();
				params[nOutputs * MAX_PARAMS + 1] = neuron.gain();
				types[nOutputs] = SIGMOID;
			} else if (neuron.type().compare("oscillator") == 0) {
				params[nOutputs * MAX_PARAMS] = neuron.period();
				params[nOutputs * MAX_PARAMS + 1] = neuron.phaseoffset();
				params[nOutputs * MAX_PARAMS + 2] = neuron.gain();
				types[nOutputs] = OSCILLATOR;
			} else {
				//TODO add in this stuff for other neurons
				std::cout << "only sigmoid and oscillator neurons supported "
						<< "currently" << std::endl;
				return false;
			}
			nOutputs++;

		} else if (neuron.layer().compare("hidden") == 0) {
			// Retrieve the body part id from the neuron part id
			//  -- even hidden neurons should be associated with a body part!
			const std::map<std::string, unsigned int>::iterator bodyPartId =
					bodyPartsMap_.find(neuron.bodypartid());

			if (bodyPartId == bodyPartsMap_.end()) {
				std::cout << "Cannot find a body part with id '"
						<< neuron.bodypartid()
						<< "' to be associated with neuron " << i << std::endl;
				return false;
			}

			// +1 because we have not incremented the counter yet
			if ((nHidden + 1) > MAX_HIDDEN_NEURONS) {
				std::cout << "The number of hidden neurons (" << (nHidden + 1)
						<< ") is greater than the maximum allowed one ("
						<< MAX_HIDDEN_NEURONS << ")" << std::endl;
				return false;
			}

			//brainHiddenToBodyPart.push_back(bodyPartId->second);
			//brainHiddenToIoId.push_back(neuron.ioid());
			hiddenNeuronIds.insert(ioPair(neuron.id(),nHidden));
			isNeuronInput.insert(
					std::pair<std::string, bool>(neuron.id(), false));



			if (neuron.type().compare("sigmoid") == 0) {
				hidden_params[nHidden * MAX_PARAMS] = neuron.bias();
				hidden_params[nHidden * MAX_PARAMS + 1] = neuron.gain();
				hidden_types[nHidden] = SIGMOID;
			} else if (neuron.type().compare("oscillator") == 0) {
				hidden_params[nHidden * MAX_PARAMS] = neuron.period();
				hidden_params[nHidden * MAX_PARAMS + 1] = neuron.phaseoffset();
				hidden_params[nHidden * MAX_PARAMS + 2] = neuron.gain();
				hidden_types[nHidden] = OSCILLATOR;
			} else {
				//TODO add in this stuff for other neurons
				std::cout << "only sigmoid and oscillator neurons supported currently" << std::endl;
				return false;
			}
			nHidden++;
		} else {
			std::cout << "Unsupported layer for neuron " << i << std::endl;
			return false;
		}

	}

	// now stick all hidden_params and types in after motor neurons
	for (unsigned int i=0; i<nHidden; i++) {
		types[nOutputs + i] = hidden_types[i];
		for (unsigned int j=0; j<MAX_PARAMS; j++) {
			params[(nOutputs + i)*MAX_PARAMS + j] = hidden_params[i*MAX_PARAMS + j];
		}
	}

	unsigned int nNonInputs = nOutputs + nHidden;

	// Reorder robot sensors/actuators according to order in neural network
	// input array, for faster access
	std::vector<boost::shared_ptr<Sensor> > orderedSensors;
	std::vector<boost::shared_ptr<Motor> > orderedMotors;

	orderedSensors.resize(nInputs);
	orderedMotors.resize(nOutputs);

	// up to here, sensors are kept in a map pointing body parts to sensors
	// iterating through the
	for (unsigned int i = 0; i < nInputs; ++i) {
		// Find the sensor
		std::map<unsigned int, std::vector<boost::shared_ptr<Sensor> > >::iterator it =
				bodyPartsToSensors_.find(brainInputToBodyPart[i]);
		std::vector<boost::shared_ptr<Sensor> > curSensors = it->second;

		// verify that io id is within the sensor size
		unsigned int pos = brainInputToIoId[i];
		if (pos >= curSensors.size()) {
			std::cout << "Cannot locate sensor " << pos << " in body part "
					<< brainInputToBodyPart[i] << std::endl;
			return false;
		}
		orderedSensors[i] = curSensors[pos];
	}

	for (unsigned int i = 0; i < nOutputs; ++i) {

		// Find the motor
		std::map<unsigned int, std::vector<boost::shared_ptr<Motor> > >::iterator it =
				bodyPartsToMotors_.find(brainOutputToBodyPart[i]);
		std::vector<boost::shared_ptr<Motor> > curMotors = it->second;

		unsigned int pos = brainOutputToIoId[i];
		if (pos >= curMotors.size()) {
			std::cout << "Cannot locate motor " << pos << " in body part "
					<< brainOutputToBodyPart[i] << std::endl;
			return false;
		}
		orderedMotors[i] = curMotors[pos];

		// TODO need to reorder params and types????

	}

	if (nInputs != sensors_.size()) {
		std::cout << "The number of input neurons (" << nInputs
				<< ") differs from the number of sensors (" << sensors_.size()
				<< ")" << std::endl;
		return false;
	}

	if (nOutputs != motors_.size()) {
		std::cout << "The number of output neurons (" << nOutputs
				<< ") differs from the number of motors (" << motors_.size()
				<< ")" << std::endl;
		return false;
	}

	if(printInfo_) {
		std::cout << "Contents of unordered sensors:" << std::endl;
		for (unsigned int i = 0; i < sensors_.size(); ++i) {
			std::cout << sensors_[i]->getLabel() << std::endl;
		}
	}
	sensors_ = orderedSensors;
	if(printInfo_) {
		std::cout << "Contents of ordered sensors:" << std::endl;
		for (unsigned int i = 0; i < sensors_.size(); ++i) {
			std::cout << sensors_[i]->getLabel() << std::endl;
		}
	}
	motors_ = orderedMotors;

	// Count how many neurons
	neuralNetwork_.reset(new NeuralNetwork);

	// Decode the connections
	for (int i = 0; i < robotBrain.connection_size(); ++i) {

		const robogenMessage::NeuralConnection& connection =
				robotBrain.connection(i);

		// Find source and destination neurons
		std::map<std::string, bool>::iterator isSourceInputIt =
				isNeuronInput.find(connection.src());
		if (isSourceInputIt == isNeuronInput.end()) {
			std::cout << "Cannot find source neuron in the neural network: "
					<< connection.src() << std::endl;
			return false;
		}

		std::map<std::string, bool>::iterator isDestInputIt =
				isNeuronInput.find(connection.dest());
		if (isDestInputIt == isNeuronInput.end()) {
			std::cout
					<< "Cannot find destination neuron in the neural network: "
					<< connection.dest() << std::endl;
			return false;
		}

		int sourceNeuronPos;
		if (isSourceInputIt->second == true) {
			sourceNeuronPos = inputNeuronIds[connection.src()];
		} else {
			// can be either output or hidden
			if( outputNeuronIds.count(connection.src()))
				sourceNeuronPos = outputNeuronIds[connection.src()];
			else if( hiddenNeuronIds.count(connection.src())) {
				sourceNeuronPos = hiddenNeuronIds[connection.src()] + nOutputs;
			} else {
				std::cout << "Problem with source neuron "
						<< connection.src() << std::endl;
				return false;
			}
		}

		// Cannot be an input neuron
		int destNeuronPos;
		if( outputNeuronIds.count(connection.dest()))
			destNeuronPos = outputNeuronIds[connection.dest()];
		else if( hiddenNeuronIds.count(connection.dest())) {
			destNeuronPos = hiddenNeuronIds[connection.dest()] + nOutputs;
		} else {
			std::cout << "Problem with dest neuron "
					<< connection.dest() << std::endl;
			return false;
		}

		if (isSourceInputIt->second == true) {
			weight[sourceNeuronPos * nNonInputs + destNeuronPos] =
					connection.weight();
		} else {
			// put in after all connections from inputs
			weight[nInputs * nNonInputs + sourceNeuronPos * nNonInputs
					+ destNeuronPos] = connection.weight();
		}
		if(printInfo_) {
			std::cout<<"connection, src: " << connection.src() << ", dest: " <<
				connection.dest() << " " << connection.weight() << " " <<
				sourceNeuronPos << " " << destNeuronPos << std::endl;
		}
	}
	::initNetwork(neuralNetwork_.get(), nInputs, nOutputs, nHidden,
			&weight[0], &params[0], &types[0]);

	return true;

}

void Robot::optimizePhysics() {
#ifdef DEBUG_OPTIMIZE
	std::cout << "*********************************************************************************\n";
	std::cout << "***********              OPTIMIZING PHYSICS          ****************************\n";
	std::cout << "*********************************************************************************\n";
	std::cout << "\n\n\n";

	std::cout << "body parts: " << std::endl;

	for(size_t i=0; i<this->bodyParts_.size(); ++i) {
		std::cout << bodyParts_[i]->getId() << std::endl;
		for(size_t j=0; j<this->bodyParts_[i]->getBodies().size(); ++j) {
			std::cout << "\t";
			const osg::Vec3 pos = this->bodyParts_[i]->getBodies()[j]->getPosition();
			const osg::Quat rot = this->bodyParts_[i]->getBodies()[j]->getAttitude();
			std::cout << "Physical body:" << this->bodyParts_[i]->getBodies()[j]
			          << " attached to "
			          << this->bodyParts_[i]->getBodies()[j]->getBody()
			          << " located at "
			          << pos[0] << " " << pos[1] << " " << pos[2]
					  << " with rotation "
					  << rot[0] << " " << rot[1] << " " << rot[2]
					  << " " << rot[3] << std::endl;
		}
	}
#endif

	composites_.clear();
	unsigned int numFixed = 0, numHinge = 0;

	std::vector<boost::shared_ptr<Joint> >::iterator it=joints_.begin();
	std::set<boost::shared_ptr<Joint> > uniqueJoints;
	// iterate over vector so order is determined by insertion,
	// but use set to prevent duplicates
	while(it!=joints_.end()) {
		if (uniqueJoints.count((*it)) > 0) {
			continue;
		}
		uniqueJoints.insert(*it);

		if ((*it)->getType() == Joint::FIXED) {
			boost::shared_ptr<AbstractBody> bodyA = (*it)->getBodyA().lock();
			boost::shared_ptr<AbstractBody> bodyB = (*it)->getBodyB().lock();
#ifdef DEBUG_OPTIMIZE
			std::cout << "Pre -" << std::endl;
			std::cout << "\tA: " << bodyA << " " << bodyA->getJoints().size() << std::endl;
			std::cout << "\tB: " << bodyB << " " << bodyB->getJoints().size() << std::endl;
#endif
			// reset the joint (which will remove it from the bodies)
			// before creating composite
			(*it)->reset();
			// also remove it from the set
			it = joints_.erase(it);
#ifdef DEBUG_OPTIMIZE
			std::cout << "Post -" << std::endl;
			std::cout << "\tA: " << bodyA << " " << bodyA->getJoints().size() << std::endl;
			std::cout << "\tB: " << bodyB << " " << bodyB->getJoints().size() << std::endl;
#endif
			numFixed++;

			std::vector<boost::shared_ptr<AbstractBody> > toMerge;
			toMerge.push_back( bodyA );
			toMerge.push_back( bodyB );

			boost::shared_ptr<CompositeBody> composite(new CompositeBody());
			composite->init(toMerge, odeWorld_, true);
			composites_.push_back(composite);


		} else {
			it++;
			numHinge++;
		}
	}
#ifdef DEBUG_OPTIMIZE
	std::cout << numFixed << " fixed joints!" << std::endl;
	std::cout << numHinge << " hinge joints!" << std::endl;


	std::cout << composites_.size() << " composites\n";

	for(size_t i=0; i<composites_.size(); ++i) {
		std::cout << i << " " << composites_[i] << " " << composites_[i]->str() << std::endl;
	}
	std::cout << "********************************************************\n";
	std::cout << "body parts: " << std::endl;
	for(size_t i=0; i<this->bodyParts_.size(); ++i) {
		std::cout << bodyParts_[i]->getId() << std::endl;
		for(size_t j=0; j<this->bodyParts_[i]->getBodies().size(); ++j) {
			std::cout << "\t";
			const osg::Vec3 pos = this->bodyParts_[i]->getBodies()[j]->getPosition();
			const osg::Quat rot = this->bodyParts_[i]->getBodies()[j]->getAttitude();
			std::cout << "Physical body:" << this->bodyParts_[i]->getBodies()[j]
					  << " attached to "
					  << this->bodyParts_[i]->getBodies()[j]->getBody()
					  << " located at "
					  << pos[0] << " " << pos[1] << " " << pos[2]
					  << " with rotation "
					  << rot[0] << " " << rot[1] << " " << rot[2]
					  << " " << rot[3] << std::endl;
		}
	}
#endif
}


void Robot::reconnect() {
#ifdef DEBUG_OPTIMIZE
	std::cout << "------------------ RECONNECTING ROBOT ------------------" << std::endl;
#endif
	// Let's now actually connect the body parts
	// vis will do the job
	BodyConnectionVisitor vis(odeWorld_, connectionJointGroup_, this);
	// purge current connection joint group

	std::vector<boost::shared_ptr<Joint> >::iterator it=joints_.begin();
	while(it!=joints_.end()) {
		(*it)->reset();
		++it;
	}
	this->joints_.clear();

	dJointGroupEmpty(connectionJointGroup_);

	boost::breadth_first_search(*bodyTree_, rootNode_, boost::visitor(vis));
}

int Robot::getRoot() {
	return rootNode_;
}
void Robot::translateRobot(const osg::Vec3& translation) {

	for (unsigned int i = 0; i < this->bodyParts_.size(); ++i) {
		this->bodyParts_[i]->translateRootPosition(translation);
	}

}

void Robot::rotateRobot(const osg::Quat &rot) {
	for (unsigned int i = 0; i < this->bodyParts_.size(); ++i) {
		// rotate all parts. this will also rotate the root
		// TODO rotate root only once known
		this->bodyParts_[i]->setRootAttitude(rot);
	}
	// needs reconnecting (would have even if all parts were correctly rotated)
	this->reconnect();
}

void Robot::getBB(double& minX, double& maxX, double& minY, double& maxY,
		double& minZ, double& maxZ) {

	minX = 100000000;
	minY = 100000000;
	minZ = 100000000;
	maxX = -100000000;
	maxY = -100000000;
	maxZ = -100000000;

	std::vector<boost::shared_ptr<SimpleBody> > bodies;
	for (unsigned int i = 0; i < this->bodyParts_.size(); ++i) {
		std::vector<boost::shared_ptr<SimpleBody> > tempBodies =
				this->bodyParts_[i]->getBodies();
		bodies.insert(bodies.begin(), tempBodies.begin(), tempBodies.end());
	}

	for (unsigned int i = 0; i < bodies.size(); ++i) {

		dGeomID curBodyGeom = dBodyGetFirstGeom(bodies[i]->getBody());

		while(curBodyGeom) {
			dReal aabb[6];
			dGeomGetAABB(curBodyGeom, aabb);

			if (aabb[0] < minX) {
				minX = aabb[0];
			}

			if (aabb[1] > maxX) {
				maxX = aabb[1];
			}

			if (aabb[2] < minY) {
				minY = aabb[2];
			}

			if (aabb[3] > maxY) {
				maxY = aabb[3];
			}

			if (aabb[4] < minZ) {
				minZ = aabb[4];
			}

			if (aabb[5] > maxZ) {
				maxZ = aabb[5];
			}
			curBodyGeom = dBodyGetNextGeom(curBodyGeom);
		}
	}
}

boost::shared_ptr<Model> Robot::getBodyPart(std::string id) {
	return bodyParts_[bodyPartsMap_[id]];
}


int Robot::getId() const {
	return id_;
}

const robogenMessage::Robot& Robot::getMessage() {
	return *robotMessage_;
}

const std::vector<boost::shared_ptr<Connection> >& Robot::getBodyConnections() const {

	return bodyConnections_;

}

void Robot::traverseBody(const std::vector<boost::shared_ptr<Model> >,
		const std::vector<boost::shared_ptr<Connection> >) {

}

}
