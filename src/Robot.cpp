/*
 * @(#) Robot.cpp   1.0   Mar 4, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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

namespace robogen {

struct BodyEdgeDescriptorTag {
	typedef boost::edge_property_tag kind;
};
typedef boost::property<BodyEdgeDescriptorTag, robogenMessage::BodyConnection> BodyEdgeProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
		boost::no_property, BodyEdgeProperty> BodyGraph;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
		boost::no_property, BodyEdgeProperty> BodyUndirectedGraph;
typedef boost::graph_traits<BodyGraph>::edge_descriptor BodyEdge;

/**
 * Visit a body tree connecting body parts accordingly
 */
class BodyConnectionVisitor: public boost::default_bfs_visitor {

public:

	BodyConnectionVisitor(dWorldID& odeWorld,
			std::vector<boost::shared_ptr<Model> >& bodyParts,
			std::map<std::string, unsigned int>& nodeIdToPos) :
			odeWorld_(odeWorld), bodyParts_(bodyParts), nodeIdToPos_(
					nodeIdToPos) {

	}

	void tree_edge(BodyEdge v, const BodyGraph& g) const {
		boost::property_map<BodyGraph, BodyEdgeDescriptorTag>::const_type bodyConnectionMap =
				boost::get(BodyEdgeDescriptorTag(), g);

		robogenMessage::BodyConnection c = boost::get(bodyConnectionMap, v);

		// Get id positions
		std::map<std::string, unsigned int>::iterator srcNodeIt =
				nodeIdToPos_.find(c.src());
		std::map<std::string, unsigned int>::iterator dstNodeIt =
				nodeIdToPos_.find(c.dest());

		std::cout << "Connect: " << c.dest() << " with " << c.src() << "("
				<< c.destslot() << ", " << c.srcslot() << ")" << std::endl;

		RobogenUtils::connect(bodyParts_[dstNodeIt->second], c.destslot(),
				bodyParts_[srcNodeIt->second], c.srcslot(), 0, odeWorld_);

		return;
	}

private:

	dWorldID& odeWorld_;

	std::vector<boost::shared_ptr<Model> >& bodyParts_;

	std::map<std::string, unsigned int>& nodeIdToPos_;

};

// Define property of edge (contains the body connection retrieved from a robogen message)

Robot::Robot(dWorldID odeWorld, dSpaceID odeSpace) :
		odeWorld_(odeWorld), odeSpace_(odeSpace) {

}

Robot::~Robot() {

}

const std::vector<boost::shared_ptr<Sensor> >& Robot::getSensors() {
	return sensors_;
}

const std::vector<boost::shared_ptr<Motor> >& Robot::getMotors() {
	return motors_;
}

const std::vector<boost::shared_ptr<Model> >& Robot::getBodyParts() {
	return bodyParts_;
}

const boost::shared_ptr<NeuralNetwork>& Robot::getBrain() {
	return neuralNetwork_;
}
boost::shared_ptr<Model> Robot::getCoreComponent() {
	return coreComponent_;
}

bool Robot::init(const robogenMessage::Robot& robotSpec) {
	const robogenMessage::Body& body = robotSpec.body();
	const robogenMessage::Brain& brain = robotSpec.brain();
	if (!this->decodeBody(body)) {
		std::cout << "Cannot decode the body of the robot. Exiting."
				<< std::endl;
		return false;
	}
	if (!this->decodeBrain(brain)) {
		std::cout << "Cannot decode the brain of the robot. Exiting."
				<< std::endl;
		return false;
	}

	configurationFile_ = robotSpec.configuration();

	return true;
}

bool Robot::decodeBody(const robogenMessage::Body& robotBody) {

	// Will maintain all the body structure
	BodyGraph bodyTree(robotBody.part_size());

	float x = 0;
	float y = 0;
	float z = 200;
	float spacing = 200;
	int rootNode = -1;
	for (int i = 0; i < robotBody.part_size(); ++i) {

		const robogenMessage::BodyPart& bodyPart = robotBody.part(i);
		boost::shared_ptr<Model> model = RobogenUtils::createModel(bodyPart,
				odeWorld_, odeSpace_);

		if (model == NULL) {
			std::cerr << "Unrecognized body part: " << bodyPart.id()
					<< ". Exiting." << std::endl;
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

		} else if (boost::dynamic_pointer_cast<ActuatedComponent>(model)) {
			std::vector<boost::shared_ptr<Motor> > motors;
			boost::dynamic_pointer_cast<ActuatedComponent>(model)->getMotors(
					motors);
			bodyPartsToMotors_.insert(
					std::pair<int, std::vector<boost::shared_ptr<Motor> > >(i,
							motors));

			motors_.insert(motors_.end(), motors.begin(), motors.end());
		}

		if (bodyPart.root()) {
			std::cout << "Root node! " << i << std::endl;
			rootNode = i;
			coreComponent_ = model;
		}

		std::cout << "Node: " << i << " id: " << bodyPart.id() << " type: "
				<< bodyPart.type() << std::endl;

		x += spacing;
		y += spacing;

	}

	// Look for the root node and modify its position to the origin
	for (unsigned int i = 0; i < bodyParts_.size(); ++i) {
		if ((int) i == rootNode) {
			bodyParts_[i]->setRootPosition(osg::Vec3(0, 0, 0));
		}
	}

	// We now have to connect them properly
	// Decode the body connections
	for (int i = 0; i < robotBody.connection_size(); ++i) {
		const robogenMessage::BodyConnection& connection = robotBody.connection(
				i);

		const std::map<std::string, unsigned int>::iterator srcNodeIt =
				bodyPartsMap_.find(connection.src());
		const std::map<std::string, unsigned int>::iterator dstNodeIt =
				bodyPartsMap_.find(connection.dest());

		if (srcNodeIt == bodyPartsMap_.end()
				|| dstNodeIt == bodyPartsMap_.end()) {
			std::cout
					<< "The source and/or destination nodes are not in the body tree ("
					<< connection.src() << ", " << connection.dest() << ")"
					<< std::endl;
			return false;
		}

		std::cout << "Edge: " << connection.src() << " -> " << connection.dest()
				<< std::endl;

		boost::add_edge(srcNodeIt->second, dstNodeIt->second,
				BodyEdgeProperty(connection), bodyTree);
	}

	// Let's run first a connectivity check
	std::vector<int> component(robotBody.part_size());

	BodyUndirectedGraph bodyTreeUndirected;
	boost::copy_graph(bodyTree, bodyTreeUndirected);

	int numComponents = boost::connected_components(bodyTreeUndirected,
			&component[0]);
	if (numComponents != 1) {
		std::cout
				<< "The robot body has some disconnected component (ConnComponents: "
				<< numComponents << ")" << std::endl;
		return false;
	}

	BodyConnectionVisitor vis(odeWorld_, bodyParts_, bodyPartsMap_);
	boost::breadth_first_search(bodyTree, rootNode, boost::visitor(vis));

	std::cout << "Sensors: " << sensors_.size() << ", motors: "
			<< motors_.size() << std::endl;

	return true;

}

bool Robot::decodeBrain(const robogenMessage::Brain& robotBrain) {

	unsigned int nInputs = 0;
	unsigned int nOutputs = 0;

	std::map<std::string, bool> isNeuronInput;

	std::map<std::string, unsigned int> inputNeuronIds;
	std::vector<unsigned int> brainInputToBodyPart;
	std::vector<unsigned int> brainInputToIoId;

	std::map<std::string, unsigned int> outputNeuronIds;
	std::vector<unsigned int> brainOutputToBodyPart;
	std::vector<unsigned int> brainOutputToIoId;

	float weight[MAX_INPUT_NEURONS * MAX_OUTPUT_NEURONS
			+ MAX_OUTPUT_NEURONS * MAX_OUTPUT_NEURONS];
	float bias[MAX_OUTPUT_NEURONS];
	float gain[MAX_OUTPUT_NEURONS];

	// Fill it with zeros
	for (unsigned int i = 0; i < MAX_INPUT_NEURONS; ++i) {
		for (unsigned int j = 0; j < MAX_OUTPUT_NEURONS; ++j) {
			weight[i * MAX_OUTPUT_NEURONS + j] = 0;
		}
	}
	for (unsigned int i = 0; i < MAX_OUTPUT_NEURONS; ++i) {
		for (unsigned int j = 0; j < MAX_OUTPUT_NEURONS; ++j) {
			weight[MAX_INPUT_NEURONS * MAX_OUTPUT_NEURONS
					+ i * MAX_OUTPUT_NEURONS + j] = 0;
		}
	}

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
			brainOutputToBodyPart.push_back(bodyPartId->second);
			brainOutputToIoId.push_back(neuron.ioid());
			outputNeuronIds.insert(
					std::pair<std::string, unsigned int>(neuron.id(),
							nOutputs));
			isNeuronInput.insert(
					std::pair<std::string, bool>(neuron.id(), false));

			bias[nOutputs] = neuron.biasweight();
			gain[nOutputs] = 1;

			nOutputs++;

		} else {
			std::cout << "Unsupported layer for neuron " << i << std::endl;
			return false;
		}

	}

	if (nInputs > MAX_INPUT_NEURONS) {
		std::cout << "The number of input neurons(" << nInputs
				<< ") is greater than the maximum allowed one ("
				<< MAX_INPUT_NEURONS << ")" << std::endl;
		return false;
	}

	if (nOutputs > MAX_OUTPUT_NEURONS) {
		std::cout << "The number of output neurons(" << nOutputs
				<< ") is greater than the maximum allowed one ("
				<< MAX_OUTPUT_NEURONS << ")" << std::endl;
		return false;
	}

	// Order sensors/actuators accordingly, for faster access
	std::vector<boost::shared_ptr<Sensor> > orderedSensors;
	std::vector<boost::shared_ptr<Motor> > orderedMotors;

	orderedSensors.resize(nInputs);
	orderedMotors.resize(nOutputs);

	for (unsigned int i = 0; i < nInputs; ++i) {

		// Find the sensor
		std::map<unsigned int, std::vector<boost::shared_ptr<Sensor> > >::iterator it =
				bodyPartsToSensors_.find(brainInputToBodyPart[i]);
		std::vector<boost::shared_ptr<Sensor> > curSensors = it->second;

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

	std::cout << "OSensors: " << orderedSensors.size() << ", Omotors: "
			<< orderedMotors.size() << std::endl;

	sensors_ = orderedSensors;
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
			sourceNeuronPos = outputNeuronIds[connection.src()];
		}

		// Cannot be an input neuron
		int destNeuronPos = outputNeuronIds[connection.dest()];

		if (isSourceInputIt->second == true) {

			weight[sourceNeuronPos * nOutputs + destNeuronPos] =
					connection.weight();
		} else {

			weight[nInputs * nOutputs + sourceNeuronPos * nOutputs
					+ destNeuronPos] = connection.weight();
		}

	}

	::initNetwork(neuralNetwork_.get(), nInputs, nOutputs, &weight[0], &bias[0],
			&gain[0]);

	return true;

}

void Robot::translateRobot(const osg::Vec3& translation) {

	for (unsigned int i = 0; i < this->bodyParts_.size(); ++i) {
		this->bodyParts_[i]->translateRootPosition(translation);
	}

}

void Robot::getBB(double& minX, double& maxX, double& minY, double& maxY,
		double& minZ, double& maxZ) {

	minX = 100000000;
	minY = 100000000;
	minZ = 100000000;
	maxX = -100000000;
	maxY = -100000000;
	maxZ = -100000000;

	std::vector<dBodyID> bodies;
	for (unsigned int i = 0; i < this->bodyParts_.size(); ++i) {
		std::vector<dBodyID> tempBodies = this->bodyParts_[i]->getBodies();
		bodies.insert(bodies.begin(), tempBodies.begin(), tempBodies.end());
	}

	for (unsigned int i = 0; i < bodies.size(); ++i) {

		dGeomID curBodyGeom = dBodyGetFirstGeom(bodies[i]);

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

	}
}

const std::string& Robot::getConfigurationFile() const {
	return configurationFile_;
}

}
