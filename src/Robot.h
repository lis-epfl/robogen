/*
 * @(#) Robot.h   1.0   Mar 4, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
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
#ifndef ROBOGEN_ROBOT_H_
#define ROBOGEN_ROBOT_H_

#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <map>
#include <vector>

#include "Robogen.h"
#include "robogen.pb.h"
#include "model/Connection.h"

extern "C" {
#include "brain/NeuralNetwork.h"
}

namespace robogen {

class Model;
class Motor;
class Sensor;

struct BodyEdgeDescriptorTag {
	typedef boost::edge_property_tag kind;
};
typedef boost::property<BodyEdgeDescriptorTag, boost::shared_ptr<Connection> >
BodyEdgeProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
		boost::no_property, BodyEdgeProperty> BodyGraph;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
		boost::no_property, BodyEdgeProperty> BodyUndirectedGraph;
typedef boost::graph_traits<BodyGraph>::edge_descriptor BodyEdge;

/**
 * A ROBOGEN Robot
 */
class Robot {

public:
	/**
	 * Error-less constructor.
	 */
	Robot();

	/**
	 * Initializes a Robogen robot from a robogen message
	 * @param odeWorld
	 * @param odeSpace
	 * @param robotSpec
	 */
	bool init(dWorldID odeWorld, dSpaceID odeSpace,
			const robogenMessage::Robot& robotSpec,
			bool printInfo=false);

	/**
	 * Destructor
	 */
	virtual ~Robot();

	/**
	 *  @return  the body parts of which the robot is composed of
	 */
	const std::vector<boost::shared_ptr<Model> >& getBodyParts();

	/**
	 * @return the neural network that controls the robot
	 */
	const boost::shared_ptr<NeuralNetwork>& getBrain() const;

	/**
	 * @return the sensors of the robot
	 */
	const std::vector<boost::shared_ptr<Sensor> >& getSensors() const;

	/**
	 * @return the motors of the robot
	 */
	const std::vector<boost::shared_ptr<Motor> >& getMotors();

	/**
	 * @return the core component of the robot
	 */
	boost::shared_ptr<Model> getCoreComponent();

	/**
	 * Translate the robot
	 * @param pos Amount of translation
	 */
	void translateRobot(const osg::Vec3& translation);

	/**
	 * Rotate the robot
	 * @param rot rotation Quaternion
	 */
	void rotateRobot(const osg::Quat &rot);

	/**
	 * Returns the robot bounding box
	 */
	void getBB(double& minX, double& maxX, double& minY, double& maxY,
			double& minZ, double& maxZ);

	/**
	 * @return the robot ID
	 */
	int getId() const;

	/**
	 *
	 */
	boost::shared_ptr<Model> getBodyPart(std::string id);

	/**
	 * @return message that generated the robot
	 */
	const robogenMessage::Robot& getMessage();

	const std::vector<boost::shared_ptr<Connection> >& getBodyConnections()
			const;
	void traverseBody(const std::vector<boost::shared_ptr<Model> >,
			const std::vector<boost::shared_ptr<Connection> >);
	int getRoot();
private:

	/**
	 * Decodes the body of the robot
	 * @param robotBody
	 * @return true if the operation completed successfully
	 */
	bool decodeBody(const robogenMessage::Body& robotBody);

	/**
	 * Decodes the brain of the robot
	 * @param robotBrain
	 * @return true if the operation completed successfully
	 */
	bool decodeBrain(const robogenMessage::Brain& robotBrain);

	/**
	 * Connects all body parts to the root part
	 */
	void reconnect();

	/**
	 * ODE physics world
	 */
	dWorldID odeWorld_;

	/**
	 * ODE collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * Robot body parts
	 */
	std::vector<boost::shared_ptr<Model> > bodyParts_;

	/**
	 * Connections between the body parts.
	 */
	std::vector<boost::shared_ptr<Connection> > bodyConnections_;

	/**
	 * Mapping from body part to associated sensors
	 */
	std::map<unsigned int, std::vector<boost::shared_ptr<Sensor> > > bodyPartsToSensors_;

	/**
	 * Mapping from body part to associated motors
	 */
	std::map<unsigned int, std::vector<boost::shared_ptr<Motor> > > bodyPartsToMotors_;

	/**
	 * Robot sensors
	 */
	std::vector<boost::shared_ptr<Sensor> > sensors_;

	/**
	 * Robot motors
	 */
	std::vector<boost::shared_ptr<Motor> > motors_;

	/**
	 * Neural network
	 */
	boost::shared_ptr<NeuralNetwork> neuralNetwork_;

	/**
	 * Maps the identifier of a body part with the body part in the bodyParts_ vector
	 */
	std::map<std::string, unsigned int> bodyPartsMap_;

	/**
	 * The core component of the robot
	 */
	boost::shared_ptr<Model> coreComponent_;

	/**
	 * Robot ID
	 */
	int id_;

	/**
	 * Root part index of the robot
	 */
	int rootNode_;

	/**
	 * Boost graph of body tree.
	 */
	boost::shared_ptr<BodyGraph> bodyTree_;

	/**
	 * Joint group of connections between parts
	 */
	dJointGroupID connectionJointGroup_;

	/**
	 * Contains the robot message
	 */

	const robogenMessage::Robot* robotMessage_;

	bool printInfo_;
};

}

#endif /* ROBOGEN_ROBOT_H_ */
