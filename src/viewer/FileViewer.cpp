/*
 * @(#) FileViewer.cpp   1.0   Mar 5, 2013
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
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include "config/ConfigurationReader.h"
#include "config/RobogenConfig.h"
#include "scenario/Scenario.h"
#include "scenario/ScenarioFactory.h"
#include "utils/network/ProtobufPacket.h"
#include "utils/network/TcpSocket.h"
#include "utils/RobogenCollision.h"
#include "utils/RobogenUtils.h"
#include "viewer/KeyboardHandler.h"
#include "Models.h"
#include "RenderModels.h"
#include "Robogen.h"
#include "Robot.h"
#include "robogen.pb.h"

using namespace robogen;

// ODE World
dWorldID odeWorld;

// Container for collisions
dJointGroupID odeContactGroup;

bool interrupted;

/**
 * Decodes a robot saved on file and visualize it
 */
int main(int argc, char *argv[]) {

	if (argc < 3) {
		std::cout
				<< "Please provide a file containing the robot description as input and the corresponding simulation configuration file '"
				<< std::string(argv[0]) << " robot.dat configuration.conf'"
				<< std::endl;
		return EXIT_FAILURE;
	}

	// Decode configuration file
	boost::shared_ptr<RobogenConfig> configuration =
			ConfigurationReader::parseConfigurationFile(std::string(argv[2]));
	if (configuration == NULL) {
		std::cout << "Problems parsing the configuration file. Quit."
				<< std::endl;
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// ODE Initialization
	// ---------------------------------------

	dInitODE();

	// Create ODE world
	odeWorld = dWorldCreate();

	// Set gravity [mm/s]
	dWorldSetGravity(odeWorld, 0, 0, -9.81);

	dWorldSetERP(odeWorld, 0.4);
	dWorldSetCFM(odeWorld, 0.2);

	// Create collision world
	dSpaceID odeSpace = dSimpleSpaceCreate(0);

	// Create contact group
	odeContactGroup = dJointGroupCreate(0);

	// ---------------------------------------
	// OSG Initialization
	// ---------------------------------------

	//Creating the viewer
	osgViewer::Viewer viewer;

	viewer.setUpViewInWindow(300, 300, 1280, 1024);

	osg::ref_ptr<KeyboardHandler> keyboardEvent(new KeyboardHandler());

	viewer.addEventHandler(keyboardEvent.get());

	// Camera
	osg::ref_ptr<osg::Camera> camera = viewer.getCamera();

	//Creating the root node
	osg::ref_ptr<osg::Group> root(new osg::Group);

	// ---------------------------------------
	// Robot decoding
	// ---------------------------------------
	std::ifstream robotFile(argv[1], std::ios::binary);
	if (!robotFile.is_open()) {
		std::cout << "Cannot open " << std::string(argv[1]) << ". Quit."
				<< std::endl;
		return EXIT_FAILURE;
	}

	ProtobufPacket<robogenMessage::Robot> robogenPacket;

	robotFile.seekg(0, robotFile.end);
	unsigned int packetSize = robotFile.tellg();
	robotFile.seekg(0, robotFile.beg);

	std::vector<unsigned char> packetBuffer;
	packetBuffer.resize(packetSize);
	robotFile.read((char*) &packetBuffer[0], packetSize);
	robogenPacket.decodePayload(packetBuffer);

	// ---------------------------------------
	// Setup environment
	// ---------------------------------------

	boost::shared_ptr<Scenario> scenario = ScenarioFactory::createScenario(
			configuration);
	if (scenario == NULL) {
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// Generate Robot
	// ---------------------------------------
	boost::shared_ptr<Robot> robot(new Robot(odeWorld, odeSpace));
	if (!robot->init(*robogenPacket.getMessage().get())) {
		std::cout << "Problems decoding the robot. Quit." << std::endl;
		return EXIT_FAILURE;
	}

	// Register sensors
	std::vector<boost::shared_ptr<Sensor> > sensors = robot->getSensors();
	std::vector<boost::shared_ptr<TouchSensor> > touchSensors;
	for (unsigned int i = 0; i < sensors.size(); ++i) {
		if (boost::dynamic_pointer_cast<TouchSensor>(sensors[i])) {
			touchSensors.push_back(
					boost::dynamic_pointer_cast<TouchSensor>(sensors[i]));
		}
	}

	// Register robot motors
	std::vector<boost::shared_ptr<Motor> > motors = robot->getMotors();

	std::cout << "S: " << sensors.size() << std::endl;
	std::cout << "M: " << motors.size() << std::endl;

	// Register brain and body parts
	boost::shared_ptr<NeuralNetwork> neuralNetwork = robot->getBrain();
	std::vector<boost::shared_ptr<Model> > bodyParts = robot->getBodyParts();

	// Initialize scenario
	if (!scenario->init(odeWorld, odeSpace, robot)) {
		std::cout << "Cannot initialize scenario. Quit." << std::endl;
		return EXIT_FAILURE;
	}

	// Setup environment
	boost::shared_ptr<Environment> env = scenario->getEnvironment();

	if (!scenario->setupSimulation()) {
		std::cout << "Cannot setup scenario. Quit." << std::endl;
		return EXIT_FAILURE;
	}

	std::vector<boost::shared_ptr<RenderModel> > renderModels;
	for (unsigned int i = 0; i < bodyParts.size(); ++i) {

		boost::shared_ptr<RenderModel> renderModel =
				RobogenUtils::createRenderModel(bodyParts[i]);
		if (renderModel == NULL) {
			std::cout << "Cannot create a render model for model " << i
					<< std::endl;
			return EXIT_FAILURE;
		}

		renderModel->initRenderModel();
		renderModels.push_back(renderModel);
		root->addChild(renderModels[i]->getRootNode());
	}

	// Terrain render model
	boost::shared_ptr<TerrainRender> terrainRender(
			new TerrainRender(scenario->getTerrain()));
	root->addChild(terrainRender->getRootNode());

	// Obstacles render model
	const std::vector<boost::shared_ptr<BoxObstacle> >& obstacles =
			scenario->getObstacles();
	for (unsigned int i = 0; i < obstacles.size(); ++i) {
		boost::shared_ptr<BoxObstacleRender> obstacleRender(
				new BoxObstacleRender(obstacles[i]));
		root->addChild(obstacleRender->getRootNode());
	}

	// ---------------------------------------
	// Setup OSG viewer
	// ---------------------------------------

	viewer.setSceneData(root.get());

	viewer.realize();

	if (!viewer.getCameraManipulator()
			&& viewer.getCamera()->getAllowEventFocus()) {
		viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	}

	viewer.setReleaseContextAtEndOfFrameHint(false);

	// ---------------------------------------
	// ROBOGEN Simulation
	// ---------------------------------------

	int count = 0;
	double deltaSecs = 0;
	double t = 0;
	osg::Timer_t prevTime = osg::Timer::instance()->tick();
	while (t < configuration->getSimulationTime() && !viewer.done()) {

		viewer.frame();

		if (!keyboardEvent->isPaused()) {

			double step = configuration->getTimeStepLength();
			const osg::Timer_t now = osg::Timer::instance()->tick();
			deltaSecs += osg::Timer::instance()->delta_s(prevTime, now);
			prevTime = now;

			while (deltaSecs > step) {

				deltaSecs -= step;
				t += step;

				if ((count++) % 100 == 0) {
					std::cout << "Step!" << count << std::endl;
				}

				// Prepare touch sensors for collision detection
				for (unsigned int i = 0; i < touchSensors.size(); ++i) {
					touchSensors[i]->reset();
				}

				// Collision detection
				dSpaceCollide(odeSpace, 0, odeCollisionCallback);

				// Step the world by one timestep
				dWorldStep(odeWorld, step);

				// Empty contact groups used for collisions handling
				dJointGroupEmpty(odeContactGroup);

				float networkInput[MAX_INPUT_NEURONS];
				float networkOutputs[MAX_OUTPUT_NEURONS];

				// Feed neural network
				for (unsigned int i = 0; i < bodyParts.size(); ++i) {
					if (boost::dynamic_pointer_cast<PerceptiveComponent>(
							bodyParts[i])) {
						boost::dynamic_pointer_cast<PerceptiveComponent>(
								bodyParts[i])->updateSensors(env);
					}
				}

				for (unsigned int i = 0; i < sensors.size(); ++i) {

					if (boost::dynamic_pointer_cast<TouchSensor>(sensors[i])) {
						networkInput[i] = boost::dynamic_pointer_cast<
								TouchSensor>(sensors[i])->read();
					} else if (boost::dynamic_pointer_cast<LightSensor>(
							sensors[i])) {
						networkInput[i] = boost::dynamic_pointer_cast<
								LightSensor>(sensors[i])->read(
								env->getLightSources(), env->getAmbientLight());
					} else if (boost::dynamic_pointer_cast<SimpleSensor>(
							sensors[i])) {
						networkInput[i] = boost::dynamic_pointer_cast<
								SimpleSensor>(sensors[i])->read();
					}
				}
				::feed(neuralNetwork.get(), &networkInput[0]);

				// Step the neural network
				::step(neuralNetwork.get());

				// Fetch the neural network ouputs
				::fetch(neuralNetwork.get(), &networkOutputs[0]);

				// Send control to motors
				for (unsigned int i = 0; i < motors.size(); ++i) {
					if (boost::dynamic_pointer_cast<ServoMotor>(motors[i])) {

						boost::shared_ptr<ServoMotor> motor =
								boost::dynamic_pointer_cast<ServoMotor>(
										motors[i]);

						if (motor->isVelocityDriven()) {
							motor->setVelocity(networkOutputs[i]);
						} else {
							motor->setPosition(
									osg::inDegrees(networkOutputs[i]));
						}
					}
				}

				if (!scenario->afterSimulationStep()) {
					std::cout
							<< "Cannot execute scenario after simulation step. Quit."
							<< std::endl;
					return EXIT_FAILURE;
				}

			}

		} else {
			prevTime = osg::Timer::instance()->tick();
		}

	}

	if (!scenario->endSimulation()) {
		std::cout << "Cannot complete scenario. Quit." << std::endl;
		return EXIT_FAILURE;
	}

	// ---------------------------------------
	// Compute fitness
	// ---------------------------------------

	double fitness = scenario->getFitness();
	std::cout << "Fitness for the current solution: " << fitness << std::endl;

	// ---------------------------------------
	// Simulator finalization
	// ---------------------------------------

	// Destroy ODE space
	dSpaceDestroy(odeSpace);

	// Destroy ODE world
	dWorldDestroy(odeWorld);

	// Destroy the ODE engine
	dCloseODE();

	return EXIT_SUCCESS;
}
