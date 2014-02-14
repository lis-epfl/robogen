/*
 * @(#) FileViewer.cpp   1.0   Mar 5, 2013
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
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/format.hpp>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgDB/WriteFile>

#include "config/ConfigurationReader.h"
#include "config/RobogenConfig.h"
#include "evolution/representation/RobotRepresentation.h"
#include "scenario/Scenario.h"
#include "scenario/ScenarioFactory.h"
#include "utils/network/ProtobufPacket.h"
#include "utils/network/TcpSocket.h"
#include "utils/json2pb/json2pb.h"
#include "utils/RobogenCollision.h"
#include "utils/RobogenUtils.h"
#include "viewer/KeyboardHandler.h"
#include "viewer/FileViewerLog.h"
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

class SnapImageDrawCallback : public osg::Camera::DrawCallback {
public:

	SnapImageDrawCallback()
	{
		_snapImageOnNextFrame = false;
	}

	void setFileName(const std::string& filename) { _filename = filename; }
	const std::string& getFileName() const { return _filename; }

	void setSnapImageOnNextFrame(bool flag) { _snapImageOnNextFrame = flag; }
	bool getSnapImageOnNextFrame() const { return _snapImageOnNextFrame; }

	virtual void operator () (const osg::Camera& camera) const
	{
		if (!_snapImageOnNextFrame) return;

		int x,y,width,height;
		x = camera.getViewport()->x();
		y = camera.getViewport()->y();
		width = camera.getViewport()->width();
		height = camera.getViewport()->height();

		osg::ref_ptr<osg::Image> image = new osg::Image;
		image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);

		if (osgDB::writeImageFile(*image,_filename))
		{
		std::cout << "Saved screen image to `"<<_filename<<"`"<< std::endl;
		}

		_snapImageOnNextFrame = false;
	}

protected:

	std::string _filename;
	mutable bool _snapImageOnNextFrame;


};

/**
 * Decodes a robot saved on file and visualize it
 */
int main(int argc, char *argv[]) {

	if (argc < 3) {
		std::cout << "Please provide a file containing the robot description as"
				" input and the corresponding simulation configuration file. "
				<< std::endl << "For example: " << std::string(argv[0])
				<< " robot.dat configuration.conf'" << std::endl
				<< "You can also select the starting position by "
						"appending an integer 1..n to the command" << std::endl
				<< "To save frames to file (for video rendering), use option "
				<< "--record <N, INTEGER> <DIR, STRING>, "
				<< "to save every Nth frame in directory DIR"
				<< std::endl
				<< "To generate output files: sensor logs and Arduino files, "
				<< "use option --output <DIR_POSTFIX, STRING>"
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

	// verify desired start position is specified in configuration
	unsigned int desiredStart = 0;
	unsigned int recordFrequency = 0;
	bool recording = false;
	char *recordDirectoryName;

	bool writeLog = false;
	char *outputDirectoryName;


	int currentArg = 3;
	if (argc >= 4 && !boost::starts_with(argv[3], "--")) {
		std::stringstream ss(argv[3]);
		currentArg++;
		ss >> desiredStart;
		--desiredStart; // -- accounts for parameter being 1..n
		if (ss.fail()) {
			std::cout << "Specified desired starting position \"" << argv[3]
					<< "\" is not an integer. Aborting..." << std::endl;
			return EXIT_FAILURE;
		}
		if (desiredStart
				>= configuration->getStartingPos()->getStartPosition().size()) {
			std::cout << "Specified desired starting position " << argv[3]
					<< " does not index a starting position. Aborting..."
					<< std::endl;
			return EXIT_FAILURE;
		}
	}

	for (; currentArg<argc; currentArg++) {
		if (std::string("--record").compare(argv[currentArg]) == 0) {
			if (argc < (currentArg + 3)) {
				std::cout << "In order to record frames, must provide frame "
						<< "frequency and target directory."
						<< std::endl;
						return EXIT_FAILURE;
			}
			recording = true;
			currentArg++;
			std::stringstream ss(argv[currentArg]);
			ss >> recordFrequency;
			if (ss.fail()) {
				std::cout << "Specified record frequency \"" << argv[currentArg]
						<< "\" is not an integer. Aborting..." << std::endl;
				return EXIT_FAILURE;
			}
			currentArg++;

			recordDirectoryName = argv[currentArg];
			boost::filesystem::path recordDirectory(recordDirectoryName);

			if (recording && !boost::filesystem::is_directory(recordDirectory) ) {
				boost::filesystem::create_directories(recordDirectory);
			}

		}
		if (std::string("--output").compare(argv[currentArg]) == 0) {
			if (argc < (currentArg + 2)) {
				std::cout << "In order to write output files, must provide "
										<< "directory postfix."
										<< std::endl;
										return EXIT_FAILURE;

			}
			writeLog = true;
			currentArg++;

			outputDirectoryName = argv[currentArg];
		}

	}




	// ---------------------------------------
	// ODE Initialization
	// ---------------------------------------

	dInitODE();

	// Create ODE world
	odeWorld = dWorldCreate();

	// Set gravity [mm/s]
	dWorldSetGravity(odeWorld, 0, 0, -9.81);

	dWorldSetERP(odeWorld, 0.1);
	dWorldSetCFM(odeWorld, 10e-6);

	// Create collision world
	//dSpaceID odeSpace = dSimpleSpaceCreate(0);
	dSpaceID odeSpace = dHashSpaceCreate(0);

	// Create contact group
	odeContactGroup = dJointGroupCreate(0);

	// ---------------------------------------
	// OSG Initialization
	// ---------------------------------------

	//Creating the viewer
	osgViewer::Viewer viewer;

	viewer.setUpViewInWindow(200, 200, 800, 600);

	osg::ref_ptr<KeyboardHandler> keyboardEvent(new KeyboardHandler());

	viewer.addEventHandler(keyboardEvent.get());

	// Camera
	osg::ref_ptr<osg::Camera> camera = viewer.getCamera();

	//Creating the root node
	osg::ref_ptr<osg::Group> root(new osg::Group);

	// ---------------------------------------
	// Robot decoding
	// ---------------------------------------
	robogenMessage::Robot robotMessage;
	std::string robotFileString(argv[1]);

	if (boost::filesystem::path(argv[1]).extension().string().compare(".dat") == 0) {

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
		robotMessage = *robogenPacket.getMessage().get();

	} else if (boost::filesystem::path(argv[1]).extension().string().compare(".txt") == 0) {

		RobotRepresentation robot;
		if (!robot.init(argv[1])) {
			std::cout << "Failed interpreting robot text file!" << std::endl;
			return EXIT_FAILURE;
		}
		robotMessage = robot.serialize();

	} else if (boost::filesystem::path(argv[1]).extension().string().compare(".json") == 0) {

		std::ifstream robotFile(argv[1]);
		if (!robotFile.is_open()) {
			std::cout << "Cannot open " << std::string(argv[1]) << ". Quit."
					<< std::endl;
			return EXIT_FAILURE;
		}

		robotFile.seekg(0, robotFile.end);
		unsigned int packetSize = robotFile.tellg();
		robotFile.seekg(0, robotFile.beg);

		std::vector<unsigned char> packetBuffer;
		packetBuffer.resize(packetSize);
		robotFile.read((char*) &packetBuffer[0], packetSize);

		json2pb(robotMessage, (char*) &packetBuffer[0], packetSize);

	} else {
		std::cout << "File extension of provided robot file could not be "
				"resolved. Use .dat or .json for robot messages and .txt for "
				"robot text files" << std::endl;
		return EXIT_FAILURE;
	}

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
	boost::shared_ptr<Robot> robot(new Robot);
	if (!robot->init(odeWorld, odeSpace, robotMessage)) {
		std::cout << "Problems decoding the robot. Quit." << std::endl;
		return EXIT_FAILURE;
	}

	// Register sensors
	std::vector<boost::shared_ptr<Sensor> > sensors = robot->getSensors();

	// Register motors
	std::vector<boost::shared_ptr<Motor> > motors = robot->getMotors();

	std::cout << "S: " << sensors.size() << std::endl;
	std::cout << "M: " << motors.size() << std::endl;

	// Register brain and body parts
	boost::shared_ptr<NeuralNetwork> neuralNetwork = robot->getBrain();
	std::vector<boost::shared_ptr<Model> > bodyParts = robot->getBodyParts();

	// Initialize scenario
	scenario->setStartingPosition(desiredStart);
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

		if (!renderModel->initRenderModel()) {
			std::cout
					<< "Cannot initialize a render model for one of the components. "
					<< std::endl
					<< "Please check that the models/ folder is in the same folder of this executable."
					<< std::endl;
		}
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

	// Light render model
	const std::vector<boost::shared_ptr<LightSource> >& lightSources =
			scenario->getEnvironment()->getLightSources();
	for (unsigned int i = 0; i < lightSources.size(); ++i) {
		boost::shared_ptr<LightSourceRender> lightSourceRender(
				new LightSourceRender(lightSources[i], root));
		root->addChild(lightSourceRender->getRootNode());
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

	if (recording) {
		osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = new
													SnapImageDrawCallback();
		viewer.getCamera()->setPostDrawCallback (snapImageDrawCallback.get());
	}

	// ---------------------------------------
	// Set up log files
	// ---------------------------------------

	boost::shared_ptr<FileViewerLog> log(new FileViewerLog);
	if (writeLog) {
		if (!log->init(std::string(argv[1]), std::string(argv[2]),
				configuration->getObstacleFile(),
				configuration->getStartPosFile(),
				robot,
				std::string(outputDirectoryName)
				)) {
			std::cout << "Problem initializing log!" << std::endl;
			return EXIT_FAILURE;
		}
	}
	// ---------------------------------------
	// ROBOGEN Simulation
	// ---------------------------------------

	std::cout << "Press P to pause/unpause the simulation." << std::endl;
	std::cout << "Press Q to quit the visualizer." << std::endl;

	int count = 0;
	double t = 0;
	unsigned int frameCount = 0;

	while (!viewer.done() && !keyboardEvent->isQuit()) {

		viewer.frame();

		if (t < configuration->getSimulationTime()
				&& !keyboardEvent->isPaused()) {

			double step = configuration->getTimeStepLength();
			if (recording && count % recordFrequency == 0) {
				osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback =
						dynamic_cast<SnapImageDrawCallback*>
						(viewer.getCamera()->getPostDrawCallback());

				if(snapImageDrawCallback.get()) {
					std::stringstream ss;
					ss << recordDirectoryName << "/" <<
							boost::format("%|04|")%frameCount << ".jpg";
					snapImageDrawCallback->setFileName(ss.str());
					snapImageDrawCallback->setSnapImageOnNextFrame(true);
					frameCount++;
				}
			}


			if ((count++) % 500 == 0) {
				std::cout << "." << std::flush;
			}

			// Collision detection
			dSpaceCollide(odeSpace, 0, odeCollisionCallback);

			// Step the world by one timestep
			dWorldStep(odeWorld, step);

			// Empty contact groups used for collisions handling
			dJointGroupEmpty(odeContactGroup);

			float networkInput[MAX_INPUT_NEURONS];
			float networkOutput[MAX_OUTPUT_NEURONS];

			// Elapsed time since last call
			env->setTimeElapsed(step);

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
					networkInput[i] = boost::dynamic_pointer_cast<TouchSensor>(
							sensors[i])->read();
				} else if (boost::dynamic_pointer_cast<LightSensor>(
						sensors[i])) {
					networkInput[i] = boost::dynamic_pointer_cast<LightSensor>(
							sensors[i])->read(env->getLightSources(),
							env->getAmbientLight());
				} else if (boost::dynamic_pointer_cast<SimpleSensor>(
						sensors[i])) {
					networkInput[i] = boost::dynamic_pointer_cast<SimpleSensor>(
							sensors[i])->read();
				}
			}

			if (writeLog) {
				log->logSensors(networkInput, sensors.size());
			}

			::feed(neuralNetwork.get(), &networkInput[0]);

			// Step the neural network
			::step(neuralNetwork.get());

			// Fetch the neural network ouputs
			::fetch(neuralNetwork.get(), &networkOutput[0]);

			// Send control to motors
			for (unsigned int i = 0; i < motors.size(); ++i) {
				if (boost::dynamic_pointer_cast<ServoMotor>(motors[i])) {

					boost::shared_ptr<ServoMotor> motor =
							boost::dynamic_pointer_cast<ServoMotor>(motors[i]);

					if (motor->isVelocityDriven()) {
						motor->setVelocity(networkOutput[i]);
					} else {
						motor->setPosition(networkOutput[i]);
					}
				}
			}

			if(writeLog) {
				log->logMotors(networkOutput, motors.size());
			}

			if (!scenario->afterSimulationStep()) {
				std::cout
						<< "Cannot execute scenario after simulation step. Quit."
						<< std::endl;
				return EXIT_FAILURE;
			}

			// log trajectory
			if(writeLog) {
				log->logPosition(
					scenario->getRobot(
							)->getCoreComponent()->getRootPosition());
			}
			t += step;

		} /* If doing something */

	} /* while (!viewer.done() && !keyboardEvent->isQuit()) */

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

	// Destroy robot (because of associated ODE joint group)
	robot.reset();
	// has shared pointer in scenario, so destroy that too
	scenario->prune();

	// Destroy ODE space
	dSpaceDestroy(odeSpace);

	// Destroy ODE world
	dWorldDestroy(odeWorld);

	// Destroy the ODE engine
	dCloseODE();

	return EXIT_SUCCESS;
}
