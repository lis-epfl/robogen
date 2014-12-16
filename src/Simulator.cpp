/*
 * @(#) FileViewer.cpp   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Joshua Auerbach
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

#include "Simulator.h"
#include "utils/RobogenCollision.h"
#include "viewer/Viewer.h"
#include "Models.h"
#include "Robot.h"


// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen{

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage,
		Viewer *viewer) {
	boost::shared_ptr<FileViewerLog> log;
	return runSimulations(scenario, configuration,
			robotMessage, viewer, false, log);
}

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage, Viewer *viewer,
		bool onlyOnce, boost::shared_ptr<FileViewerLog> log) {

	bool accelerationCapExceeded = false;

	while (scenario->remainingTrials() && (!accelerationCapExceeded)) {

		// ---------------------------------------
		// Simulator initialization
		// ---------------------------------------

		dInitODE();

		// Create ODE world
		odeWorld = dWorldCreate();

		// Set gravity [mm/s]
		dWorldSetGravity(odeWorld, 0, 0, -9.81);

		dWorldSetERP(odeWorld, 0.1);
		dWorldSetCFM(odeWorld, 10e-6);

		// Create collision world
		dSpaceID odeSpace = dSimpleSpaceCreate(0);

		// Create contact group
		odeContactGroup = dJointGroupCreate(0);

		// ---------------------------------------
		// Generate Robot
		// ---------------------------------------
		boost::shared_ptr<Robot> robot(new Robot);
		if (!robot->init(odeWorld, odeSpace, robotMessage)) {
			std::cout << "Problems decoding the robot. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}


		if (log) {
			if (!log->init(robot, configuration)) {
				std::cout << "Problem initializing log!" << std::endl;
				return SIMULATION_FAILURE;
			}
		}

		std::cout << "Evaluating individual " << robot->getId()
				<< ", trial: " << scenario->getCurTrial()
				<< std::endl;

		// Register sensors
		std::vector<boost::shared_ptr<Sensor> > sensors =
				robot->getSensors();
		std::vector<boost::shared_ptr<TouchSensor> > touchSensors;
		for (unsigned int i = 0; i < sensors.size(); ++i) {
			if (boost::dynamic_pointer_cast<TouchSensor>(
					sensors[i])) {
				touchSensors.push_back(
						boost::dynamic_pointer_cast<TouchSensor>(
								sensors[i]));
			}
		}

		// Register robot motors
		std::vector<boost::shared_ptr<Motor> > motors =
				robot->getMotors();

		// Register brain and body parts
		boost::shared_ptr<NeuralNetwork> neuralNetwork =
				robot->getBrain();
		std::vector<boost::shared_ptr<Model> > bodyParts =
				robot->getBodyParts();

		// Initialize scenario
		if (!scenario->init(odeWorld, odeSpace, robot)) {
			std::cout << "Cannot initialize scenario. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}

		// Setup environment
		boost::shared_ptr<Environment> env =
				scenario->getEnvironment();

		if (!scenario->setupSimulation()) {
			std::cout << "Cannot setup scenario. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}

		bool visualize = (viewer != NULL);
		if(visualize && !viewer->configureScene(bodyParts, scenario)) {
			std::cout << "Cannot configure scene. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}


#ifdef CAP_ACCELERATION
		//setup vectors for keeping velocities
		dReal previousLinVel[3];
		dReal previousAngVel[3];
#endif

		// ---------------------------------------
		// Main Loop
		// ---------------------------------------

		int count = 0;
		double t = 0;


		double step = configuration->getTimeStepLength();
		while ((t < configuration->getSimulationTime())
			   && (!(visualize && viewer->done()))) {

			if(visualize) {
				if(!viewer->frame(t, count)) {
					continue;
				}
			}



			if ((count++) % 500 == 0) {
				std::cout << "." << std::flush;
			}

			// Collision detection
			dSpaceCollide(odeSpace, configuration.get(), odeCollisionCallback);

			// Step the world by one timestep
			dWorldStep(odeWorld, step);

			// Empty contact groups used for collisions handling
			dJointGroupEmpty(odeContactGroup);

#ifdef CAP_ACCELERATION
			dBodyID rootBody =
					robot->getCoreComponent()->getRoot();
			const dReal *angVel, *linVel;

			angVel = dBodyGetAngularVel(rootBody);
			linVel = dBodyGetLinearVel(rootBody);

			if(t > 0) {
				double angAccel = dCalcPointsDistance3(
						angVel, previousAngVel);
				double linAccel = dCalcPointsDistance3(
						linVel, previousLinVel);

				if(angAccel > MAX_ACCELERATION || linAccel > MAX_ACCELERATION) {
					printf("EVALUATION CANCELED: max accel");
					printf(" exceeded at time %f,", t);
					printf("  will give 0 fitness.\n");
					accelerationCapExceeded = true;
					break;
				}

			}

			// save current velocities as previous
			for(unsigned int j=0; j<3; j++) {
				previousAngVel[j] = angVel[j];
				previousLinVel[j] = linVel[j];
			}
#endif


			float networkInput[MAX_INPUT_NEURONS];
			float networkOutputs[MAX_OUTPUT_NEURONS];

			// Elapsed time since last call
			env->setTimeElapsed(step);

			// Update Sensors
			for (unsigned int i = 0; i < bodyParts.size();
					++i) {
				if (boost::dynamic_pointer_cast<
						PerceptiveComponent>(bodyParts[i])) {
					boost::dynamic_pointer_cast<
							PerceptiveComponent>(bodyParts[i])->updateSensors(
							env);
				}
			}

			if(((count - 1) % configuration->getActuationPeriod()) == 0) {
				// Feed neural network
				for (unsigned int i = 0; i < sensors.size(); ++i) {

					if (boost::dynamic_pointer_cast<TouchSensor>(
							sensors[i])) {
						networkInput[i] =
								boost::dynamic_pointer_cast<
										TouchSensor>(sensors[i])->read();
					} else if (boost::dynamic_pointer_cast<
							LightSensor>(sensors[i])) {
						networkInput[i] =
								boost::dynamic_pointer_cast<
										LightSensor>(sensors[i])->read(
										env->getLightSources(),
										env->getAmbientLight());
					} else if (boost::dynamic_pointer_cast<
							SimpleSensor>(sensors[i])) {
						networkInput[i] =
								boost::dynamic_pointer_cast<
										SimpleSensor>(sensors[i])->read();
					}
				}
				if (log) {
					log->logSensors(networkInput, sensors.size());
				}


				::feed(neuralNetwork.get(), &networkInput[0]);

				// Step the neural network
				::step(neuralNetwork.get(), t);

				// Fetch the neural network ouputs
				::fetch(neuralNetwork.get(), &networkOutputs[0]);

				// Send control to motors
				for (unsigned int i = 0; i < motors.size(); ++i) {
					if (boost::dynamic_pointer_cast<ServoMotor>(
							motors[i])) {

						boost::shared_ptr<ServoMotor> motor =
								boost::dynamic_pointer_cast<
										ServoMotor>(motors[i]);

						if (motor->isVelocityDriven()) {
							motor->setVelocity(networkOutputs[i], step *
									configuration->getActuationPeriod());
						} else {
							motor->setPosition(networkOutputs[i], step *
									configuration->getActuationPeriod());
						}
					}
				}

				if(log) {
					log->logMotors(networkOutputs, motors.size());
				}
			}

			if (!scenario->afterSimulationStep()) {
				std::cout
						<< "Cannot execute scenario after simulation step. Quit."
						<< std::endl;
				return SIMULATION_FAILURE;
			}

			if(log) {
				log->logPosition(
					scenario->getRobot(
							)->getCoreComponent()->getRootPosition());
			}

			t += step;

		}

		if (!scenario->endSimulation()) {
			std::cout << "Cannot complete scenario. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}

		// ---------------------------------------
		// Simulator finalization
		// ---------------------------------------

		// Destroy robot (because of associated ODE joint group)
		robot.reset();
		// has shared pointer in scenario, so destroy that too
		scenario->prune();

		// Destroy the joint group
		dJointGroupDestroy(odeContactGroup);

		// Destroy ODE space
		dSpaceDestroy(odeSpace);

		// Destroy ODE world
		dWorldDestroy(odeWorld);

		// Destroy the ODE engine
		dCloseODE();

		if(accelerationCapExceeded)
			return ACCELERATION_CAP_EXCEEDED;

		if(onlyOnce) {
			break;
		}
	}
	return SIMULATION_SUCCESS;
}

}
