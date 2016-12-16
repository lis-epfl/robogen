/*
 * @(#) Simulator.cpp   1.0   Nov 27, 2014
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
#include "Models.h"
#include "Robot.h"
#include "viewer/WebGLLogger.h"

//#define DEBUG_MASSES

// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen{

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage,
		IViewer *viewer, boost::random::mt19937 &rng) {
	boost::shared_ptr<FileViewerLog> log;
	return runSimulations(scenario, configuration,
			robotMessage, viewer, rng, false, log);
}

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const robogenMessage::Robot &robotMessage, IViewer *viewer,
		boost::random::mt19937 &rng,
		bool onlyOnce, boost::shared_ptr<FileViewerLog> log) {

	bool constraintViolated = false;

	boost::random::normal_distribution<float> normalDistribution;
	boost::random::uniform_01<float> uniformDistribution;

	while (scenario->remainingTrials() && (!constraintViolated)) {

		// ---------------------------------------
		// Simulator initialization
		// ---------------------------------------

		dInitODE();

		// Create ODE world
		odeWorld = dWorldCreate();

		// Set gravity from config
		osg::Vec3 gravity = configuration->getGravity();
		dWorldSetGravity(odeWorld, gravity.x(), gravity.y(), gravity.z());

		dWorldSetERP(odeWorld, 0.1);
		dWorldSetCFM(odeWorld, 10e-6);
		dWorldSetAutoDisableFlag(odeWorld, 1);

		// Create collision world
		dSpaceID odeSpace = dSimpleSpaceCreate(0);

		// Create contact group
		odeContactGroup = dJointGroupCreate(0);

		// wrap all this in block so things get cleaned up before shutting down
		// ode
		{

		// ---------------------------------------
		// Generate Robot
		// ---------------------------------------
		//Gael Degug*************************************************************
			std::cout <<"Simulator::runSimulations"
					 <<std::endl;
		//***********************************************************************
		boost::shared_ptr<Robot> robot(new Robot);
		if (!robot->init(odeWorld, odeSpace, robotMessage)) {
			std::cout << "Problems decoding the robot. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}

#ifdef DEBUG_MASSES
		float totalMass = 0;
		for (unsigned int i = 0; i < robot->getBodyParts().size(); ++i) {
			float partMass = 0;
			for (unsigned int j = 0;
					j < robot->getBodyParts()[i]->getBodies().size(); ++j) {

				dMass mass;
				dBodyGetMass(robot->getBodyParts()[i]->getBodies()[j], &mass);
				partMass += mass.mass;

			}
			std::cout << robot->getBodyParts()[i]->getId() <<  " has mass: "
					<< partMass * 1000. << "g" << std::endl;
			totalMass += partMass;
		}

		std::cout << "total mass is " << totalMass * 1000. << "g" << std::endl;
#endif



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

		// set cap for checking motor burnout
		for(unsigned int i=0; i< motors.size(); i++) {
			motors[i]->setMaxDirectionShiftsPerSecond(
						configuration->getMaxDirectionShiftsPerSecond());

		}

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

		if((configuration->getObstacleOverlapPolicy() ==
				RobogenConfig::CONSTRAINT_VIOLATION) &&
				scenario->wereObstaclesRemoved()) {
			std::cout << "Using 'contraintViolation' obstacle overlap policy,"
					<< " and ostacles were removed, so will return min fitness."
					<< std::endl;
			constraintViolated = true;
			break;
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

		/***
         * Init the webGLLogger
         */
        boost::shared_ptr<WebGLLogger> webGLlogger;
        if (log && log->isWriteWebGL()) {
        	webGLlogger.reset(new WebGLLogger(log->getWebGLFileName(),
        										scenario));
        }



		//setup vectors for keeping velocities
		dReal previousLinVel[3];
		dReal previousAngVel[3];

		// ---------------------------------------
		// Main Loop
		// ---------------------------------------

		int count = 0;
		double t = 0;

		boost::shared_ptr<CollisionData> collisionData(
				new CollisionData(scenario) );

		double step = configuration->getTimeStepLength();
		while ((t < configuration->getSimulationTime())
			   && (!(visualize && viewer->done()))) {

			if(visualize) {
				if(!viewer->frame(t, count)) {
					continue;
				}
			}


			if (scenario->shouldStopSimulationNow()) {
				std::cout << "Scenario has stopped the simulation!"
						<< std::endl;
				break;
			}

			if ((count++) % 500 == 0) {
				std::cout << "." << std::flush;
			}

			// Collision detection
			dSpaceCollide(odeSpace, collisionData.get(), odeCollisionCallback);

			// Step the world by one timestep
			dWorldStep(odeWorld, step);

			// Empty contact groups used for collisions handling
			dJointGroupEmpty(odeContactGroup);

			if (configuration->isDisallowObstacleCollisions() &&
					collisionData->hasObstacleCollisions()) {
				constraintViolated = true;
				break;
			}

			if (configuration->isCapAlleration()) {
				dBodyID rootBody =
						robot->getCoreComponent()->getRoot()->getBody();
				const dReal *angVel, *linVel;

				angVel = dBodyGetAngularVel(rootBody);
				linVel = dBodyGetLinearVel(rootBody);

				if(t > 0) {
					// TODO make this use the step size and update default
					// limits to account for this
					double angAccel = dCalcPointsDistance3(
							angVel, previousAngVel);
					double linAccel = dCalcPointsDistance3(
							linVel, previousLinVel);

					if(angAccel > configuration->getMaxAngularAcceleration() ||
					   linAccel > configuration->getMaxLinearAcceleration()) {

						printf("EVALUATION CANCELED: max accel");
						printf(" exceeded at time %f.", t);
						printf(" Angular accel: %f, Linear accel: %f.\n",
								angAccel, linAccel);
						printf("Will give %f fitness.\n", MIN_FITNESS);
						constraintViolated = true;
						break;
					}

				}

				// save current velocities as previous
				for(unsigned int j=0; j<3; j++) {
					previousAngVel[j] = angVel[j];
					previousLinVel[j] = linVel[j];
				}
			}


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
					networkInput[i] = sensors[i]->read();

					// Add sensor noise: Gaussian with std dev of
					// sensorNoiseLevel * actualValue
					if (configuration->getSensorNoiseLevel() > 0.0) {
						networkInput[i] += (normalDistribution(rng) *
								configuration->getSensorNoiseLevel() *
								networkInput[i]);
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

					// Add motor noise:
					// uniform in range +/- motorNoiseLevel * actualValue
					if(configuration->getMotorNoiseLevel() > 0.0) {
						networkOutputs[i] += (
									((uniformDistribution(rng) *
									2.0 *
									configuration->getMotorNoiseLevel())
									- configuration->getMotorNoiseLevel())
									* networkOutputs[i]);
					}


					if (boost::dynamic_pointer_cast<
							RotationMotor>(motors[i])) {
						boost::dynamic_pointer_cast<RotationMotor>(motors[i]
						   )->setDesiredVelocity(networkOutputs[i], step *
								   	   	  configuration->getActuationPeriod());
					} else if (boost::dynamic_pointer_cast<
							ServoMotor>(motors[i])) {
						boost::dynamic_pointer_cast<ServoMotor>(motors[i]
						   )->setDesiredPosition(networkOutputs[i], step *
								configuration->getActuationPeriod());
						//motor->setPosition(networkOutputs[i], step *
						//		configuration->getActuationPeriod());
					}

				}

				if(log) {
					log->logMotors(networkOutputs, motors.size());
				}
			}

			bool motorBurntOut = false;
			for (unsigned int i = 0; i < motors.size(); ++i) {
				motors[i]->step( step ) ; //* configuration->getActuationPeriod() );

				// TODO find a cleaner way to do this
				// for now will reuse accel cap infrastructure
				if (motors[i]->isBurntOut()) {
					std::cout << "Motor burnt out, will terminate now "
							<< std::endl;
					motorBurntOut = true;
					//constraintViolated = true;
				}

			}

			if(constraintViolated || motorBurntOut) {
				break;
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

			if(webGLlogger) {
				webGLlogger->log(t);
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

		// Destroy the WebGlLogger, since contains pointer to scenario
		if(webGLlogger) {
			webGLlogger.reset();
		}
		} // end code block protecting objects for ode code clean up


		// scenario has a shared ptr to the robot, so need to prune it
		scenario->prune();

		// Destroy the joint group
		dJointGroupDestroy(odeContactGroup);

		// Destroy ODE space
		dSpaceDestroy(odeSpace);

		// Destroy ODE world
		dWorldDestroy(odeWorld);

		// Destroy the ODE engine
		dCloseODE();

		if(constraintViolated || onlyOnce) {
			break;
		}
	}
	if(constraintViolated)
		return CONSTRAINT_VIOLATED;
	return SIMULATION_SUCCESS;
}

}
