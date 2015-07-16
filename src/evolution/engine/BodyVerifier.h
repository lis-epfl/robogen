/*
 * BodyVerifier.h
 *
 *  Created on: Sep 16, 2013
 *      Author: lis
 */

#ifndef BODYVERIFIER_H_
#define BODYVERIFIER_H_

#include <boost/shared_ptr.hpp>

#include "Robogen.h"
#include "config/ConfigurationReader.h"
#include "config/RobogenConfig.h"
#include "evolution/representation/RobotRepresentation.h"
#include "scenario/Scenario.h"
#include "scenario/ScenarioFactory.h"
#include "utils/network/ProtobufPacket.h"
#include "utils/network/TcpSocket.h"
#include "utils/RobogenCollision.h"
#include "utils/RobogenUtils.h"
#include "Models.h"
#include "RenderModels.h"
#include "Robogen.h"
#include "Robot.h"
#include "robogen.pb.h"
#include "viewer/FileViewerLog.h"
#include "evolution/representation/RobotRepresentation.h"

#ifdef VISUAL_DEBUG
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include "viewer/KeyboardHandler.h"
#endif

#include <ode/ode.h>
#include <vector>

// includes pasted from FileViewer.cpp for the sake of please just work

#define WHEEL_SEPARATION 0.005

namespace robogen {

/**
 * The body verifier class takes care of the verification of a given body
 * design in the evolver. It borrows from simulator code to build a robot
 * in ODE and run a collision test on it. It also makes sure the constraints
 * imposed by the arduino (amount for sensors, motors) are satisfied.
 */
class BodyVerifier {
public:
	/**
	 * error codes used to indicate the problem with a given robot design
	 */
	enum errorCodes{
		INTERNAL_ERROR,
		ARDUINO_CONSTRAINTS_EXCESS,
		MISSING_CORE_COMPONENT,
		SELF_INTERSECTION
	};

	/**
	 * ODE collision callback which identifies intersecting body parts
	 */
	static void collisionCallback(void*, dGeomID o1, dGeomID o2);

	/**
	 * Verifies a given robot design. Should be similar to file viewer code
	 * @param robot representation of robot to verify
	 * @param errorCode if error, corresponding errorCode from errorCodes
	 * @param affectedBodyParts if error, identifiers of affected body parts
	 * @return true if robot valid, false otherwise
	 */
	static bool verify(const RobotRepresentation &robot, int &errorCode,
			std::vector<std::pair<std::string,std::string> >&affectedBodyParts);

	/**
	 * Suggested routine for handling a robot body with potential
	 * self-intersections: For each pair of offending body parts, the amount
	 * of bodyparts in the substree of each offending part is counted and the
	 * body part with less subtree parts is trimmed. In particular, this handles
	 * the problem where one part is in the subtree of the other per definition.
	 * Other routines can be implemented using verify().
	 * @param robot robot to be treated
	 */
	static bool fixRobotBody(RobotRepresentation &robot);

private:
	/**
	 * Private constructor prevents instantiation
	 */
	BodyVerifier();

	virtual ~BodyVerifier();

	struct CollisionData {
		std::vector<std::pair<dBodyID, dBodyID> > offendingBodies;
		std::vector<dGeomID> cylinders;
	};


};

} /* namespace robogen */
#endif /* BODYVERIFIER_H_ */
