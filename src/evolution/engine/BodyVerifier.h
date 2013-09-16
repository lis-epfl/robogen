/*
 * BodyVerifier.h
 *
 *  Created on: Sep 16, 2013
 *      Author: lis
 */

#ifndef BODYVERIFIER_H_
#define BODYVERIFIER_H_

#include <boost/shared_ptr.hpp>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
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
#include "viewer/KeyboardHandler.h"
#include "Models.h"
#include "RenderModels.h"
#include "Robogen.h"
#include "Robot.h"
#include "robogen.pb.h"
#include "viewer/FileViewerLog.h"
#include "evolution/representation/RobotRepresentation.h"

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
		ARDUINO_CONSTRAINTS_EXCESS,
		MISSING_CORE_COMPONENT,
		SELF_INTERSECTION
	};

	/**
	 * Creates an ODE/OSG instance ready to review robots. Should be similar to
	 * file viewer code.
	 */
	BodyVerifier();

	virtual ~BodyVerifier();

	/**
	 * Verifies a given robot design. Should be similar to file viewer code
	 * @param robot representation of robot to verify
	 * @param errorCode if error, corresponding errorCode from errorCodes
	 * @param affectedBodyParts if error, identifiers of affected body parts
	 * @return true if robot valid, false otherwise
	 */
	bool verify(const RobotRepresentation &robot, int &errorCode,
			std::vector<std::pair<std::string,std::string> > &affectedBodyParts);

	static void collisionCallback(void*, dGeomID o1, dGeomID o2);

};

} /* namespace robogen */
#endif /* BODYVERIFIER_H_ */
