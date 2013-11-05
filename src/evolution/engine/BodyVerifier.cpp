/*
 * BodyVerifier.cpp
 *
 *  Created on: Sep 16, 2013
 *      Author: lis
 */

#include "evolution/engine/BodyVerifier.h"
#include "Robot.h"
#include "utils/RobogenUtils.h"
#include "model/Model.h"

namespace robogen {

BodyVerifier::BodyVerifier() {

}

BodyVerifier::~BodyVerifier() {
}

void BodyVerifier::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
	std::vector<std::pair<dBodyID, dBodyID> > *offendingBodies = (std::vector<
			std::pair<dBodyID, dBodyID> > *) data;
	offendingBodies->clear();
	const int MAX_CONTACTS = 32; // maximum number of contact points per body
	// TODO will it work with just 1 point? Probably yes.

	// exit without doing anything if the two bodies are connected by a joint
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
		return;
	}

	dContact contact[MAX_CONTACTS];
	for (int i = 0; i < MAX_CONTACTS; i++) {
		contact[i].surface.mode = 0;
	}

	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
			sizeof(dContact));

	if (collisionCounts != 0) {
		offendingBodies->push_back(std::pair<dBodyID, dBodyID>(b1, b2));
	}
}

bool BodyVerifier::verify(const RobotRepresentation &robotRep, int &errorCode,
		std::vector<std::pair<std::string, std::string> > &affectedBodyParts) {

	bool success = true;
	errorCode = INTERNAL_ERROR;
	// TODO check arduino constraints satisfied before anything else!

	// Initialize ODE
	dInitODE();
	dWorldID odeWorld = dWorldCreate();
	dWorldSetGravity(odeWorld, 0, 0, 0);
	dSpaceID odeSpace = dHashSpaceCreate(0);
	dJointGroupID odeJoints = dJointGroupCreate(0);

#ifdef VISUAL_DEBUG
	// Initialize OSG
	osgViewer::Viewer viewer;
	viewer.setUpViewInWindow(200, 200, 800, 600);
	osg::ref_ptr<KeyboardHandler> keyboardEvent(new KeyboardHandler());
	viewer.addEventHandler(keyboardEvent.get());
	osg::ref_ptr<osg::Camera> camera = viewer.getCamera();
#endif

	// parse robot message
	robogenMessage::Robot robotMessage = robotRep.serialize();
	// parse robot
	boost::shared_ptr<Robot> robot(new Robot);
	if (!robot->init(odeWorld, odeSpace, robotMessage)) {
		std::cout << "Problem when initializing robot in body verifier!"
				<< std::endl;
		return false;
	}
	std::vector<boost::shared_ptr<Model> > bodyParts = robot->getBodyParts();

#ifdef VISUAL_DEBUG
	// body part rendering
	osg::ref_ptr<osg::Group> root = osg::ref_ptr<osg::Group>(new osg::Group);
	std::vector<boost::shared_ptr<RenderModel> > renderModels;
	for (unsigned int i = 0; i < bodyParts.size(); ++i) {
		boost::shared_ptr<RenderModel> renderModel =
				RobogenUtils::createRenderModel(bodyParts[i]);
		if (renderModel == NULL) {
			std::cout << "Cannot create a render model for model " << i
					<< std::endl;
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
	// viewer setup
	viewer.setSceneData(root.get());
	viewer.realize();
	if (!viewer.getCameraManipulator()
			&& viewer.getCamera()->getAllowEventFocus()) {
		viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	}
	viewer.setReleaseContextAtEndOfFrameHint(false);
#endif

	// build map from ODE bodies to body part ID's: needed to identify
	// offending body parts
	std::map<dBodyID, std::string> dBodyToPartID;
	for (unsigned int i = 0; i < bodyParts.size(); ++i) {
		std::vector<dBodyID> dBodies = bodyParts[i]->getBodies();
		for (unsigned int j = 0; j < dBodies.size(); ++j) {
			dBodyToPartID[dBodies[j]] = bodyParts[i]->getId();
		}
	}

	// perform body part collision test
	std::vector<std::pair<dBodyID, dBodyID> > offendingBodies;
	dSpaceCollide(odeSpace, (void *) &offendingBodies, collisionCallback);
	if (offendingBodies.size()) {
		success = false;
		errorCode = SELF_INTERSECTION;
	}
	for (unsigned int i = 0; i < offendingBodies.size(); i++) {
		// TODO unlikely event that body not found in map?
		affectedBodyParts.push_back(
				std::pair<std::string, std::string>(
						dBodyToPartID[offendingBodies[i].first],
						dBodyToPartID[offendingBodies[i].second]));
	}

#ifdef VISUAL_DEBUG
	// show robot in viewer
	while (!keyboardEvent->isQuit() && !viewer.done()) {
		viewer.frame();
	};
#endif

	// destruction
	robot.reset();
	dSpaceDestroy(odeSpace);
	dWorldDestroy(odeWorld);
	dCloseODE();
	return success;
}

bool BodyVerifier::fixRobotBody(RobotRepresentation &robot) {
	bool changed = false;
	while (true) {
		// check velidity of body
		int errorCode;
		std::vector<std::pair<std::string, std::string> > offenders;
		if (!BodyVerifier::verify(robot, errorCode, offenders)) {
			// TODO treat other cases: Arduino constraints, missing core
			if (errorCode == BodyVerifier::SELF_INTERSECTION) {
				std::cerr << "Robot body has following intersection pairs:"
						<< std::endl;
				for (unsigned int i = 0; i < offenders.size(); ++i) {
					// get robots IdPartMap: Volatile, so needs update
					const RobotRepresentation::IdPartMap idPartMap =
							robot.getBody();
					std::cerr << offenders[i].first << " with "
							<< offenders[i].second << std::endl;

					// check if offending body part hasn't been removed yet
					if (idPartMap.find(offenders[i].first) == idPartMap.end()
							|| idPartMap.find(offenders[i].second)
									== idPartMap.end()) {
						continue;
					}
					// will remove body part with less descendants (i.e. this
					// covers the case where one part descends from the other)
					int numDesc[] =
							{
									idPartMap.find(offenders[i].first)->second.lock()->numDescendants(),
									idPartMap.find(offenders[i].second)->second.lock()->numDescendants() };
					std::cout << offenders[i].first << " has " << numDesc[0]
							<< " descendants" << std::endl;
					std::cout << offenders[i].second << " has " << numDesc[1]
							<< " descendants" << std::endl;
					if (numDesc[0] > numDesc[1]) {
						robot.trimBodyAt(offenders[i].second);
						std::cout << "Removing latter" << std::endl;
					} else {
						robot.trimBodyAt(offenders[i].first);
						std::cout << "Removing former" << std::endl;
					}
					changed = true;
				}
			}
			if (errorCode == BodyVerifier::INTERNAL_ERROR) {
				std::cout << "Body verification failed due to internal error!"
						<< std::endl;
				return false;
			}
		} else {
			break;
		}
	}
	return changed;
}

} /* namespace robogen */
