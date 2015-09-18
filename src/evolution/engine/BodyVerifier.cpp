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

// Comment the following if you don't want OSG popping up all the time
//#define VISUAL_DEBUG

namespace robogen {

//std::vector<dGeomID> BodyVerifier::cylinders;

BodyVerifier::BodyVerifier() {

}

BodyVerifier::~BodyVerifier() {
}

void BodyVerifier::collisionCallback(void *data, dGeomID o1, dGeomID o2) {

	CollisionData *collisionData = (CollisionData *) data;
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


	if (dGeomGetClass(o1) == dCylinderClass &&
			dGeomGetClass(o2) == dCylinderClass) {
		collisionData->cylinders.push_back(o1);
		collisionData->cylinders.push_back(o2);
	}
	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
				sizeof(dContact));

	if (collisionCounts != 0) {
		collisionData->offendingBodies.push_back(std::pair<dBodyID,
				dBodyID>(b1, b2));
	}
}

bool BodyVerifier::verify(const RobotRepresentation &robotRep, int &errorCode,
		std::vector<std::pair<std::string, std::string> > &affectedBodyParts) {

	bool success = true;
	errorCode = INTERNAL_ERROR;

	// Initialize ODE
	dInitODE();
	dWorldID odeWorld = dWorldCreate();
	dWorldSetGravity(odeWorld, 0, 0, 0);
	dSpaceID odeSpace = dHashSpaceCreate(0);
	CollisionData *collisionData = new CollisionData();


#ifdef VISUAL_DEBUG
	// Initialize OSG
	osgViewer::Viewer viewer;
	viewer.setUpViewInWindow(200, 200, 800, 600);
	osg::ref_ptr<KeyboardHandler> keyboardEvent(new KeyboardHandler(true,
			true, true));
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
		success = false;
	}

	if (success) {

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
						<< "Please check that the models/ folder exists in the parent folder of this executable."
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
			std::vector<boost::shared_ptr<SimpleBody> > dBodies = bodyParts[i]->getBodies();
			for (unsigned int j = 0; j < dBodies.size(); ++j) {
				dBodyToPartID[dBodies[j]->getBody()] = bodyParts[i]->getId();
			}
		}

		dSpaceCollide(odeSpace, (void *) collisionData, collisionCallback);
		std::vector<std::pair<dBodyID, dBodyID> >::iterator iter;
		for (iter = collisionData->offendingBodies.begin();
				iter != collisionData->offendingBodies.end(); ) {
			std::string bodyA = dBodyToPartID[iter->first];
			std::string bodyB = dBodyToPartID[iter->second];
			//std::cout << bodyA << " " << bodyB  << std::endl;

			//std:: cout << robotRep.getBody().at(bodyA).lock()->getId() << std::endl;
			//std:: cout << robotRep.getBody().at(bodyB).lock()->getId() << std::endl;

			// purge out parent/child pairs
			// TODO is there a better way to do this?  Stopped working do to
			// overlaps in components
			if (robotRep.getBody().at(bodyA).lock()->getParent() ==
					robotRep.getBody().at(bodyB).lock().get() ||
				robotRep.getBody().at(bodyB).lock()->getParent() ==
					robotRep.getBody().at(bodyA).lock().get() ) {
				//std::cout << "parent/child\n";
				iter = collisionData->offendingBodies.erase(iter);

			} else {
				++iter;
			}
		}

		if (collisionData->offendingBodies.size()) {
			//std::cout << "self intersection!" << std::endl;
			success = false;
			errorCode = SELF_INTERSECTION;
		} else if(collisionData->cylinders.size() > 0) {
			// hack to make sure we have separation between wheels
			for(unsigned int i=0; i<collisionData->cylinders.size(); ++i) {
				dReal radius;
				dReal length;
				dGeomCylinderGetParams(collisionData->cylinders[i],
						&radius, &length);
				dGeomCylinderSetParams(collisionData->cylinders[i],
						radius + WHEEL_SEPARATION, length);
			}
			dSpaceCollide(odeSpace, (void *) collisionData, collisionCallback);
			if ( collisionData->offendingBodies.size() ) {
				//std::cout << "self intersection!" << std::endl;
				success = false;
				errorCode = SELF_INTERSECTION;
			}
			collisionData->cylinders.clear();
		}
		for (unsigned int i = 0; i < collisionData->offendingBodies.size(); i++) {
			// TODO unlikely event that body not found in map?
			affectedBodyParts.push_back(
					std::pair<std::string, std::string>(
							dBodyToPartID[collisionData->offendingBodies[i].first],
							dBodyToPartID[collisionData->offendingBodies[i].second
										  ]));


		}

	#ifdef VISUAL_DEBUG
		// show robot in viewer
		while (!keyboardEvent->isQuit() && !viewer.done()) {
			viewer.frame();
		};
	#endif

	}

	// destruction
	collisionData->offendingBodies.clear();
	delete collisionData;
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
					unsigned int numDesc[] =
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
