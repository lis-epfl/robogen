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
	// Initialize ODE
	dInitODE();
}

BodyVerifier::~BodyVerifier() {
	// Close ODE
	dCloseODE();
}

void BodyVerifier::collisionCallback(void *data, dGeomID o1, dGeomID o2){
	std::vector<std::pair<dBodyID, dBodyID> > *offendingBodies =
			(std::vector<std::pair<dBodyID, dBodyID> > *) data;
	offendingBodies->clear();
	const int MAX_CONTACTS = 32; // maximum number of contact points per body

	// exit without doing anything if the two bodies are connected by a joint
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) {
		return;
	}

	dContact contact[MAX_CONTACTS];
	for (int i = 0; i < MAX_CONTACTS; i++) {
		contact[i].surface.mode = 0;
	}

	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
			sizeof(dContact));

	if (collisionCounts != 0) {
		dBodyID b1 = dGeomGetBody(o1);
		dBodyID b2 = dGeomGetBody(o2);
		offendingBodies->push_back(std::pair<dBodyID, dBodyID>(b1,b2));
	}
}

bool BodyVerifier::verify(const RobotRepresentation &robotRep, int &errorCode,
		std::vector<std::pair<std::string,std::string> > &affectedBodyParts){

	bool success = true;
	// TODO check arduino constraints satisfied before anything else!

	// Initialize ODE
	dWorldID odeWorld = dWorldCreate();
	dWorldSetGravity(odeWorld, 0, 0, 0);
	dSpaceID odeSpace = dHashSpaceCreate(0);
	dJointGroupID odeJoints = dJointGroupCreate(0);

	// Initialize OSG
	osgViewer::Viewer viewer;
	viewer.setUpViewInWindow(200, 200, 800, 600);
	osg::ref_ptr<KeyboardHandler> keyboardEvent(new KeyboardHandler());
	viewer.addEventHandler(keyboardEvent.get());
	osg::ref_ptr<osg::Camera> camera = viewer.getCamera();
	// parse robot message
	robogenMessage::Robot robotMessage = robotRep.serialize();
	// parse robot
	boost::shared_ptr<Robot> robot(new Robot(odeWorld,odeSpace,robotMessage));
	std::vector<boost::shared_ptr<Model> > bodyParts = robot->getBodyParts();

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

	// build map from ODE bodies to body part ID's: needed to identify
	// offending bodies
	std::map<dBodyID, std::string> dBodyToPartID;
	for (unsigned int i=0; i<bodyParts.size(); ++i){
		std::vector<dBodyID> dBodies = bodyParts[i]->getBodies();
		for (unsigned int j=0; j<dBodies.size(); ++j){
			dBodyToPartID[dBodies[j]] = bodyParts[i]->getId();
		}
	}

	// perform body part collision test
	std::vector<std::pair<dBodyID, dBodyID> > offendingBodies;
	dSpaceCollide(odeSpace, (void *)&offendingBodies, collisionCallback);
	if (offendingBodies.size()){
		success = false;
		errorCode = this->SELF_INTERSECTION;
	}
	for (unsigned int i=0; i<offendingBodies.size(); i++){
		// TODO unlikely event that body not found in map?
		affectedBodyParts.push_back(std::pair<std::string, std::string>(
				dBodyToPartID[offendingBodies[i].first],
				dBodyToPartID[offendingBodies[i].second]));
	}

	// show robot in viewer
	while (!keyboardEvent->isQuit() && !viewer.done()){
		viewer.frame();
	};

	// destruction
	robot.reset();
	dSpaceDestroy(odeSpace);
	dWorldDestroy(odeWorld);
	return success;
}



} /* namespace robogen */
