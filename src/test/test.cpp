#include "Robogen.h"
#include "utils/RobogenUtils.h"

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <iostream>

#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include "model/motors/ServoMotor.h"
#include "model/objects/LightSource.h"
#include "model/sensors/LightSensor.h"
#include "model/sensors/TouchSensor.h"

#include "model/components/actuated/ActiveCardanModel.h"
#include "model/components/actuated/ActiveHingeModel.h"
#include "model/components/actuated/ActiveWheelModel.h"
#include "model/components/CardanModel.h"
#include "model/components/perceptive/CoreComponentModel.h"
#include "model/components/HingeModel.h"
#include "model/components/ParametricBrickModel.h"
#include "model/components/PassiveWheelModel.h"
#include "model/components/actuated/RotateJointModel.h"
#include "model/components/actuated/ActiveWhegModel.h"
#include "model/components/perceptive/TouchSensorModel.h"
#include "model/components/perceptive/LightSensorModel.h"

#include "render/components/ActiveCardanRenderModel.h"
#include "render/components/ActiveHingeRenderModel.h"
#include "render/components/ActiveWheelRenderModel.h"
#include "render/components/CardanRenderModel.h"
#include "render/components/CoreComponentRenderModel.h"
#include "render/components/HingeRenderModel.h"
#include "render/components/ParametricBrickRenderModel.h"
#include "render/components/PassiveWheelRenderModel.h"
#include "render/components/RotateJointRenderModel.h"
#include "render/components/ActiveWhegRenderModel.h"
#include "render/components/TouchSensorRenderModel.h"
#include "render/components/LightSensorRenderModel.h"
#include "render/objects/LightSourceRender.h"

#include "scenario/Terrain.h"
#include "render/objects/TerrainRender.h"

using namespace robogen;

// ODE World
dWorldID odeWorld;

// Container for collisions
dJointGroupID odeContactGroup;

std::vector<boost::shared_ptr<TouchSensor> > touchSensors;

void setupCollision(std::vector<boost::shared_ptr<TouchSensor> >& sensors) {

	for (unsigned int i = 0; i < sensors.size(); ++i) {
	//	sensors[i]->reset();
	}

}

void odeCollisionCallback(void*, dGeomID o1, dGeomID o2) {

	const int MAX_CONTACTS = 8; // maximum number of contact points per body

	// exit without doing anything if the two bodies are connected by a joint
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 && b2 && dAreConnected(b1, b2)) {
		return;
	}

	dContact contact[MAX_CONTACTS];
	for (int i = 0; i < MAX_CONTACTS; i++) {

		contact[i].surface.mode = dContactBounce | dContactSoftCFM;
		contact[i].surface.mu = dInfinity;
		contact[i].surface.mu2 = 0;
		contact[i].surface.bounce = 0.01;
		contact[i].surface.bounce_vel = 0.1;
		contact[i].surface.soft_cfm = 0.0001;

	}

	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
			sizeof(dContact));

	if (collisionCounts != 0) {
		for (int i = 0; i < collisionCounts; i++) {

			dJointID c = dJointCreateContact(odeWorld, odeContactGroup,
					contact + i);
			dJointAttach(c, b1, b2);

		}

		/*
		// Check if one of the two is a touch sensor
		void* customData1 = dGeomGetData(o1);
		if (customData1 != NULL) {
			CustomGeomData* data = (CustomGeomData*) customData1;
			if (data->getId() == CustomGeomData::TOUCH_SENSOR_INFO) {
				((TouchSensor::TouchSensorInfo*) data->getData())->touching =
						true;
			}
		}

		void* customData2 = dGeomGetData(o2);
		if (customData2 != NULL) {
			CustomGeomData* data = (CustomGeomData*) customData2;
			if (data->getId() == CustomGeomData::TOUCH_SENSOR_INFO) {
				((TouchSensor::TouchSensorInfo*) data->getData())->touching =
						true;
			}
		}
		*/
	}
}

template<typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>& operator<<(
		std::basic_ostream<CharT, Traits>& out, osg::Vec3& o) {
	out << "(" << o.x() << ", " << o.y() << ", " << o.z() << ")";
	return out;
}

template<typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>& operator<<(
		std::basic_ostream<CharT, Traits>& out, osg::Quat& o) {
	out << "(" << o.w() << "|" << o.x() << ", " << o.y() << ", " << o.z()
			<< ")";
	return out;
}

// Connects a to b
void connect(boost::shared_ptr<Model> a, unsigned int slotA,
		boost::shared_ptr<Model> b, unsigned int slotB, float orientation) {

	// 1) Rotate slotAxis of B such that we obtain a normal pointing inward the body
	osg::Vec3 bSlotAxis = b->getSlotAxis(slotB);
	osg::Vec3 bSlotAxisInv = -bSlotAxis;

	// 2) Find quaternion to rotate the vector pointing from the a root to the slot
	// and align it with B slot inward axis
	osg::Vec3 aSlotPos = a->getSlotPosition(slotA);
	osg::Vec3 aCenter = a->getRootPosition();
	osg::Vec3 aCenterAxis = aSlotPos - aCenter;

	osg::Quat rotAxisQuat = RobogenUtils::makeRotate(aCenterAxis, bSlotAxisInv);
	a->setRootAttitude(rotAxisQuat);

	// 3) Compute A new center and translate it
	osg::Vec3 bSlotPos = b->getSlotPosition(slotB);
	osg::Vec3 aSlotNewPos = bSlotPos;
	aSlotPos = a->getSlotPosition(slotA);
	osg::Vec3 aTranslation = aSlotNewPos - aSlotPos;
	a->setRootPosition(aCenter + aTranslation);

	if (!RobogenUtils::areAxisParallel(a->getSlotAxis(slotA),
			-b->getSlotAxis(slotB))) {
		std::cout << "ALERT1!!!!!! axis not parallel" << std::endl;
		osg::Vec3 a1 = a->getSlotAxis(slotA);
		osg::Vec3 a2 = b->getSlotAxis(slotB);
		std::cout << "a1: " << a1 << std::endl;
		std::cout << "a2: " << a2 << std::endl;
	}

	// 4) At this point we need to orient the slots to the "zero" orientation
	osg::Vec3 bSlotOrientation = b->getSlotOrientation(slotB);
	osg::Vec3 aSlotOrientation = a->getSlotOrientation(slotA);
	double angle = RobogenUtils::getAngle(aSlotOrientation, bSlotOrientation);

	//std::cout << "Angle: " << angle << std::endl;

	osg::Vec3 aSlotAxis = a->getSlotAxis(slotA);
	osg::Quat slotAlignRotation;
	slotAlignRotation.makeRotate(osg::inDegrees(angle), aSlotAxis);
	/*
	 std::cout << "bSlotOrientation: " << bSlotOrientation << std::endl;
	 std::cout << "aSlotOrientation: " << aSlotOrientation << std::endl;
	 std::cout << "slotAlignRotation: " << slotAlignRotation << std::endl;
	 */
	// ...and has the correct orientation
	if (fabs(orientation) < 1e-6 ) {

		osg::Quat rotOrientationQuat;
		rotOrientationQuat.makeRotate(osg::inDegrees(orientation), aSlotAxis);
		a->setRootAttitude(slotAlignRotation * rotOrientationQuat);

	} else {

		a->setRootAttitude(slotAlignRotation);

	}

	//osg::Vec3 a1 = a->getSlotAxis(slotA);
	//osg::Vec3 a2 = b->getSlotAxis(slotB);
	//std::cout << "a1: " << a1 << std::endl;
	//std::cout << "a2: " << a2 << std::endl;

	// Final check, if the axis are not parallel, something is very wrong
	if (!RobogenUtils::areAxisParallel(a->getSlotAxis(slotA),
			-b->getSlotAxis(slotB))) {
		std::cout << "ALERT2!!!!!! axis not parallel" << std::endl;
	}

	// Create a joint to hold pieces in position
	/*dJointID joint = dJointCreateSlider(odeWorld, 0);
	dJointAttach(joint, a->getSlot(slotA), b->getSlot(slotB));
	dJointSetSliderAxis(joint, 0, 0, 1);
	dJointSetSliderParam(joint, dParamLoStop, 0);
	dJointSetSliderParam(joint, dParamHiStop, 0);*/

	dJointID joint = dJointCreateFixed(odeWorld, 0);
	dJointAttach(joint, a->getSlot(slotA), b->getSlot(slotB));
	dJointSetFixed(joint);

}

int main(int /*argc*/, char* /*argv*/[]) {

	// ---------------------------------------
	// ODE Initialization
	// ---------------------------------------

	dInitODE();

	// Create ODE world
	odeWorld = dWorldCreate();

	// Set gravity
	dWorldSetGravity(odeWorld, 0, 0, -9.81);

	dWorldSetERP(odeWorld, 0.4);
	dWorldSetCFM(odeWorld, 0.2);

	// Create contact group
	odeContactGroup = dJointGroupCreate(0);

	// Create collision world
	dSpaceID odeSpace = dSimpleSpaceCreate(0);

	// ---------------------------------------
	// OSG Initialization
	// ---------------------------------------

	//Creating the viewer
	osgViewer::Viewer viewer;

	viewer.setUpViewInWindow(300, 300, 400, 400);

	// Camera
	osg::ref_ptr<osg::Camera> camera = viewer.getCamera();

	//Creating the root node
	osg::ref_ptr<osg::Group> root(new osg::Group);

	//--------------------------------------------------
	// Example
	//--------------------------------------------------

	// Update OSG visualization

	// OSG Plane
	/*osg::ref_ptr<osg::Box> osgPlane(
			new osg::Box(osg::Vec3(0, 0, 0), 500, 500, 0.1));
	osg::ref_ptr<osg::ShapeDrawable> osgPlaneDrawable(
			new osg::ShapeDrawable(osgPlane.get()));
	osg::ref_ptr<osg::Geode> osgGroundGeode(new osg::Geode());
	root->addChild(osgGroundGeode);
	osgGroundGeode->addDrawable(osgPlaneDrawable.get()); */

	boost::shared_ptr<Terrain> terrain(new Terrain(odeWorld, odeSpace));
	//terrain->initRough("../terrain.gif", 1000, 1000, 500);
	terrain->initFlat(1, 1);
	boost::shared_ptr<TerrainRender> terrainRender(new TerrainRender(terrain));
	root->addChild(terrainRender->getRootNode());


	 // CoreComponent A
	 boost::shared_ptr<CoreComponentModel> modelA(
	 new CoreComponentModel(odeWorld, odeSpace, "CoreA", false));
	 modelA->initModel();
	 modelA->setRootPosition(osg::Vec3(0, 0, 1));

	 boost::shared_ptr<CoreComponentRenderModel> renderModelA(
	 new CoreComponentRenderModel(modelA));
	 renderModelA->initRenderModel();

	 root->addChild(renderModelA->getRootNode());

	 // CoreComponent B
	 boost::shared_ptr<CoreComponentModel> modelB(
	 new CoreComponentModel(odeWorld, odeSpace, "CoreB", false));
	 modelB->initModel();
	 modelB->setRootPosition(osg::Vec3(0.1, 0.1, 1));

	 osg::Quat rotationModelB;
	 rotationModelB.makeRotate(osg::inDegrees(90.0), osg::Vec3(-1, 0, 0));
	 modelB->setRootAttitude(rotationModelB);

	 boost::shared_ptr<CoreComponentRenderModel> renderModelB(
	 new CoreComponentRenderModel(modelB));
	 renderModelB->initRenderModel();
	 root->addChild(renderModelB->getRootNode());

	 // CoreComponent C (the root!)
	 boost::shared_ptr<CoreComponentModel> modelC(
	 new CoreComponentModel(odeWorld, odeSpace, "CoreC", false));
	 modelC->initModel();
	 modelC->setRootPosition(osg::Vec3(0.1, 0.2, 0.1));
	 osg::Quat rotationAngleC;
	 rotationAngleC.makeRotate(osg::inDegrees(45.0), osg::Vec3(0, 1, 0));
	 modelC->setRootAttitude(rotationAngleC);

	 boost::shared_ptr<CoreComponentRenderModel> renderModelC(
	 new CoreComponentRenderModel(modelC));
	 renderModelC->initRenderModel();
	 root->addChild(renderModelC->getRootNode());

	 // CoreComponent D
	 boost::shared_ptr<CoreComponentModel> modelD(
	 new CoreComponentModel(odeWorld, odeSpace, "CoreD", false));
	 modelD->initModel();
	 modelD->setRootPosition(osg::Vec3(0.1, 0.3, 1));

	 boost::shared_ptr<CoreComponentRenderModel> renderModelD(
	 new CoreComponentRenderModel(modelD));
	 renderModelD->initRenderModel();
	 root->addChild(renderModelD->getRootNode());

	 // CoreComponent E
	 boost::shared_ptr<CoreComponentModel> modelE(
	 new CoreComponentModel(odeWorld, odeSpace, "CoreE", false));
	 modelE->initModel();
	 modelE->setRootPosition(osg::Vec3(0.1, 0.4, 1));

	 boost::shared_ptr<CoreComponentRenderModel> renderModelE(
	 new CoreComponentRenderModel(modelE));
	 renderModelE->initRenderModel();
	 root->addChild(renderModelE->getRootNode());

	 // Hinge joint
	 boost::shared_ptr<HingeModel> hingeA(new HingeModel(odeWorld, odeSpace,
			 "Hinge1"));
	 hingeA->initModel();
	 hingeA->setRootPosition(osg::Vec3(0.2, 0.1, 1));

	 boost::shared_ptr<HingeRenderModel> renderModelHingeA(
	 new HingeRenderModel(hingeA));
	 renderModelHingeA->initRenderModel();

	 root->addChild(renderModelHingeA->getRootNode());

	 // Hinge joint
	 boost::shared_ptr<HingeModel> hingeB(new HingeModel(odeWorld, odeSpace,
			 "Hinge2"));
	 hingeB->initModel();
	 hingeB->setRootPosition(osg::Vec3(0.2, 0.2, 1));

	 boost::shared_ptr<HingeRenderModel> renderModelHingeB(
	 new HingeRenderModel(hingeB));
	 renderModelHingeB->initRenderModel();

	 root->addChild(renderModelHingeB->getRootNode());

	 boost::shared_ptr<HingeModel> hingeC(new HingeModel(odeWorld, odeSpace,
			 "Hinge3"));
	 hingeC->initModel();
	 hingeC->setRootPosition(osg::Vec3(0.2, 0.3, 1));

	 boost::shared_ptr<HingeRenderModel> renderModelHingeC(
	 new HingeRenderModel(hingeC));
	 renderModelHingeC->initRenderModel();

	 root->addChild(renderModelHingeC->getRootNode());

	 // Active hinge model

	 boost::shared_ptr<ActiveHingeModel> activeHingeA(
	 new ActiveHingeModel(odeWorld, odeSpace, "AHinge"));
	 activeHingeA->initModel();
	 activeHingeA->setRootPosition(osg::Vec3(0.2, 0.4, 1));

	 boost::shared_ptr<ActiveHingeRenderModel> renderModelActiveHingeA(
	 new ActiveHingeRenderModel(activeHingeA));
	 renderModelActiveHingeA->initRenderModel();

	 root->addChild(renderModelActiveHingeA->getRootNode());

	 // Passive wheel model

	 boost::shared_ptr<PassiveWheelModel> passiveWheelA(
	 new PassiveWheelModel(odeWorld, odeSpace, "Pwheel", inMm(30)));
	 passiveWheelA->initModel();
	 passiveWheelA->setRootPosition(osg::Vec3(0.3, 0.1, 0.5));

	 boost::shared_ptr<PassiveWheelRenderModel> renderModelPassiveWheelA(
	 new PassiveWheelRenderModel(passiveWheelA));
	 renderModelPassiveWheelA->initRenderModel();

	 root->addChild(renderModelPassiveWheelA->getRootNode());


	 // Active wheel model

	 boost::shared_ptr<ActiveWheelModel> activeWheelA(
	 new ActiveWheelModel(odeWorld, odeSpace, "Awheel", inMm(50)));
	 activeWheelA->initModel();
	 activeWheelA->setRootPosition(osg::Vec3(0.3, 0.2, 1));

	 boost::shared_ptr<ActiveWheelRenderModel> renderModelActiveWheelA(
	 new ActiveWheelRenderModel(activeWheelA));
	 renderModelActiveWheelA->initRenderModel();

	 root->addChild(renderModelActiveWheelA->getRootNode());




	 boost::shared_ptr<ActiveWhegModel> activeWhegA(
	 	 new ActiveWhegModel(odeWorld, odeSpace, "AWheg", inMm(44)));
	 activeWhegA->initModel();
	 activeWhegA->setRootPosition(osg::Vec3(0.3, 0.3, 1));

		boost::shared_ptr<ActiveWhegRenderModel> renderModelActiveWhegA(
		new ActiveWhegRenderModel(activeWhegA));
		renderModelActiveWhegA->initRenderModel();

		root->addChild(renderModelActiveWhegA->getRootNode());



	 // Cardan model

	 boost::shared_ptr<CardanModel> cardanModelA(
	 new CardanModel(odeWorld, odeSpace, "Cardan"));
	 cardanModelA->initModel();
	 cardanModelA->setRootPosition(osg::Vec3(0.4, 0.1, 1));

	 boost::shared_ptr<CardanRenderModel> renderModelCardanA(
	 new CardanRenderModel(cardanModelA));
	 renderModelCardanA->initRenderModel();

	 root->addChild(renderModelCardanA->getRootNode());

	 // Active Cardan model

	 boost::shared_ptr<ActiveCardanModel> activeCardanModelA(
	 new ActiveCardanModel(odeWorld, odeSpace, "ACardan"));
	 activeCardanModelA->initModel();
	 activeCardanModelA->setRootPosition(osg::Vec3(0.4, 0.2, 1));

	 boost::shared_ptr<ActiveCardanRenderModel> renderModelActiveCardanA(
	 new ActiveCardanRenderModel(activeCardanModelA));
	 renderModelActiveCardanA->initRenderModel();

	 root->addChild(renderModelActiveCardanA->getRootNode());

	 // Parametric model

	boost::shared_ptr<ParametricBrickModel> paramModelA(
	 new ParametricBrickModel(odeWorld, odeSpace, "P1", 0.01, osg::inDegrees(45.0), osg::inDegrees(45.0)));
	 paramModelA->initModel();
	 paramModelA->setRootPosition(osg::Vec3(0.4, 0.3, 1));

	 boost::shared_ptr<ParametricBrickRenderModel> renderParamModelA(
	 new ParametricBrickRenderModel(paramModelA));
	 renderParamModelA->initRenderModel();

	 root->addChild(renderParamModelA->getRootNode());

	 boost::shared_ptr<ParametricBrickModel> paramModelB(
	 new ParametricBrickModel(odeWorld, odeSpace, "P2", 0.02, osg::inDegrees(30.0), osg::inDegrees(60.0)));
	 paramModelB->initModel();
	 paramModelB->setRootPosition(osg::Vec3(0.4, 0.4, 1));

	 boost::shared_ptr<ParametricBrickRenderModel> renderParamModelB(
	 new ParametricBrickRenderModel(paramModelB));
	 renderParamModelB->initRenderModel();

	 root->addChild(renderParamModelB->getRootNode());


	 // Rotate Joint
	 boost::shared_ptr<RotateJointModel> rotateJointA(
	 new RotateJointModel(odeWorld, odeSpace, "R1"));
	 rotateJointA->initModel();
	 rotateJointA->setRootPosition(osg::Vec3(0.5, 0.1, 1));

	 boost::shared_ptr<RotateJointRenderModel> renderRotateJointA(
	 new RotateJointRenderModel(rotateJointA));
	 renderRotateJointA->initRenderModel();

	 root->addChild(renderRotateJointA->getRootNode());



	 // Touch Sensor
	 boost::shared_ptr<TouchSensorModel> touchSensorA(
	 new TouchSensorModel(odeWorld, odeSpace, "T1"));
	 touchSensorA->initModel();
	 touchSensorA->setRootPosition(osg::Vec3(0.5, 0.2, 1));

	 boost::shared_ptr<TouchSensorRenderModel> renderTouchSensorA(
	 new TouchSensorRenderModel(touchSensorA));
	 renderTouchSensorA->initRenderModel();

	 root->addChild(renderTouchSensorA->getRootNode());

	 std::vector<boost::shared_ptr<Sensor> > touchSensorsA;
	 touchSensorA->getSensors(touchSensorsA);

	 touchSensors.push_back(boost::dynamic_pointer_cast<TouchSensor>(touchSensorsA[0]));
	 touchSensors.push_back(boost::dynamic_pointer_cast<TouchSensor>(touchSensorsA[1]));


	 //connect(activeHingeA, ActiveHingeModel::SLOT_A, modelA,
	 //	 	 CoreComponentModel::LEFT_FACE_SLOT, 0);


	//renderModelC->setColor(osg::Vec4(1, 0, 0, 1));
	//renderModelB->setColor(osg::Vec4(1, 1, 1, 1));
	//renderModelHingeA->setColor(osg::Vec4(0, 0, 1, 1));
	// TEST CHAINING

	 /*connect(activeCardanModelA, ActiveCardanModel::SLOT_A, modelA,
	 	 	 CoreComponentModel::LEFT_FACE_SLOT, 0);*/

	/* connect(rotateJointA, RotateJointModel::SLOT_A, modelA,
	 	 CoreComponentModel::LEFT_FACE_SLOT, 0);

	 connect(activeCardanModelA, ActiveCardanModel::SLOT_A, rotateJointA,
			 RotateJointModel::SLOT_B, 0);*/

	 /*connect(touchSensorA, TouchSensorModel::SLOT_A, activeCardanModelA,
			 ActiveCardanModel::SLOT_B, 0);*/


/*
	connect(modelD, CoreComponentModel::TOP_FACE_SLOT, modelC,
	 CoreComponentModel::BOTTOM_FACE_SLOT, 0);

	 connect(hingeA, HingeModel::SLOT_A, modelD,
	 CoreComponentModel::BOTTOM_FACE_SLOT, 0);

	 connect(modelA, CoreComponentModel::LEFT_FACE_SLOT, modelC,
	 CoreComponentModel::TOP_FACE_SLOT, 0);

	 connect(hingeB, HingeModel::SLOT_A, modelA,
	 CoreComponentModel::BOTTOM_FACE_SLOT, 0); // RIGHT_FACE_SLOT

	 connect(hingeC, HingeModel::SLOT_A, modelA,
	 CoreComponentModel::TOP_FACE_SLOT, 0);

	 connect(activeHingeA, ActiveHingeModel::SLOT_A, hingeC, HingeModel::SLOT_B,
	 0);

	 connect(modelE, CoreComponentModel::LEFT_FACE_SLOT, activeHingeA,
	 ActiveHingeModel::SLOT_B, 0);

	 connect(passiveWheelA, PassiveWheelModel::SLOT_A, modelE,
	 CoreComponentModel::RIGHT_FACE_SLOT, 0);

	 connect(cardanModelA, CardanModel::SLOT_A, modelE,
	 CoreComponentModel::TOP_FACE_SLOT, 0);

	 connect(activeCardanModelA, CardanModel::SLOT_A, modelE,
	 CoreComponentModel::BOTTOM_FACE_SLOT, 0);

	 connect(modelB, CoreComponentModel::LEFT_FACE_SLOT, hingeB,
	 HingeModel::SLOT_B, 0);

	 connect(paramModelA, ParametricBrickModel::SLOT_A, modelB,
	 CoreComponentModel::FRONT_FACE_SLOT, 0);

	 connect(paramModelB, ParametricBrickModel::SLOT_A, modelB,
	 CoreComponentModel::BOTTOM_FACE_SLOT, 0);

	 connect(activeWheelA, ActiveWheelModel::SLOT_A, modelB,
	 CoreComponentModel::TOP_FACE_SLOT, 0);

	 connect(rotateJointA, RotateJointModel::SLOT_B, modelD,
	 CoreComponentModel::LEFT_FACE_SLOT, 0);

	 connect(activeWhegA, ActiveWhegModel::SLOT_A, modelD,
	 CoreComponentModel::RIGHT_FACE_SLOT, 0);

	 connect(touchSensorA, TouchSensorModel::SLOT_A, modelD,
	 CoreComponentModel::FRONT_FACE_SLOT, 0);*/

	/* connect(lightSensorA, LightSensorModel::SLOT_A, modelB,
	 CoreComponentModel::RIGHT_FACE_SLOT, 0);

	 connect(lightSensorB, LightSensorModel::SLOT_A, modelB,
	 CoreComponentModel::BACK_FACE_SLOT, 0);*/



	 //------------------------------------------------

	/*

	 connect(activeCardanModelA, ActiveCardanModel::SLOT_A, modelC,
	 CoreComponentModel::TOP_FACE_SLOT, 0);

	 connect(modelB, CoreComponentModel::TOP_FACE_SLOT, activeCardanModelA, ActiveCardanModel::SLOT_B, 0); */

	//------------------------------------------------
	/*std::vector<boost::shared_ptr<Motor> > wheelMotors;
	 activeWhegA->getMotors(wheelMotors);
	 boost::dynamic_pointer_cast<ServoMotor>(wheelMotors[0])->setVelocity(10000);

	 activeWheelA->getMotors(wheelMotors);
	 boost::dynamic_pointer_cast<ServoMotor>(wheelMotors[0])->setVelocity(10000);


	 setupCollision(touchSensors);

	 */

	// ---------------------------------------
	// Light Sensor debug
	// ---------------------------------------

	//std::vector<boost::shared_ptr<LightSensorModel> > lightSensors;

	// osg::Vec3 sphereCenter(100, 100, 100);

	// 1. Create a sphere with light sensors on its surface, pointing towards the center of the sphere
	/*float interval = 10;
	float radius = 100;

	//for (double phi = -(90-interval*3); phi <= (90-interval*3); phi += interval) {

	double phi = 90;
	double theta = 180;

	osg::Quat initRot;
	initRot.makeRotate(osg::inDegrees(180.0), osg::Vec3(0, 0, 1));



		osg::Quat rotationPhi;
		rotationPhi.makeRotate(osg::inDegrees(-(90-phi)), osg::Vec3(0, 1, 0));

		//for (double theta = 0; theta < 360; theta += interval) {

			// Generate a ray with origin in the sensor position, passing at a point at specified altitude and azimut
			osg::Quat rotationTheta;
			rotationTheta.makeRotate(osg::inDegrees(theta), osg::Vec3(0, 0, 1));

			// Find center
			dReal x = radius * sin(osg::inDegrees(phi)) * cos(osg::inDegrees(theta));
			dReal y = radius * sin(osg::inDegrees(phi)) * sin(osg::inDegrees(theta));
			dReal z = radius * cos(osg::inDegrees(phi));

			boost::shared_ptr<LightSensorModel> lightSensorA(
						new LightSensorModel(odeWorld, odeSpace, id, false));
			lightSensorA->initModel();
			lightSensorA->setRootPosition(sphereCenter + osg::Vec3(x, y, z));
			lightSensorA->setRootAttitude(initRot * rotationPhi * rotationTheta);

			lightSensors.push_back(lightSensorA);

			boost::shared_ptr<LightSensorRenderModel> renderLightSensorA(
					new LightSensorRenderModel(lightSensorA, false));
			renderLightSensorA->initRenderModel();

			root->addChild(renderLightSensorA->getRootNode());
		//}

	//} */

	// Light Sensor
	/*boost::shared_ptr<LightSensorModel> lightSensorA(
			new LightSensorModel(odeWorld, odeSpace));
	lightSensorA->initModel();
	lightSensorA->setRootPosition(osg::Vec3(200, 200, 600));

	boost::shared_ptr<LightSensorRenderModel> renderLightSensorA(
			new LightSensorRenderModel(lightSensorA, false));
	renderLightSensorA->initRenderModel();

	root->addChild(renderLightSensorA->getRootNode());

	boost::shared_ptr<LightSensorModel> lightSensorB(
			new LightSensorModel(odeWorld, odeSpace));
	lightSensorB->initModel();
	lightSensorB->setRootPosition(osg::Vec3(150, 150, 600));

	boost::shared_ptr<LightSensorRenderModel> renderLightSensorB(
			new LightSensorRenderModel(lightSensorB, true));
	renderLightSensorB->initRenderModel();

	root->addChild(renderLightSensorB->getRootNode());
*/

	// Light Source
	/*boost::shared_ptr<LightSource> lightSource(
			new LightSource(odeSpace, id, sphereCenter, 100));
	boost::shared_ptr<LightSourceRender> lightSourceRender(
			new LightSourceRender(lightSource, root));

	root->addChild(lightSourceRender->getRootNode());

	std::vector<boost::shared_ptr<LightSource> > lightSources;
	lightSources.push_back(lightSource);*/

	// ---------------------------------------
	// Setup OSG viewer
	// ---------------------------------------

	viewer.setSceneData(root.get());

	osg::Timer_t prevTime = osg::Timer::instance()->tick();
	viewer.realize();

	if (!viewer.getCameraManipulator()
			&& viewer.getCamera()->getAllowEventFocus()) {
		viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	}

	viewer.setReleaseContextAtEndOfFrameHint(false);

	// ---------------------------------------
	// OSG Main Loop
	// ---------------------------------------

	double t = 0;
	double deltaSecs = 0;
	while (!viewer.done()) {

		viewer.frame();

		// Update physics

		const double MAX_STEP = 0.01;
		const osg::Timer_t now = osg::Timer::instance()->tick();
		deltaSecs += osg::Timer::instance()->delta_s(prevTime, now);
		prevTime = now;

		while (deltaSecs > MAX_STEP) {

			setupCollision(touchSensors);

			dSpaceCollide(odeSpace, 0, odeCollisionCallback);

			const double step = std::min(MAX_STEP, deltaSecs);
			deltaSecs -= MAX_STEP;

			dWorldStep(odeWorld, step);

			std::vector<boost::shared_ptr<Motor> > motors;
			//activeHingeA->getMotors(motors);
			//boost::dynamic_pointer_cast<ServoMotor>(motors[0])->setPosition(0.5*sin(t)+0.5);
			t += step/3;



			dJointGroupEmpty(odeContactGroup);


			// Get value from sensors
			/*for (unsigned int i = 0; i < lightSensors.size(); ++i) {
				std::vector<boost::shared_ptr<Sensor> > sensors;
				lightSensors[i]->getSensors(sensors);
				osg::Vec3 pos = lightSensors[i]->getRootPosition();
				osg::Quat attitude = lightSensors[i]->getRootAttitude();
				std::cout << "Sensor (" << pos << ") - ( " <<attitude << ": " << boost::dynamic_pointer_cast<LightSensor>(sensors[0])->read(lightSources, 10) << std::endl;
			}*/

		}

		// ---------------------------------------
		// Temporary
		// ---------------------------------------

	}

	FILE* myFile = fopen("world.txt", "w+");
	dWorldExportDIF(odeWorld, myFile, "myWorld");
	fclose(myFile);

	// ---------------------------------------
	// Finalize
	// ---------------------------------------

	// Destroy ODE space
	dSpaceDestroy(odeSpace);

	// Destroy ODE world
	dWorldDestroy(odeWorld);

	dCloseODE();

}

