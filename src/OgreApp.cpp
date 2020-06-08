#include <OgreEntity.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreSceneNode.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "OgreApp.h"

using namespace Ogre;
using namespace OgreBites;

using eos::cpp17::optional;
using eos::cpp17::nullopt;

// Available materials:
// GPG_Gold, GPG_Amethyst, GPG_AmethystBG

void OgreApp::setup(void)
{
	ApplicationContext::setup();

	// register for input events
	addInputListener(this);
	mRoot = getRoot();
	mRoot->restoreConfig();

	mRoot->addFrameListener(this);

	mScene = mRoot->createSceneManager();

	// Start the video stream
	frameCap.startVideoStream();

	SceneNode* camNode = mScene->getRootSceneNode()->createChildSceneNode();
	//camNode->setPosition(650, 20, 0);
	//camNode->lookAt(Vector3(0, 20, 0), Node::TS_PARENT);

	// Set-up camera and camera controls
	mainCamera = mScene->createCamera("MainCam");
	mainCamera->setAutoAspectRatio(true);
	mainCamera->setNearClipDistance(0.1);
	camNode->attachObject(mainCamera);

	camController = new CameraMan(camNode);
	camController->setStyle(CameraStyle::CS_ORBIT);
	camController->setYawPitchDist(Radian(Degree(90)), Radian(0.3), 650);
	addInputListener(camController);

	getRenderWindow()->addViewport(mainCamera);

	// Set the lights
	mScene->setAmbientLight(ColourValue(.5, .5, .5));

	Light* light = mScene->createLight("MainLight");
	light->setPosition(800, 100, -100);

	Light* light2 = mScene->createLight("BackLight");
	light2->setPosition(-800, -100, 100);

	// For HLMS shading
	hlmsManager = new HlmsManager(mScene, "General");

	// Setup threading
	meshNeedsUpdating = false;
	assert(!mThread);
	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&OgreApp::runFrame, this)));
}

Eigen::Vector3f* OgreApp::calculateNormals(eos::core::Mesh mesh) {
	Eigen::Vector3f normals[20208]; // SFM
	int timesUsed[20208]; // amount of times the vertex was part of the polygon

	for (auto t : mesh.tvi) {
		// Normals calculation
		Eigen::Vector3f dir0 = mesh.vertices[t[2]] - mesh.vertices[t[0]];
		Eigen::Vector3f dir1 = mesh.vertices[t[0]] - mesh.vertices[t[1]];
		Eigen::Vector3f n = dir0.cross(dir1).normalized();

		// Check normals for all three used vertices, average if already exists
		for (int vi = 0; vi < 3; vi++) {
			if (t[vi] < 20208) { // checking for bounds
				int times = timesUsed[t[vi]];
				normals[t[vi]] = (normals[t[vi]] * times + n) / (times + 1); // additive average
			}
		}

		// increase "used" counter for all three vertices
		for (int vi = 0; vi < 3; vi++) {
			if (t[vi] < 20208) { // checking for bounds
				timesUsed[t[vi]] += 1;
			}
		}
	}

	return normals;
}

void OgreApp::addMesh(eos::core::Mesh mesh) {
	// Colors for albedo
	auto amethystColor = ColourValue(0.377, 0.27, 0.384);
	auto goldColor = ColourValue(1.0, 0.96, 0.656);

	int vertexCount = mesh.vertices.size();
	auto normals = calculateNormals(mesh);

	ManualObject* man = mScene->createManualObject("profile");
	man->setDynamic(true);
	man->estimateVertexCount(vertexCount);
	man->begin("GPG_Gold");

	for (int i = 0; i < vertexCount; i++) {
		auto v = mesh.vertices[i];
		man->position(v[0], v[1] + 5.0f, v[2] - 5.0f);
		auto n = normals[i]; // previously calculated normal for the vertex
		man->normal(n[0], n[1], n[2]);
	}

	for (auto t : mesh.tvi) {
		man->triangle(t[0], t[1], t[2]);
	}

	man->end();

	SceneNode* profileNode = mScene->getRootSceneNode()->createChildSceneNode();
	//createHLMSMaterial(profileEntity->getSubEntity(0), 0);
	//profileNode->attachObject(profileEntity);
	profileNode->attachObject(man);
	profileNode->scale(0.05, 1, 1);

	// Adding the rest of the head
	Entity* headEntity = mScene->createEntity("head.mesh");
	SceneNode* headNode = mScene->getRootSceneNode()->createChildSceneNode();
	headEntity->setMaterialName("GPG_Gold");
	//createHLMSMaterial(headEntity->getSubEntity(0), 1, amethystColor, 0.9f);
	headNode->attachObject(headEntity);
	headNode->scale(0.05, 1, 1);

	// And the gem's base
	Entity* baseCenterEntity = mScene->createEntity("baseCenter.mesh");
	SceneNode* baseCenterNode = mScene->getRootSceneNode()->createChildSceneNode();
	baseCenterEntity->setMaterialName("GPG_AmethystBG");
	//createHLMSMaterial(baseCenterEntity->getSubEntity(0), 2, amethystColor, 0.9f);
	baseCenterNode->attachObject(baseCenterEntity);
	//baseCenterNode->scale(230, 230, 230);
	baseCenterNode->translate(-5.0f, 40.0f, -130.0f);

	Entity* baseEdgeEntity = mScene->createEntity("baseEdge.mesh");
	SceneNode* baseEdgeNode = mScene->getRootSceneNode()->createChildSceneNode();
	baseEdgeEntity->setMaterialName("GPG_Gold");
	//(baseEdgeEntity->getSubEntity(0), 3, goldColor, 0.9f);//0.255f);
	baseEdgeNode->attachObject(baseEdgeEntity);
	//baseEdgeNode->scale(230, 230, 230);
	baseEdgeNode->translate(-5.0f, 40.0f, -130.0f);

	// Set the overall gem shape as a target for the camera
	camController->setTarget(baseEdgeNode);
}

void OgreApp::updateMesh(eos::core::Mesh mesh) {
	int vertexCount = mesh.vertices.size();
	auto normals = calculateNormals(mesh);

	ManualObject* man = mScene->getManualObject("profile");
	man->beginUpdate(0);

	for (int i = 0; i < vertexCount; i++) {
		auto v = mesh.vertices[i];
		man->position(v[0], v[1] + 5.0f, v[2] - 5.0f);
		auto n = normals[i]; // previously calculated normal for the vertex
		man->normal(n[0], n[1], n[2]);
	}
	
	for (auto t : mesh.tvi) {
		man->triangle(t[0], t[1], t[2]);
	}

	man->end();
}

void OgreApp::addOrUpdateMesh(eos::core::Mesh mesh) {
	if (!mScene->hasManualObject("profile")) {
		addMesh(mesh);
	}
	else {
		updateMesh(mesh);
	}
	meshNeedsUpdating = false;
}

void OgreApp::createHLMSMaterial(SubEntity* subEntity, unsigned int id, ColourValue color, float roughness)
{
	PbsMaterial* pbsMaterial = new PbsMaterial();

	MaterialPtr materialPtr = subEntity->getMaterial();
	String newMaterialName = "Pbs_" + subEntity->getMaterialName() + "_" + StringConverter::toString(id);

	// Some test properties set
	pbsMaterial->setAlbedo(color);

	pbsMaterial->setRoughness(roughness);
	pbsMaterial->setLightRoughnessOffset(0.2);

	float f0 = 0.9f;
	pbsMaterial->setF0(color);

	MaterialPtr newMaterialPtr = subEntity->getMaterial()->clone(newMaterialName);
	newMaterialPtr->removeAllTechniques();
	Pass* pass = newMaterialPtr->createTechnique()->createPass();
	pass->setName("pbs");

	subEntity->setMaterial(newMaterialPtr);

	hlmsManager->bind(subEntity, pbsMaterial, "pbs");
}

bool OgreApp::keyPressed(const OgreBites::KeyboardEvent& evt)
{
	if (evt.keysym.sym == SDLK_ESCAPE)
	{
		mExiting = true;
		exit(EXIT_SUCCESS);
	}
	if (evt.keysym.sym == SDLK_F1) {
		detector.toggleWindow();
	}
	if (evt.keysym.sym == SDLK_F2) {
		isPaused = !isPaused;
	}
	return true;
}

void OgreApp::runFrame() {
	while (!mExiting) {
		if (!isPaused && !meshNeedsUpdating) {
			// Do calculations
			auto data = frameCap.capture();
			auto face = detector.detect(data);
			if (face.has_value())
			{
				auto tempMesh = morpher.morph(face.value(), data.first);

				// Lock mutex
				boost::unique_lock<boost::mutex>* lock = new boost::unique_lock<boost::mutex>(mMutex);

				mesh = tempMesh;
				meshNeedsUpdating = true;

				// Unlock mutex
				delete lock;
			}
		}
	}
}

bool OgreApp::frameRenderingQueued(const FrameEvent& evt) {
	if (mExiting) {
		std::cout << "Exiting the application." << std::endl;
		return false;
	}

	// Lock mutex
	boost::unique_lock<boost::mutex>* lock = new boost::unique_lock<boost::mutex>(mMutex);

	if (meshNeedsUpdating) {
		addOrUpdateMesh(mesh);
	}

	delete lock;

	return true;
}