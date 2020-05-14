#include <OgreEntity.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreSceneNode.h>

#include <iostream>

#include "OgreApp.h"

#define MATERIAL "BaseWhiteNoLighting"

using namespace Ogre;
using namespace OgreBites;

OgreApp::OgreApp() : ApplicationContext("Glyptics Portrait Generator Ogre")
{
}

void OgreApp::setup(void)
{
	ApplicationContext::setup();

	// register for input events
	addInputListener(this);
	mRoot = getRoot();
	mRoot->restoreConfig();

	mScene = mRoot->createSceneManager();

	SceneNode* camNode = mScene->getRootSceneNode()->createChildSceneNode();
	camNode->setPosition(700, 0, -75.0);
	camNode->lookAt(Vector3(0, 0, -75.0), Node::TS_PARENT);

	Camera* mCamera = mScene->createCamera("MainCam");
	mCamera->setAutoAspectRatio(true);
	mCamera->setNearClipDistance(0.1);
	camNode->attachObject(mCamera);

	getRenderWindow()->addViewport(mCamera);

	mScene->setAmbientLight(ColourValue(.5, .5, .5));

	Light* light = mScene->createLight("MainLight");
	light->setPosition(20, 80, 50);

	// For HLMS shading
	hlmsManager = new HlmsManager(mScene, "General");
}

void OgreApp::addMesh(eos::core::Mesh mesh) {
	ManualObject* man = mScene->createManualObject("profile");

	man->estimateVertexCount(mesh.vertices.size());
	man->begin("BaseWhite");

	for (auto v : mesh.vertices) {
		man->position(v[0], v[1] + 5.0f, v[2] - 5.0f);
	}

	for (auto t : mesh.tvi) {
		man->triangle(t[0], t[1], t[2]);
		// Normals calculation
		Eigen::Vector3f dir0 = mesh.vertices[t[2]] - mesh.vertices[t[0]];
		Eigen::Vector3f dir1 = mesh.vertices[t[0]] - mesh.vertices[t[1]];
		Eigen::Vector3f n = dir0.cross(dir1).normalized();
		man->normal(n[0], n[1], n[2]);
	}
	man->end();

	//man->convertToMesh("profileEntity");

	//Entity* profileEntity = mScene->createEntity("profileEntity");

	SceneNode* profileNode = mScene->getRootSceneNode()->createChildSceneNode();
	//createHLMSMaterial(profileEntity->getSubEntity(0), 0);
	//profileNode->attachObject(profileEntity);
	profileNode->attachObject(man);

	// Adding the rest of the head
	Entity* headEntity = mScene->createEntity("head.mesh");

	SceneNode* headNode = mScene->getRootSceneNode()->createChildSceneNode();
	createHLMSMaterial(headEntity->getSubEntity(0), 1);
	headNode->attachObject(headEntity);
}

void OgreApp::updateMesh(eos::core::Mesh mesh) {
	ManualObject* man = mScene->getManualObject("profile");

	man->beginUpdate(0);

	for (auto v : mesh.vertices) {
		man->position(v[0], v[1], v[2]);
	}
}

void OgreApp::addOrUpdateMesh(eos::core::Mesh mesh) {
	if (!mScene->hasManualObject("profile")) {
		addMesh(mesh);
	}
	else {
		updateMesh(mesh);
	}
}

void OgreApp::createHLMSMaterial(SubEntity* subEntity, unsigned int id)
{
	PbsMaterial* pbsMaterial = new PbsMaterial();

	MaterialPtr materialPtr = subEntity->getMaterial();
	String newMaterialName = "Pbs_" + subEntity->getMaterialName() + "_" + StringConverter::toString(id);

	// Some test properties set
	pbsMaterial->setAlbedo(ColourValue::Green);

	pbsMaterial->setRoughness(0.6f);
	pbsMaterial->setLightRoughnessOffset(0.2);

	float f0 = 0.9f;
	pbsMaterial->setF0(ColourValue(f0, f0, f0));

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
		getRoot()->queueEndRendering();
	}
	if (evt.keysym.sym == SDLK_LEFT)
	{
		std::cout << "Left arrow pressed" << std::endl;
	}
	return true;
}