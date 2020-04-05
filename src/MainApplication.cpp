#include "MainApplication.h"

#include <OgreEntity.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreSceneNode.h>

#include <iostream>

using namespace Ogre;
using namespace OgreBites;

MainApplication::MainApplication() : ApplicationContext()
{
}

void MainApplication::setup(void)
{
	OgreBites::ApplicationContext::setup();

	// register for input events
	addInputListener(this);
	mRoot = getRoot();
	mRoot->restoreConfig();
	
	mScene = mRoot->createSceneManager();

	Ogre::SceneNode* camNode = mScene->getRootSceneNode()->createChildSceneNode();
	camNode->setPosition(0, 0, 1);
	camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);

	Camera* mCamera = mScene->createCamera("MainCam");
	mCamera->setAutoAspectRatio(true);
	mCamera->setNearClipDistance(0.5);
	camNode->attachObject(mCamera);

	getRenderWindow()->addViewport(mCamera);

	ManualObject* triangle = mScene->createManualObject("Triangle");

	triangle->begin("BaseWhiteNoLighting");
	triangle->position(0, 0.5, 0);
	triangle->colour(ColourValue::Red);
	triangle->position(-0.5, -0.5, 0);
	triangle->colour(ColourValue::Green);
	triangle->position(0.5, -0.5, 0);
	triangle->colour(ColourValue::Blue);
	triangle->end();

	Ogre::SceneNode* triangleNode = mScene->getRootSceneNode()->createChildSceneNode();
	triangleNode->attachObject(triangle);

	mScene->setAmbientLight(ColourValue(.5, .5, .5));

	Light* light = mScene->createLight("MainLight");
	light->setPosition(20, 80, 50);
}

bool MainApplication::keyPressed(const OgreBites::KeyboardEvent& evt)
{
	if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
	{
		getRoot()->queueEndRendering();
	}
	return true;
}