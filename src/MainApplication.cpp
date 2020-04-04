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

void MainApplication::run()
{
	OgreBites::ApplicationContext::setup();
	//std::cout << "setup" << std::endl;

	// register for input events
	addInputListener(this);
	//std::cout << "addInputListener" << std::endl;
	mRoot = getRoot();
	//std::cout << "getRoot" << std::endl;
	mRoot->restoreConfig();
	//std::cout << "restoreConfig" << std::endl;

	//mWindow = mRoot->initialise(true, "Glyptics Portrait Generator");
	
	mScene = mRoot->createSceneManager();
	//std::cout << "createSceneManager" << std::endl;

	Ogre::SceneNode* camNode = mScene->getRootSceneNode()->createChildSceneNode();
	camNode->setPosition(0, 0, 15);
	camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);

	Camera* mCamera = mScene->createCamera("MainCam");
	mCamera->setAutoAspectRatio(true);
	mCamera->setNearClipDistance(0.5);
	camNode->attachObject(mCamera);

	getRenderWindow()->addViewport(mCamera);

	//Entity* ogreEntity = mSceneMgr->createEntity("ogrehead.mesh");

	//SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	//ogreNode->attachObject(ogreEntity);

	ManualObject* triangle = mScene->createManualObject("Triangle");

	// ~ glBegin, glVertex, glEnd
	// "BaseWhiteNoLighting" is a built-in name for a basic non-lit material
	triangle->begin("BaseWhiteNoLighting");
	triangle->position(0, 0.5, 0);            // ~ glVertex.
											  // Contrary to OpenGL we *first* must create the vertex
	triangle->colour(ColourValue::Red); // .. and then provide its attributes such as color (~ glColor)
	triangle->position(-0.5, -0.5, 0);
	triangle->colour(ColourValue::Green);
	triangle->position(0.5, -0.5, 0);
	triangle->colour(ColourValue::Blue);
	triangle->end();

	mScene->setAmbientLight(ColourValue(.5, .5, .5));

	Light* light = mScene->createLight("MainLight");
	light->setPosition(20, 80, 50);

	mRoot->startRendering();
}

bool MainApplication::keyPressed(const OgreBites::KeyboardEvent& evt)
{
	if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
	{
		getRoot()->queueEndRendering();
	}
	return true;
}