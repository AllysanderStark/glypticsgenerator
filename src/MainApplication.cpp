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

MainApplication::MainApplication() : ApplicationContext("Glyptics Portrait Generator")
{
}

void MainApplication::setup(void)
{
	ApplicationContext::setup();

	// register for input events
	addInputListener(this);
	mRoot = getRoot();
	mRoot->restoreConfig();

	mScene = mRoot->createSceneManager();

	SceneNode* camNode = mScene->getRootSceneNode()->createChildSceneNode();
	camNode->setPosition(0, 0, 80);
	camNode->lookAt(Vector3(0, 0, -300), Node::TS_PARENT);

	Camera* mCamera = mScene->createCamera("MainCam");
	mCamera->setAutoAspectRatio(true);
	mCamera->setNearClipDistance(0.5);
	camNode->attachObject(mCamera);

	getRenderWindow()->addViewport(mCamera);

	mScene->setAmbientLight(ColourValue(.5, .5, .5));

	Light* light = mScene->createLight("MainLight");
	light->setPosition(20, 80, 50);

	/*ManualObject* triangle = mScene->createManualObject("Triangle");

	triangle->begin("BaseWhiteNoLighting");
	triangle->position(0, 0.5, 0);
	triangle->colour(ColourValue::Red);
	triangle->position(-0.5, -0.5, 0);
	triangle->colour(ColourValue::Green);
	triangle->position(0.5, -0.5, 0);
	triangle->colour(ColourValue::Blue);
	triangle->end();

	Ogre::SceneNode* triangleNode = mScene->getRootSceneNode()->createChildSceneNode();
	triangleNode->attachObject(triangle);*/

	Ogre::Entity* ogreEntity = mScene->createEntity("ogrehead.mesh");

	Ogre::SceneNode* ogreNode = mScene->getRootSceneNode()->createChildSceneNode();
	ogreNode->attachObject(ogreEntity);
}

void MainApplication::add_mesh(pcl::PolygonMesh mesh) {
	


	/*
	ManualObject* man = mScene->createManualObject("profile");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	man->estimateVertexCount(cloud->size());
	man->begin("BaseWhiteNoLighting");

	for (auto p: cloud->points) {
		man->position(p.x, p.y, p.z);
		man->colour(ColourValue::White);
	}

	for (auto t : mesh.polygons) {
		auto v = t.vertices;
		man->triangle(v[0], v[1], v[2]);
	}
	man->end();

	SceneNode* profileNode = mScene->getRootSceneNode()->createChildSceneNode();
	profileNode->attachObject(man);
	*/
}

bool MainApplication::keyPressed(const OgreBites::KeyboardEvent& evt)
{
	if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
	{
		getRoot()->queueEndRendering();
	}
	return true;
}