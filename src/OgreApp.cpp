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

using namespace Ogre;
using namespace OgreBites;

OgreApp::OgreApp() : ApplicationContext("Glyptics Portrait Generator")
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
	camNode->setPosition(0, 0, 0.8);
	camNode->lookAt(Vector3(0, 0, 0), Node::TS_PARENT);

	Camera* mCamera = mScene->createCamera("MainCam");
	mCamera->setAutoAspectRatio(true);
	mCamera->setNearClipDistance(0.1);
	camNode->attachObject(mCamera);

	getRenderWindow()->addViewport(mCamera);

	mScene->setAmbientLight(ColourValue(.5, .5, .5));

	Light* light = mScene->createLight("MainLight");
	light->setPosition(20, 80, 50);
}

void OgreApp::add_mesh(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	ManualObject* man = mScene->createManualObject("profile");

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
}

bool OgreApp::keyPressed(const OgreBites::KeyboardEvent& evt)
{
	if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
	{
		getRoot()->queueEndRendering();
	}
	return true;
}