#include <OgreRoot.h>
#include <Bites/OgreApplicationContext.h>
#include <Bites/OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>
#include <OgreTimer.h>
#include <OgreManualObject.h>
#include <OgreCameraMan.h>
#include <HLMS/OgreHlmsManager.h>
#include <HLMS/OgreHlmsPbsMaterial.h>

#include <eos/core/Mesh.hpp>
#include <Eigen/Geometry>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include "FrameCapturer.h"
#include "FacialDetector.h"
#include "FacialMorpher.h"

class OgreApp : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
	Ogre::SceneManager* mScene;
	FrameCapturer frameCap;
	FacialDetector detector;
	FacialMorpher morpher;

	OgreApp() : OgreBites::ApplicationContext("Glyptics Portrait Generator Ogre"), morpher(SFM), mExiting(false) {}

	void setup(void);
	void addMesh(eos::core::Mesh mesh);
	void updateMesh(eos::core::Mesh mesh);
	void addOrUpdateMesh(eos::core::Mesh mesh);

private:
	Eigen::Vector3f* calculateNormals(eos::core::Mesh mesh);
	void createHLMSMaterial(Ogre::SubEntity* subEntity, unsigned int id, Ogre::ColourValue color = Ogre::ColourValue::Green, float roughness = 0.5f);
	bool keyPressed(const OgreBites::KeyboardEvent& evt);
	bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	void runFrame();

	Ogre::Root *mRoot;
	Ogre::HlmsManager *hlmsManager;
	Ogre::Camera* mainCamera;
	OgreBites::CameraMan* camController;

	bool mExiting;
	bool isPaused = false;

	boost::shared_ptr<boost::thread> mThread;
	boost::mutex mMutex;
	eos::core::Mesh mesh;
	bool meshNeedsUpdating;
};