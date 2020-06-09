#include <OgreRoot.h>
#include <Bites/OgreApplicationContext.h>
#include <Bites/OgreWindowEventUtilities.h>
#include <OgreEntity.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreSceneNode.h>
#include <OgreFrameListener.h>
#include <OgreTimer.h>
#include <OgreManualObject.h>
#include <OgreCameraMan.h>
#include <OgreTrays.h>
#include <OgreUTFString.h>
#include <HLMS/OgreHlmsManager.h>
#include <HLMS/OgreHlmsPbsMaterial.h>

#include <eos/core/Mesh.hpp>
#include <Eigen/Geometry>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp> 

#include <string>

#include "FrameCapturer.h"
#include "FacialDetector.h"
#include "FacialMorpher.h"

class OgreApp : public OgreBites::ApplicationContext, public OgreBites::InputListener, public OgreBites::TrayListener
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
	void setupUI();
	void parseMaterialsConfig();
	Eigen::Vector3f* calculateNormals(eos::core::Mesh mesh);
	void createHLMSMaterial(Ogre::SubEntity* subEntity, unsigned int id, Ogre::ColourValue color = Ogre::ColourValue::Green, float roughness = 0.5f);
	bool keyPressed(const OgreBites::KeyboardEvent& evt);
	void labelHit(OgreBites::Label *label);
	void changeNextMaterial();
	bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	void runFrame();

	Ogre::Root *mRoot;
	Ogre::HlmsManager *hlmsManager;
	Ogre::Camera* mainCamera;
	OgreBites::CameraMan* camController;
	OgreBites::TrayManager* trayMgr;

	bool mExiting;
	bool isPaused = false;

	boost::shared_ptr<boost::thread> mThread;
	boost::mutex mMutex;
	eos::core::Mesh mesh;
	bool meshNeedsUpdating;

	std::map<std::string, std::vector<std::string>> materials;
	std::pair<int, std::string> curMat; //current material
};