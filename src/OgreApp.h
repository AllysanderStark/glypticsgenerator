#include <OgreRoot.h>
#include <Bites/OgreApplicationContext.h>
#include <Bites/OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>
#include <OgreTimer.h>
#include <OgreManualObject.h>
#include <HLMS/OgreHlmsManager.h>
#include <HLMS/OgreHlmsPbsMaterial.h>

#include <eos/core/Mesh.hpp>
#include <Eigen/Geometry>

class OgreApp : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
	OgreApp();

	Ogre::SceneManager* mScene;

	void setup(void);
	void addMesh(eos::core::Mesh mesh);
	void updateMesh(eos::core::Mesh mesh);
	void addOrUpdateMesh(eos::core::Mesh mesh);

private:
	void createHLMSMaterial(Ogre::SubEntity* subEntity, unsigned int id);
	bool keyPressed(const OgreBites::KeyboardEvent& evt);

	Ogre::Root *mRoot;
	Ogre::HlmsManager *hlmsManager;
};