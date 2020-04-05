#include <OgreRoot.h>
#include <Bites/OgreApplicationContext.h>
#include <Bites/OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>
#include <OgreTimer.h>

class MainApplication : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
	MainApplication();

	Ogre::SceneManager* mScene;

	void setup(void);

private:
	bool keyPressed(const OgreBites::KeyboardEvent& evt);

	Ogre::Root *mRoot;
};