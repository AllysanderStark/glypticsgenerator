#include "OgreApp.h"

using namespace std;

// Ogre
OgreApp ogreApp;

int main(int argc, char** argv)
{		
	// Init and start Ogre app
	ogreApp.initApp();
	ogreApp.getRoot()->startRendering();
	return EXIT_SUCCESS;
}