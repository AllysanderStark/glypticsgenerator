#include <cstdlib>
#include <exception>
#include <iostream>

#include "OgreApp.h"

using namespace std;

// Ogre
OgreApp ogreApp;

int main(int argc, char** argv)
{
	try {
		// Init and start Ogre app
		ogreApp.initApp();
		ogreApp.getRoot()->startRendering();
	}
	catch (const exception& e) {
		cout << e.what() << endl;
	}
	return EXIT_SUCCESS;
}