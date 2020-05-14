#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>

#include "helpers.h"
#include "OgreApp.h"

#include "FrameCapturer.h"
#include "FacialDetector.h"
#include "FacialMorpher.h"

using namespace std;

// number of frames to be captured (WIP)
#if NDEBUG
#define N_FRAMES 5
#else
#define N_FRAMES 1
#endif

// time measurement
chrono::steady_clock::time_point tBegin;
chrono::steady_clock::time_point tEnd;
void clickStopwatch(string process);

// Ogre
OgreApp ogreApp;

int main(int argc, char** argv)
{		
	// Begin time measurement!
	tBegin = chrono::steady_clock::now();

	// Instantiate modules
	FrameCapturer frameCap;
	FacialDetector detector;
	FacialMorpher morpher(SFM);
	clickStopwatch("Modules initialization");

	// Init Ogre app
	ogreApp.initApp();
	clickStopwatch("Ogre initialization");

	// Start the video stream
	frameCap.startVideoStream();
	clickStopwatch("Video stream started");

	eos::core::Mesh mesh;

	for (int i = 0; i < N_FRAMES; i++) 
	{
		auto data = frameCap.capture();
		clickStopwatch("Frame capture");

		auto face = detector.detect(data);
		clickStopwatch("Landmark detection");

		mesh = morpher.morph(face, data.first);
		clickStopwatch("Face morphing");
	}

	ogreApp.addOrUpdateMesh(mesh);
	clickStopwatch("Mesh addition");

	ogreApp.getRoot()->startRendering();

	return EXIT_SUCCESS;
}

/**
* Snapshots the time measurements
* and prints the results into the console.
*
* @param[in] process The name of the measured process.
*/
void clickStopwatch(string process) {
	tEnd = chrono::steady_clock::now();
	cout << setw(30) << left << process + ": " << chrono::duration_cast<chrono::milliseconds>(tEnd - tBegin).count() << "ms elapsed" << endl;
	tBegin = chrono::steady_clock::now();
}