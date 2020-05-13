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

// time measurement
chrono::steady_clock::time_point t_begin;
chrono::steady_clock::time_point t_end;
void click_stopwatch(string process);

// Ogre
OgreApp ogre_app;

int main(int argc, char** argv)
{		
	// Begin time measurement!
	t_begin = chrono::steady_clock::now();

	// Instantiate modules
	FrameCapturer frameCap;
	FacialDetector detector;
	FacialMorpher morpher(SFM);
	click_stopwatch("Modules initialization");

	// Init Ogre app
	ogre_app.initApp();
	click_stopwatch("Ogre initialization");

	// Start the video stream
	frameCap.startVideoStream();
	click_stopwatch("Video stream started");

	int i = 0;
	eos::core::Mesh mesh;

	while (i < 5) 
	{
		auto data = frameCap.capture();
		click_stopwatch("Frame capture");

		auto face = detector.detect(data);
		click_stopwatch("Landmark detection");

		mesh = morpher.morph(face, data.first);
		click_stopwatch("Face morphing");

		i++;
	}

	ogre_app.add_or_update_mesh(mesh);
	click_stopwatch("Mesh addition");

	ogre_app.getRoot()->startRendering();

	return EXIT_SUCCESS;
}

/**
* Snapshots the time measurements
* and prints the results into the console.
*
* @param[in] process The name of the measured process.
*/
void click_stopwatch(string process) {
	t_end = chrono::steady_clock::now();
	cout << setw(30) << left << process + ": " << chrono::duration_cast<chrono::milliseconds>(t_end - t_begin).count() << "ms elapsed" << endl;
	t_begin = chrono::steady_clock::now();
}