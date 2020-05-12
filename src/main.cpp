#include <iostream>
#include <chrono>
#include <string>

#include "helpers.h"
#include "OgreApp.h"

#include "FrameCapturer.h"
#include "FacialDetector.h"
#include "FacialMorpher.h"

// time measurement
std::chrono::steady_clock::time_point begin;
std::chrono::steady_clock::time_point end;

// Ogre
OgreApp ogre_app;

int main(int argc, char** argv)
{
	// Instantiate modules
	FrameCapturer frameCap;
	FacialDetector detector;
	FacialMorpher morpher(SFM);

	// Init Ogre app
	ogre_app.initApp();

	// Start the video stream
	frameCap.startVideoStream();

	// Begin time measurement!
	begin = std::chrono::steady_clock::now();

	int i = 0;
	eos::core::Mesh mesh;

	while (i < 5) 
	{
		auto data = frameCap.capture();
		auto face = detector.detect(data);
		mesh = morpher.morph(face, data.first);

		i++;
	}

	ogre_app.add_or_update_mesh(mesh);
	ogre_app.getRoot()->startRendering();

	return EXIT_SUCCESS;
}