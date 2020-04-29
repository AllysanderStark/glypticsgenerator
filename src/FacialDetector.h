#pragma once

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

using namespace dlib;

class FacialDetector {
public:
	image_window win;

	FacialDetector();
	full_object_detection detect(rs2::video_frame color_frame, rs2::depth_frame depth_frame);

private:
	frontal_face_detector detector;
	shape_predictor predictor;
};