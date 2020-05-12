#pragma once

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <eos/cpp17/optional.hpp>

class FacialDetector {
public:
	dlib::image_window win;

	FacialDetector();
	dlib::full_object_detection detect(std::pair<cv::Mat, eos::cpp17::optional<rs2::depth_frame>> data);

private:
	dlib::frontal_face_detector detector;
	dlib::shape_predictor predictor;
};