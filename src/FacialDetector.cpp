#include <boost/format.hpp>
#include "FacialDetector.h"

using namespace dlib;
using namespace cv;

FacialDetector::FacialDetector() {
	win.set_title("Glyptics Portrait Generator: dlib");
	
	winShown = false;
	win.hide();

	detector = get_frontal_face_detector();
	deserialize("res/landmarks.dat") >> predictor;
}

eos::cpp17::optional<full_object_detection> FacialDetector::detect(std::pair<cv::Mat, eos::cpp17::optional<rs2::depth_frame>> data) {
	// Data extraction
	cv_image<bgr_pixel> cimg(data.first);
	bool hasDepth = data.second.has_value();

	win.set_image(cimg);
	
	// Detection
	std::vector<dlib::rectangle> faces = detector(cimg);

	// Return nullopt if no faces are detected
	if (faces.empty()) 
	{
		return eos::cpp17::nullopt;
	}

	std::vector<full_object_detection> shapes;
	for (unsigned long i = 0; i < faces.size(); ++i)
		shapes.push_back(predictor(cimg, faces[i]));

	// Overlay
	Mat ov = Mat::zeros(480, 640, CV_8UC3);
	cv_image<bgr_pixel> overlay(ov);
	win.clear_overlay();

	win.add_overlay(render_face_detections(shapes));
	auto face = shapes[0];
	point p;
	for (unsigned long i = 0; i < face.num_parts(); i++) {
		p = face.part(i);
		dlib::rectangle rect = dlib::rectangle(p);
		// Add depth labels if available
		if (hasDepth) {
			auto depth = data.second.value();
			float z = depth.get_distance(p.x(), p.y());

			// If there is no depth data for the landmark, try nearby pixels 
			if (z == 0) {
				for (int ax = -8; ax <= 8 && z == 0; ax++) {
					for (int ay = -8; ay <= 8 && z == 0; ay++) {
						z = depth.get_distance(p.x() + ax, p.y() + ay);
					}
				}
			}
			std::string z_label = (boost::format("%1$.2f") % z).str();
			win.add_overlay(image_window::overlay_rect(rect, rgb_pixel(255, 0, 0), z_label));
		}
	}
	
	return face;
}

void FacialDetector::toggleWindow() {
	if (winShown) {
		win.hide();
		winShown = false;
	}
	else {
		win.show();
		winShown = true;
	}
}