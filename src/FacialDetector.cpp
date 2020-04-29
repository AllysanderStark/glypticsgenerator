#include <boost/format.hpp>
#include "FacialDetector.h"

using namespace dlib;
using namespace cv;

FacialDetector::FacialDetector() {
	win.set_title("Glyptics Portrait Generator dlib");
	win.show();
	detector = get_frontal_face_detector();
	deserialize("res/landmarks.dat") >> predictor;
}

full_object_detection FacialDetector::detect(rs2::video_frame color_frame, rs2::depth_frame depth_frame) {
	// Dlib + OpenCV
	Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
	Mat depth(Size(640, 480), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

	cv_image<rgb_pixel> cimg(color);
	cv_image<rgb_pixel> dimg(depth);

	// Detection
	std::vector<dlib::rectangle> faces = detector(cimg);
	std::vector<full_object_detection> shapes;
	for (unsigned long i = 0; i < faces.size(); ++i)
		shapes.push_back(predictor(cimg, faces[i]));

	// Overlay
	Mat ov = Mat::zeros(480, 640, CV_8UC3);
	cv_image<rgb_pixel> overlay(ov);
	win.clear_overlay();

	win.add_overlay(render_face_detections(shapes));
	auto face = shapes[0];
	point p;
	for (unsigned long i = 0; i < face.num_parts(); i++) {
		p = face.part(i);
		dlib::rectangle rect = dlib::rectangle(p);
		float z = depth_frame.get_distance(p.x(), p.y());
		// If there is no depth data for the landmark, try nearby pixels 
		if (z == 0) {
			for (int ax = -8; ax <= 8 && z == 0; ax++) {
				for (int ay = -8; ay <= 8 && z == 0; ay++) {
					z = depth_frame.get_distance(p.x() + ax, p.y() + ay);
				}
			}
		}
		std::string z_label = (boost::format("%1$.2f") % z).str();
		win.add_overlay(image_window::overlay_rect(rect, rgb_pixel(255, 0, 0), z_label));
	}

	win.set_image(cimg);

	return face;
}