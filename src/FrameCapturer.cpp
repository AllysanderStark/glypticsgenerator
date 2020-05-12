#include "FrameCapturer.h"

using namespace cv;

FrameCapturer::FrameCapturer() {

}

void FrameCapturer::startVideoStream() {
	if (isRealSenseAvailable()) {
		// Initialize config for RealSense pipeline
		rs2::config cfg;

		//Enable both RGB and depth streams
		cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16);
		cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8); //BGR for OpenCV

		// Start the pipeline i.e. streaming from RS camera
		pipe.start(cfg);
	}
	else {
		if (!cap.open(0)) // attempt to start videostream from default device
			return; // error
	}
}

// If a RealSense device is available, returns a pair of cv::Mat and rs2::depth_frame
// otherwise the second element of the pair is nullopt
std::pair<Mat, eos::cpp17::optional<rs2::depth_frame>> FrameCapturer::capture() {
	if (isRealSenseAvailable()) {
		auto frames = getRealSenseFrames();
		Mat color(Size(640, 480), CV_8UC3, (void*)frames.first.get_data(), Mat::AUTO_STEP);
		return std::make_pair(color, frames.second);
	}
	else {
		return std::make_pair(getWebcamFrame(), eos::cpp17::nullopt);
	}
}

bool FrameCapturer::isRealSenseAvailable() {
	rs2::context ctx;
	rs2::device_list devices = ctx.query_devices();
	return devices.size() > 0;
}

std::pair<rs2::video_frame, rs2::depth_frame> FrameCapturer::getRealSenseFrames() {
	rs2::align align_to_color(RS2_STREAM_COLOR);

	// Block the application until a frameset is available
	rs2::frameset frameset = pipe.wait_for_frames();
	frameset = align_to_color.process(frameset);

	// Get both RGB and depth frame for dlib down the line
	auto depth_frame = frameset.get_depth_frame();
	auto color_frame = frameset.get_color_frame();

	return std::make_pair(color_frame, depth_frame);
}

Mat FrameCapturer::getWebcamFrame() {
	Mat frame;
	cap >> frame;
	return frame;
}