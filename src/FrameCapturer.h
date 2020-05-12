#pragma once
#include <opencv2/opencv.hpp>
#include <eos/cpp17/optional.hpp>
#include <librealsense2/rs.hpp>

class FrameCapturer {
public:
	FrameCapturer();
	void startVideoStream();
	std::pair<cv::Mat, eos::cpp17::optional<rs2::depth_frame>> capture();

private:
	// attributes
	cv::VideoCapture cap;
	rs2::pipeline pipe;

	// methods
	bool isRealSenseAvailable();
	std::pair<rs2::video_frame, rs2::depth_frame> getRealSenseFrames();
	cv::Mat getWebcamFrame();
};