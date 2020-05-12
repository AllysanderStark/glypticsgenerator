#pragma once

#include "eos/core/Image.hpp"
#include "eos/core/image/opencv_interop.hpp"
#include "eos/core/Landmark.hpp"
#include "eos/core/LandmarkMapper.hpp"
#include "eos/core/read_pts_landmarks.hpp"
#include "eos/fitting/fitting.hpp"
#include "eos/morphablemodel/Blendshape.hpp"
#include "eos/morphablemodel/MorphableModel.hpp"
#include "eos/render/draw_utils.hpp"
#include "eos/render/texture_extraction.hpp"
#include "eos/cpp17/optional.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <dlib/image_processing/full_object_detection.h>

enum ModelType {
	SFM, BFM
};

class FacialMorpher {
public:
	FacialMorpher(ModelType type);
	eos::core::Mesh morph(dlib::full_object_detection face, cv::Mat image);

private:
	ModelType type;

	eos::morphablemodel::MorphableModel morphable_model;
	eos::core::LandmarkMapper landmark_mapper;
	eos::morphablemodel::Blendshapes blendshapes;
	eos::fitting::ModelContour model_contour;
	eos::fitting::ContourLandmarks ibug_contour;
	eos::morphablemodel::EdgeTopology edge_topology;
};