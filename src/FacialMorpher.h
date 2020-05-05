#pragma once

#include <eos/core/Landmark.hpp>
#include <eos/core/LandmarkMapper.hpp>
#include <eos/morphablemodel/MorphableModel.hpp>
#include <eos/morphablemodel/Blendshape.hpp>
#include <eos/fitting/fitting.hpp>
#include <eos/fitting/nonlinear_camera_estimation.hpp>
#include <eos/render/detail/render_detail.hpp>
#include <eos/render/utils.hpp>
#include <eos/cpp17/optional.hpp>
#include <dlib/image_processing/full_object_detection.h>
#include <librealsense2/rs.hpp>

enum ModelType {
	SFM, BFM
};

class FacialMorpher {
public:
	FacialMorpher(ModelType type);
	eos::core::Mesh morph(dlib::full_object_detection face, rs2::video_frame color_frame);

private:
	eos::morphablemodel::MorphableModel morphable_model;
	eos::core::LandmarkMapper landmark_mapper;
	eos::morphablemodel::Blendshapes blendshapes;
	eos::fitting::ModelContour model_contour;
	eos::fitting::ContourLandmarks ibug_contour;
	eos::morphablemodel::EdgeTopology edge_topology;
};