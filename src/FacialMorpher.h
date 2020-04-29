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

using namespace eos;

class FacialMorpher {
public:
	FacialMorpher();
	core::Mesh morph(dlib::full_object_detection face, rs2::video_frame color_frame);

private:
	morphablemodel::MorphableModel morphable_model;
	core::LandmarkMapper landmark_mapper;
	morphablemodel::Blendshapes blendshapes;
	fitting::ModelContour model_contour;
	fitting::ContourLandmarks ibug_contour;
	morphablemodel::EdgeTopology edge_topology;
};