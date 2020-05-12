#include "FacialMorpher.h"

using namespace eos;

FacialMorpher::FacialMorpher(ModelType type) {
	this->type = type;
	// Instantiating models based on type
	switch (type) {
	case SFM:
		morphable_model = morphablemodel::load_model("res/sfm_shape_3448.bin");
		landmark_mapper = core::LandmarkMapper("res/ibug_to_sfm.txt");
		model_contour = fitting::ModelContour::load("res/sfm_model_contours.json");
		ibug_contour = fitting::ContourLandmarks::load("res/ibug_to_sfm.txt");
		edge_topology = morphablemodel::load_edge_topology("res/sfm_3448_edge_topology.json");
		blendshapes = morphablemodel::load_blendshapes("res/expression_blendshapes_3448.bin");
		break;
	case BFM:
		morphable_model = morphablemodel::load_model("res/bfm2017-1_bfm_nomouth.bin");
		landmark_mapper = core::LandmarkMapper("res/ibug_to_bfm2017-1_bfm_nomouth.txt");
		model_contour = fitting::ModelContour::load("res/bfm2017-1_bfm_nomouth_model_contours.json");
		ibug_contour = fitting::ContourLandmarks::load("res/ibug_to_bfm2017-1_bfm_nomouth.txt");
		edge_topology = morphablemodel::load_edge_topology("res/bfm2017-1_bfm_nomouth_edge_topology.json");
		//no blendshapes for BFM2017
		break;
	}
}

core::Mesh FacialMorpher::morph(dlib::full_object_detection face, cv::Mat image) {
	std::vector<Eigen::Vector4f> model_points; // 3d morphable model points
	std::vector<int> vertex_indices;
	std::vector<Eigen::Vector2f> image_points; // corresponding 2d coordinates

	core::Mesh mesh;
	fitting::RenderingParameters rendering_params;

	//Without expressions
	if (blendshapes.empty()) {
		for (int i = 0; i < face.num_parts(); i++) {
			auto p = face.part(i);

			auto mapped_name = landmark_mapper.convert(std::to_string(i + 1));
			if (!mapped_name)
			{
				continue;
			}

			int vertex_idx = std::stoi(mapped_name.value());
			auto vertex = morphable_model.get_shape_model().get_mean_at_point(vertex_idx);
			model_points.emplace_back(Eigen::Vector4f(vertex.x(), vertex.y(), vertex.z(), 1.0f));
			vertex_indices.emplace_back(vertex_idx);
			image_points.emplace_back(p.x(), p.y());
		}


		fitting::ScaledOrthoProjectionParameters pose = eos::fitting::estimate_orthographic_projection_linear(image_points, model_points, true, image.rows);
		rendering_params = fitting::RenderingParameters(pose, image.cols, image.rows);

		const Eigen::Matrix<float, 3, 4> affine_from_ortho = eos::fitting::get_3x4_affine_camera_matrix(rendering_params, image.cols, image.rows);

		const std::vector<float> fitted_coeffs = eos::fitting::fit_shape_to_landmarks_linear(morphable_model.get_shape_model(), affine_from_ortho, image_points, vertex_indices, Eigen::VectorXf(), 0.5f);

		mesh = morphable_model.draw_sample(fitted_coeffs, std::vector<float>());
	}
	// With expressions
	else {
		// Converting landmarks for eos
		core::LandmarkCollection<Eigen::Vector2f> landmarks;
		landmarks.reserve(68);

		for (int i = 0; i < face.num_parts(); i++) {
			auto p = face.part(i);

			core::Landmark<Eigen::Vector2f> landmark;
			landmark.name = std::to_string(i + 1);
			landmark.coordinates[0] = p.x();
			landmark.coordinates[1] = p.y();

			landmarks.emplace_back(landmark);
		}

		morphablemodel::MorphableModel morphable_model_with_expressions(
			morphable_model.get_shape_model(),
			blendshapes,
			morphable_model.get_color_model(),
			cpp17::nullopt,
			morphable_model.get_texture_coordinates());

		std::tie(mesh, rendering_params) = fitting::fit_shape_and_pose(
			morphable_model_with_expressions, landmarks, landmark_mapper, image.cols, image.rows, edge_topology,
			ibug_contour, model_contour, 5, cpp17::nullopt, 5.0f);
	}

	// Extract the texture from the image using given mesh and camera parameters:
	//const Eigen::Matrix<float, 3, 4> affine_from_ortho =
	//	fitting::get_3x4_affine_camera_matrix(rendering_params, image.cols, image.rows);
	//const core::Image4u isomap =
	//	render::extract_texture(mesh, affine_from_ortho, core::from_mat(image), true);

	cv::Mat outimg = image.clone();

	// Draw the fitted mesh as wireframe, and save the image:
	render::draw_wireframe(outimg, mesh, rendering_params.get_modelview(), rendering_params.get_projection(),
		fitting::get_opencv_viewport(image.cols, image.rows));
	cv::imwrite("morph.png", outimg);

	// Save the mesh
	core::write_obj(mesh, "morph.obj");

	// Textured mesh:
	//cv::imwrite("textured_morph.isomap.png", core::to_mat(isomap));
	//core::write_textured_obj(mesh, "textured_morph.obj");

	return mesh;
}