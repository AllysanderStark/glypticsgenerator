#include "FacialMorpher.h"

FacialMorpher::FacialMorpher(ModelType type) {
	// Instantiating models based on type
	switch (type) {
	case SFM:
		morphable_model = eos::morphablemodel::load_model("res/sfm_shape_3448.bin");
		landmark_mapper = eos::core::LandmarkMapper("res/ibug_to_sfm.txt");
		model_contour = eos::fitting::ModelContour::load("res/sfm_model_contours.json");
		ibug_contour = eos::fitting::ContourLandmarks::load("res/ibug_to_sfm.txt");
		edge_topology = eos::morphablemodel::load_edge_topology("res/sfm_3448_edge_topology.json");
		break;
	case BFM:
		morphable_model = eos::morphablemodel::load_model("res/bfm2017-1_bfm_nomouth.bin");
		landmark_mapper = eos::core::LandmarkMapper("res/ibug_to_bfm2017-1_bfm_nomouth.txt");
		model_contour = eos::fitting::ModelContour::load("res/bfm2017-1_bfm_nomouth_model_contours.json");
		ibug_contour = eos::fitting::ContourLandmarks::load("res/ibug_to_bfm2017-1_bfm_nomouth.txt");
		edge_topology = eos::morphablemodel::load_edge_topology("res/bfm2017-1_bfm_nomouth_edge_topology.json");
		break;
	}
}

eos::core::Mesh FacialMorpher::morph(dlib::full_object_detection face, rs2::video_frame color_frame) {
	// Create OpenCV's Mat object from RealSense color frame
	cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

	std::vector<Eigen::Vector4f> model_points; // 3d morphable model points
	std::vector<int> vertex_indices;
	std::vector<Eigen::Vector2f> image_points; // corresponding 2d coordinates

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

	eos::fitting::ScaledOrthoProjectionParameters pose = eos::fitting::estimate_orthographic_projection_linear(image_points, model_points, true, image.rows);
	eos::fitting::RenderingParameters rendering_params(pose, image.cols, image.rows);

	const float yaw_angle = glm::degrees(glm::yaw(rendering_params.get_rotation()));

	const Eigen::Matrix<float, 3, 4> affine_from_ortho = eos::fitting::get_3x4_affine_camera_matrix(rendering_params, image.cols, image.rows);

	const std::vector<float> fitted_coeffs = eos::fitting::fit_shape_to_landmarks_linear(morphable_model.get_shape_model(), affine_from_ortho, image_points, vertex_indices);

	const eos::core::Mesh mesh = morphable_model.draw_sample(fitted_coeffs, std::vector<float>());

	// Save the mesh
	eos::core::write_obj(mesh, "morph.obj");

	return mesh;
}