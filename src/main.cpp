#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <librealsense2/rs.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>

#include <iostream>
#include <chrono>
#include <string>

#include "helpers.h"
#include "OgreApp.h"
#include "FacialDetector.h"
#include "FacialMorpher.h"

// distances for RS2 camera colorizer
#define MIN_SCAN_DISTANCE 0.3f
#define MAX_SCAN_DISTANCE 0.5f

using namespace pcl;
using namespace rs2;

// function headers
void capture_frame(rs2::pipeline pipe);
void generate_mesh(PointCloud<PointXYZ>::Ptr cloud);

// time measurement
std::chrono::steady_clock::time_point begin;
std::chrono::steady_clock::time_point end;

// to define the target stream for RealSense
enum class direction
{
	to_depth,
	to_color
};

// Ogre
OgreApp ogre_app;

int main(int argc, char** argv) try
{
	// Instantiate modules
	FacialDetector detector;
	FacialMorpher morpher(SFM);

	// Init Ogre app
	//ogre_app.initApp();

	// Begin time measurement!
	begin = std::chrono::steady_clock::now();

	// RS2: Create and initialize GUI related objects
	//window app(1280, 720, "Glyptics Portrait Generator"); // Simple window handling
	//ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
	rs2::colorizer colorizer;             // Helper to colorize depth images
	texture depth_image, color_image;     // Helpers for rendering images
	threshold_filter threshold_filter;	  // Filter for thresholds

	// RS2: Create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8);
	pipe.start(cfg);

	// RS2: Define two align objects to align to depth/color viewport
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	rs2::align align_to_color(RS2_STREAM_COLOR);

	float       alpha = 0.6f;               // Transparency coefficient 
	direction   dir = direction::to_color;  // Alignment direction

	// RS2: Set options for colorizer and  filters
	colorizer.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 1.0f);
	colorizer.set_option(RS2_OPTION_MAX_DISTANCE, MAX_SCAN_DISTANCE);
	colorizer.set_option(RS2_OPTION_MIN_DISTANCE, MIN_SCAN_DISTANCE);
	threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, MAX_SCAN_DISTANCE);
	threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, MIN_SCAN_DISTANCE);

	int i = 0;
	eos::core::Mesh mesh;
	while (i < 5) 
	{
		// RS2: Block the application until a frameset is available
		frameset frameset = pipe.wait_for_frames();

		if (dir == direction::to_depth)
		{
			frameset = align_to_depth.process(frameset);
		}
		else
		{
			frameset = align_to_color.process(frameset);
		}

		// With the aligned frameset we proceed as usual
		auto depth_frame = frameset.get_depth_frame();
		auto color_frame = frameset.get_color_frame();

		auto filtered = depth_frame;
		filtered = threshold_filter.process(filtered);

		auto colorized_depth = colorizer.colorize(filtered);

		auto face = detector.detect(color_frame, depth_frame);
		mesh = morpher.morph(face, color_frame);

		i++;
		//return EXIT_SUCCESS;

		/*
		glEnable(GL_BLEND);
		// Use the Alpha channel for blending
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (dir == direction::to_depth)
		{
			// When aligning to depth, first render depth image
			// and then overlay color on top with transparancy
			depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });
			color_image.render(color_frame, { 0, 0, app.width(), app.height() }, alpha);
		}
		else
		{
			// When aligning to color, first render color image
			// and then overlay depth image on top
			color_image.render(color_frame, { 0, 0, app.width(), app.height() });
			depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() }, 1 - alpha);
		}

		glColor4f(1.f, 1.f, 1.f, 1.f);
		glDisable(GL_BLEND);

		// Render the UI:
		ImGui_ImplGlfw_NewFrame(1);
		static const int flags = ImGuiWindowFlags_NoCollapse
			| ImGuiWindowFlags_NoScrollbar
			| ImGuiWindowFlags_NoSavedSettings
			| ImGuiWindowFlags_NoTitleBar
			| ImGuiWindowFlags_NoResize
			| ImGuiWindowFlags_NoMove;
		ImGui::Begin("capture", nullptr, flags);
		if (ImGui::Button("Capture"))
		{
			capture_frame(pipe);
		}
		ImGui::End();
		ImGui::Render();
		*/
	}
	//ogre_app.add_or_update_mesh(mesh);
	//ogre_app.getRoot()->startRendering();
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

void capture_frame(rs2::pipeline pipe)
{
	pointcloud pc;
	points pts;

	auto frames = pipe.wait_for_frames();
	auto df = frames.get_depth_frame();

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	pts = pc.calculate(df);

	end = std::chrono::steady_clock::now();
	std::cout << "Adding points: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms elapsed" << std::endl;
	begin = end;
	auto vertices = pts.get_vertices();

	std::vector<rs2::vertex> newPoints;
	float totalX = 0;
	float totalY = 0;

	for (int i = 0; i < pts.size(); i++)
	{
		auto pt = vertices[i];
		if (pt.x != 0.0f || pt.y != 0.0f || pt.z != 0.0f)
		{
			if (pt.z > MIN_SCAN_DISTANCE && pt.z <= MAX_SCAN_DISTANCE) 
			{
				newPoints.push_back(pt);
				totalX += pt.x;
				totalY += pt.y;
			}
		}
	}

	// average position of all points
	float avgX = totalX / newPoints.size();
	float avgY = totalY / newPoints.size();
	
	for (auto pt : newPoints) {
		float newX = pt.x - avgX, newY = pt.y - avgY; //centering points
		if (newX * newX + newY * newY <= 0.025) //circular shape
		{
			cloud->points.push_back(PointXYZ(newX, -newY, pt.z));
		}
	}

	end = std::chrono::steady_clock::now();
	std::cout << "Added " << cloud->size() << " points: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms elapsed" << std::endl;
	begin = end;

	PointCloud<PointXYZ>::Ptr downsampledCloud(new PointCloud<PointXYZ>);
	VoxelGrid<PointXYZ> grid;
	grid.setInputCloud(cloud);
#if NDEBUG
	std::cout << "Setting leaf size to 0.001" << std::endl;
	grid.setLeafSize(0.003f, 0.003f, 0.003f);
#else
	std::cout << "Setting leaf size to 0.03" << std::endl;
	grid.setLeafSize(0.03f, 0.03f, 0.03f);
#endif
	grid.filter(*downsampledCloud);
	end = std::chrono::steady_clock::now();
	std::cout << "Downsampled point cloud by a factor of " << (float)(pts.size()) / downsampledCloud->size() << ": " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms elapsed" << std::endl;
	begin = end;

	generate_mesh(downsampledCloud);
}

void generate_mesh(PointCloud<PointXYZ>::Ptr cloud) 
{
	std::cout << "Starting mesh generation!" << std::endl;

	// Normal estimation*
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	NormalEstimation<PointXYZ, Normal> ne;
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(20);
	ne.setViewPoint(0, 0, -5);
	ne.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	Normal axis = Normal(0, 0, -1);

	//make all normals face the direction towards camera
	for (int i = 0; i < normals->points.size(); i++) {
		if (axis.getNormalVector4fMap().dot(normals->points[i].getNormalVector4fMap()) < 0) {
			normals->points[i].normal_x *= -1;
			normals->points[i].normal_y *= -1;
			normals->points[i].normal_z *= -1;
		}
	}

	end = std::chrono::steady_clock::now();
	std::cout << "Normals computed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms elapsed" << std::endl;
	begin = end;

	// Concatenate the XYZ and normal fields*
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	end = std::chrono::steady_clock::now();
	std::cout << "Tree set: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms elapsed" << std::endl;
	begin = end;

	// Initialize objects
	GreedyProjectionTriangulation<PointNormal> gp3;
	PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025); // 0.025

	// Set typical values for the parameters
	gp3.setMu(2.5); //2.5
	gp3.setMaximumNearestNeighbors(100); //100
	gp3.setMaximumSurfaceAngle(M_PI / 4); // pi/4 = 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // pi/18 = 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 2pi/3 = 120 degrees
	gp3.setNormalConsistency(true);
	gp3.setConsistentVertexOrdering(true);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	
	end = std::chrono::steady_clock::now();
	std::cout << "Reconstructed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms elapsed" << std::endl;
	begin = end;

	// Finish
	io::saveOBJFile("mesh.obj", triangles);

	end = std::chrono::steady_clock::now();
	std::cout << "Saved: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms elapsed" << std::endl;

	ogre_app.initApp();
	ogre_app.add_mesh(triangles, cloud);
}