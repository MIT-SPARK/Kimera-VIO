/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer.h
 * @brief  Build and visualize 2D mesh from Frame
 * @author Luca Carlone, AJ Haeffner
 */

#ifndef Visualizer_H_
#define Visualizer_H_

#include "Mesher.h"
#include "glog/logging.h"
#ifdef USE_CGAL
#include "Mesher_cgal.h"
#endif

namespace VIO {

enum class VisualizationType {
  POINTCLOUD, // visualize 3D VIO points  (no repeated point)
  POINTCLOUD_REPEATEDPOINTS, // visualize VIO points as point clouds (points are re-plotted at every frame)
  MESH2D, // only visualizes 2D mesh on image
  MESH2DTo3D, // get a 3D mesh from a 2D triangulation of the (right-VALID) keypoints in the left frame
  MESH2DTo3Ddense, // dense triangulation of stereo corners (only a subset are VIO keypoints)
  MESH2Dsparse, // visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
  MESH2DTo3Dsparse, // same as MESH2DTo3D but filters out triangles corresponding to non planar obstacles
  MESH3D, // 3D mesh from CGAL using VIO points (requires #define USE_CGAL!)
  NONE // does not visualize map
};

class Visualizer3D {
public:

  cv::viz::WCloudCollection mapWithRepeatedPoints_;
  cv::viz::Viz3d myWindow_;
  std::vector<cv::Affine3f> trajectoryPoses3d_;
  cv::viz::Color cloudColor = cv::viz::Color::white();
  cv::viz::Color backgroundColor = cv::viz::Color::black();

  // constructors
  Visualizer3D(): myWindow_("3D Mapper") {
    // create window and create axes:
	myWindow_.setBackgroundColor(backgroundColor);
    myWindow_.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  }
  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud using cloud widget from opencv viz
  static void VisualizePoints3D(std::vector<gtsam::Point3> points, const int waitTime = 0){
    // based on longer example: https://docs.opencv.org/2.4/doc/tutorials/viz/transformations/transformations.html#transformations

    if(points.size() == 0) // no points to visualize
      return;

    // populate cloud structure with 3D points
    cv::Mat pointCloud(1,points.size(),CV_32FC3);
    cv::Point3f* data = pointCloud.ptr<cv::Point3f>();
    for(size_t i=0; i<points.size();i++){
      data[i].x = float ( points.at(i).x() );
      data[i].y = float ( points.at(i).y() );
      data[i].z = float ( points.at(i).z() );
    }
    // pointCloud *= 5.0f; // my guess: rescaling the cloud

    // Create a cloud widget.
    cv::viz::WCloud cloud_widget(pointCloud, cv::viz::Color::green()); // TODO: consider this entire method with its own window. Is it out of place?
    cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    // create window and create axes:
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

    // plot points
    myWindow.showWidget("point cloud map",  cloud_widget);

    /// Start event loop.
    myWindow.spinOnce(waitTime);
  }

  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud using cloud widget from opencv viz
  void visualizeMap3D_repeatedPoints(const std::vector<gtsam::Point3> points, const int waitTime = 0){
    // based on longer example: https://docs.opencv.org/2.4/doc/tutorials/viz/transformations/transformations.html#transformations

    if(points.size() == 0) // no points to visualize
      return;

    // populate cloud structure with 3D points
    cv::Mat pointCloud(1,points.size(),CV_32FC3);
    cv::Point3f* data = pointCloud.ptr<cv::Point3f>();
    for(size_t i=0; i<points.size();i++){
      data[i].x = float ( points.at(i).x() );
      data[i].y = float ( points.at(i).y() );
      data[i].z = float ( points.at(i).z() );
    }

    // add to the existing map
    mapWithRepeatedPoints_.addCloud(pointCloud, cloudColor);
    mapWithRepeatedPoints_.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    // plot points
    myWindow_.showWidget("point cloud map", mapWithRepeatedPoints_);

    /// Start event loop.
    myWindow_.spinOnce(waitTime);
  }
  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity
  void visualizePoints3D(const std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId, const Mesher& mesher, const int waitTime = 0){

    // sanity check dimension
    if(pointsWithId.size() == 0) // no points to visualize
      return;

    // Create a cloud widget.
    cv::viz::WCloud cloud_widget(mesher.mapPoints3d_, cloudColor);
    cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    myWindow_.showWidget("point cloud map",  cloud_widget); // plot points
    /// Start event loop.
    myWindow_.spinOnce(waitTime);
  }
  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity
  void visualizeMesh3D(const cv::Mat mapPoints3d, const cv::Mat polygonsMesh){

    // sanity check dimension
    if(mapPoints3d.rows == 0 || polygonsMesh.rows == 0) // no points/mesh to visualize
      return;

    // Create a cloud widget.
    cv::viz::WMesh mesh(mapPoints3d.t(), polygonsMesh);
    myWindow_.showWidget("point cloud map", mesh); // plot mesh
    /// Start event loop.
    myWindow_.spinOnce(100);
  }

  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity,
  // and provide color for each polygon.
  void visualizeMesh3D(const cv::Mat mapPoints3d, const cv::Mat polygonsMesh,
                       const cv::Mat& colors) {

    // sanity check dimension
    if(mapPoints3d.rows == 0 || polygonsMesh.rows == 0) // no points/mesh to visualize
      return;

    // Create a cloud widget.
    cv::viz::WMesh mesh(mapPoints3d.t(), polygonsMesh, colors.t());
    myWindow_.showWidget("point cloud map", mesh); // plot mesh
    /// Start event loop.
    myWindow_.spinOnce(100);
  }

  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity
  void visualizeMesh3D(const Mesher& mesher){
    // Color the mesh.
    cv::Mat colors (mesher.mapPoints3d_.rows, 1, CV_8UC3, cv::viz::Color::red());
    // The code below assumes triangles as polygons.
    if (mesher.triangle_clusters_.size() > 0) {
      // TODO now it only prints the first cluster.
      if (mesher.triangle_clusters_.at(0).triangle_ids_.size() > 0) {
        for (const size_t& triangle_id: mesher.triangle_clusters_.at(0).triangle_ids_) {
          size_t triangle_idx = std::round(triangle_id * 4);
          if (triangle_idx + 3 >= mesher.polygonsMesh_.rows) {
            throw std::runtime_error("Visualizer3D: an id in triangle_ids_ is too large.");
          }
          colors.row(mesher.polygonsMesh_.at<int32_t>(triangle_idx + 1)) = cv::viz::Color::green();
          colors.row(mesher.polygonsMesh_.at<int32_t>(triangle_idx + 2)) = cv::viz::Color::green();
          colors.row(mesher.polygonsMesh_.at<int32_t>(triangle_idx + 3)) = cv::viz::Color::green();
        }
      } else {
        LOG(ERROR) << "No elements in triangle cluster.";
      }
    } else {
      LOG(ERROR) << "Triangle clusters is empty.";
    }

    visualizeMesh3D(mesher.mapPoints3d_, mesher.polygonsMesh_, colors);
  }

  /* ----------------------------------------------------------------------------- */
  // add pose to the previous trajectory
  void addPoseToTrajectory(gtsam::Pose3 current_pose_gtsam){
    trajectoryPoses3d_.push_back( UtilsOpenCV::Pose2Affine3f(current_pose_gtsam) );
  }

  /* ----------------------------------------------------------------------------- */
  // Visualize trajectory
  void visualizeTrajectory3D(){
    if(trajectoryPoses3d_.size() == 0) // no points to visualize
      return;
    // Show current camera pose.
    static const cv::Matx33d K (458,0.0,360,0.0,458,240,0.0,0.0,1.0);
    cv::viz::WCameraPosition cam_widget (K, 1.0, cv::viz::Color::white());
    myWindow_.showWidget("Camera Pose with Frustum",  cam_widget, trajectoryPoses3d_.back());
    // Create a Trajectory widget. (argument can be PATH, FRAMES, BOTH).
    cv::viz::WTrajectory trajectory_widget (trajectoryPoses3d_, cv::viz::WTrajectory::PATH, 1.0, cv::viz::Color::red());
    myWindow_.showWidget("Trajectory",  trajectory_widget);
    /// Start event loop.
    //TODO: myWindow_.spinOnce(50);
  }

};
} // namespace VIO
#endif /* Visualizer_H_ */


