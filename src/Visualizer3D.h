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
#include "Mesher_cgal.h"

namespace VIO {
enum VisualizationType {
  POINTCLOUD, POINTCLOUD_REPEATEDPOINTS,MESH2D,MESH2DTo3D, MESH3D
};

class Visualizer3D {
public:

  cv::viz::WCloudCollection mapWithRepeatedPoints_;
  cv::viz::Viz3d myWindow_;
  std::vector<cv::Affine3f> trajectoryPoses3d_;

  // constructors
  Visualizer3D(): myWindow_("3D Mapper") {
    // create window and create axes:
    myWindow_.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  }
  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud using cloud widget from opencv viz
  static void VisualizePoints3D(std::vector<gtsam::Point3> points, int timeHold = 0){
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
    cv::viz::WCloud cloud_widget(pointCloud, cv::viz::Color::green());
    cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    // create window and create axes:
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

    // plot points
    myWindow.showWidget("point cloud map",  cloud_widget);

    /// Start event loop.
    // TODO
//    if(timeHold == 0)
//      myWindow.spin();
//    else
//      myWindow.spinOnce(timeHold);
  }

  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud using cloud widget from opencv viz
  void visualizeMap3D_repeatedPoints(const std::vector<gtsam::Point3> points){
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
    mapWithRepeatedPoints_.addCloud(pointCloud, cv::viz::Color::green());
    mapWithRepeatedPoints_.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    // plot points
    myWindow_.showWidget("point cloud map", mapWithRepeatedPoints_);

    /// Start event loop.
    //TODO: myWindow_.spinOnce(100);
  }
  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity
  void visualizePoints3D(const std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId, const Mesher& mesher){

    // sanity check dimension
    if(pointsWithId.size() == 0) // no points to visualize
      return;

    // Create a cloud widget.
    cv::viz::WCloud cloud_widget(mesher.mapPoints3d_, cv::viz::Color::green());
    cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    myWindow_.showWidget("point cloud map",  cloud_widget); // plot points
    /// Start event loop.
    //TODO: myWindow_.spinOnce(50);
  }
  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity
  void visualizeMesh3D(const cv::Mat mapPoints3d, const cv::Mat polygonsMesh){

    // sanity check dimension
    if(mapPoints3d.rows == 0 || polygonsMesh.rows == 0) // no points/mesh to visualize
      return;

    // Create a cloud widget.
    cv::viz::WMesh mesh(mapPoints3d.t(), polygonsMesh);
    myWindow_.showWidget("point cloud map",  mesh); // plot mesh
    /// Start event loop.
    myWindow_.spinOnce(100);
  }
  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity
  void visualizeMesh3D(const Mesher& mesher){
    visualizeMesh3D(mesher.mapPoints3d_,mesher.polygonsMesh_);
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
    // Create a Trajectory widget. (argument can be PATH, FRAMES, BOTH)
    cv::viz::WTrajectory trajectory_widget(trajectoryPoses3d_, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::blue());
    myWindow_.showWidget("Trajectory",  trajectory_widget);
    /// Start event loop.
    //TODO: myWindow_.spinOnce(50);
  }

};
} // namespace VIO
#endif /* Visualizer_H_ */


