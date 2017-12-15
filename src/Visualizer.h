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

namespace VIO {

class Visualizer {


public:
  cv::viz::WCloudCollection mapWithRepeatedPoints_;
  cv::viz::Viz3d myWindow_;

  // constructors
  Visualizer(): myWindow_("3D Mapper") {
    // create window and create axes:
    myWindow_.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  }

  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static void VisualizeMesh2D(const Frame& frame, const std::vector<cv::Vec6f> triangulation2D, const double waitTime = 0){
    cv::Scalar delaunay_color(0,255,0), points_color(255, 0,0);

    // sanity check
    if(frame.landmarks_.size() != frame.keypoints_.size())
      throw std::runtime_error("mesher: wrong dimension for the landmarks");


    cv::Mat img = frame.img_.clone();
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    cv::Size size = img.size();
    cv::Rect rect(0,0, size.width, size.height);

    std::vector<cv::Point> pt(3);

    for(size_t i = 0; i < triangulation2D.size(); i++)
    {
      cv::Vec6f t = triangulation2D[i];
      pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
      pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
      pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

      cv::line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
      cv::line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
      cv::line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
    }

    for(size_t i=0; i < frame.keypoints_.size(); i++){
      if(frame.landmarks_[i] != -1) // only for valid keypoints
        cv::circle(img, frame.keypoints_[i], 2, points_color, CV_FILLED, CV_AA, 0);
    }
    cv::imshow("Mesh Results", img);
    cv::waitKey(waitTime);
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
    if(timeHold == 0)
      myWindow.spin();
    else
      myWindow.spinOnce(timeHold);
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
    myWindow_.spinOnce(100);
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

    // plot points
    myWindow_.showWidget("point cloud map",  cloud_widget);

    /// Start event loop.
    myWindow_.spinOnce(50);
  }

  /* ----------------------------------------------------------------------------- */
  // Visualize a 3D point cloud of unique 3D landmarks with its connectivity
  void visualizeMesh3D(const std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId, const Frame& frame, const Mesher& mesher){

    // sanity check dimension
    if(pointsWithId.size() == 0) // no points to visualize
      return;

    // Create a cloud widget.
    std::cout << "WMesh " <<  std::endl;
    cv::viz::WMesh mesh(mesher.mapPoints3d_.t(), mesher.createMesh3d_MapPointId(frame));

    // plot points
    std::cout << "WMesh  showWidget" <<  std::endl;
    myWindow_.showWidget("point cloud map",  mesh);

    /// Start event loop.
    myWindow_.spinOnce(100);
  }
};
} // namespace VIO
#endif /* Visualizer_H_ */


