/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher.h
 * @brief  Build and visualize 2D mesh from Frame
 * @author Luca Carlone, AJ Haeffner
 */

#ifndef Mesher_H_
#define Mesher_H_

#include "Frame.h"
#include <stdlib.h>
#include <opengv/point_cloud/methods.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace VIO {

class Mesher {


public:
  using LandmarkIdToMapPointId = std::unordered_map<LandmarkId, int>;

  cv::viz::WCloudCollection mapWithRepeatedPoints_;
  cv::viz::Viz3d myWindow_;

  cv::Mat mapPoints3d_; // set of (non-repeated) points = valid landmark positions
  LandmarkIdToMapPointId lmkIdToMapPointId_; // maps lmk id to corresponding 3D points
  int points3D_count_; // number of points

  // constructors
  Mesher(): myWindow_("3D Mapper"), points3D_count_(0) {
    // create window and create axes:
    myWindow_.showWidget("Coordinate Widget", viz::WCoordinateSystem());
  }

  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static std::vector<cv::Vec6f> CreateMesh2D(const Frame& frame){

    frame.print();

    // Rectangle to be used with Subdiv2D
    cv::Size size = frame.img_.size();
    cv::Rect2f rect(0, 0, size.width, size.height);

    // subdiv has the delaunay triangulation function
    cv::Subdiv2D subdiv(rect);

    // add points from Frame
    for(size_t i=0; i < frame.keypoints_.size(); i++){
      if(frame.landmarks_[i] != -1 && rect.contains(frame.keypoints_[i])){ // only for valid keypoints
        subdiv.insert(frame.keypoints_[i]);
      }
    }

    // do triangulation
    std::vector<cv::Vec6f> triangulation2D, triangulation2DwithExtraTriangles;

    // getTriangleList returns some spurious triangle with vertices outside image
    subdiv.getTriangleList(triangulation2DwithExtraTriangles);
    std::vector<cv::Point> pt(3);
    for(size_t i = 0; i < triangulation2DwithExtraTriangles.size(); i++)
        {
          cv::Vec6f t = triangulation2DwithExtraTriangles[i];
          pt[0] = cv::Point(t[0], t[1]);
          pt[1] = cv::Point(t[2], t[3]);
          pt[2] = cv::Point(t[4], t[5]);

          if(rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
            triangulation2D.push_back(t);
        }

    return triangulation2D;
  }
  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static void VisualizeMesh2D(const Frame& frame, const std::vector<cv::Vec6f> triangulation2D, const double waitTime = 0){
    cv::Scalar delaunay_color(0,255,0), points_color(255, 0,0);

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
  static void VisualizePoints3D(vector<gtsam::Point3> points, int timeHold = 0){
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
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

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
  void visualizeMap3D_repeatedPoints(vector<gtsam::Point3> points){
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
  // Visualize a 3D mesh of unique 3D landmarks with its connectivity
  void visualizeMesh3D(vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId){

    // sanity check dimension
    if(pointsWithId.size() == 0) // no points to visualize
      return;

    // update 3D points by adding new points or updating old reobserved ones
    for(size_t i=0; i<pointsWithId.size();i++) {

      LandmarkId lmk_id = pointsWithId.at(i).first;
      gtsam::Point3 point_i = pointsWithId.at(i).second;
      auto lm_it = lmkIdToMapPointId_.find(lmk_id);
      if (lm_it == lmkIdToMapPointId_.end()) // new landmark
      {
        cv::Point3f p(float(point_i.x()), float(point_i.y()), float(point_i.z()));
        mapPoints3d_.push_back(p);
        lmkIdToMapPointId_.insert(std::make_pair(lmk_id, mapPoints3d_.rows-1));
        ++points3D_count_;
      }
      else // replace point for existing landmark
      {
        cv::Point3f* data = mapPoints3d_.ptr<cv::Point3f>();
        int row_id = (*lm_it).second;
        data[row_id].x = float ( point_i.x() );
        data[row_id].y = float ( point_i.y() );
        data[row_id].z = float ( point_i.z() );
      }
    }
    // Create a cloud widget.
    cv::viz::WCloud cloud_widget(mapPoints3d_, cv::viz::Color::green());
    cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    // plot points
    myWindow_.showWidget("point cloud map",  cloud_widget);

    /// Start event loop.
    myWindow_.spinOnce(100);
  }

};
} // namespace VIO
#endif /* Mesher_H_ */


