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
//#include <opencv2/viz/widget_accessor.hpp> // TODO: needed for sphere visualize, but cause compile issues

#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace VIO {

class Mesher {


public:

  cv::viz::WCloudCollection map_;
  cv::viz::Viz3d myWindow_;

  // constructors
  Mesher(): myWindow_("3D Mapper") {
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
  // TODO: make the following code compile
  //  // Visualize a 3D point cloud using sphere widget from opencv viz
  //  static void VisualizePoints3D_sphere(vector<gtsam::Point3> points){
  //
  //    const double radius = .15;
  //    const int resolution = 15;
  //    const cv::viz::Color color = viz::Color::green();
  //    cv::viz::Viz3d myWindow("Point Cloud 3D");
  //
  //    vector<cv::viz::WSphere> widgets;
  //    for(size_t i = 0 ; i < points.size(); i++){
  //      cv::Point3f point_i;
  //      point_i.x = float ( points.at(i).x() );
  //      point_i.y = float ( points.at(i).y() );
  //      point_i.z = float ( points.at(i).z() );
  //      widgets.push_back(viz::WSphere(points[i], radius, resolution, color));
  //    }
  //
  //    for(size_t i = 0; i < widgets.size(); i++)
  //      myWindow.showWidget("Point #" + std::to_string(i), widgets[i]);
  //
  //    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
  //
  //    myWindow.spin();
  //  }

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

    std::cout << "before creating widget" << std::endl;
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
  void visualizeMap3D(vector<gtsam::Point3> points){
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
    map_.addCloud(pointCloud, cv::viz::Color::green());
    map_.setRenderingProperty( cv::viz::POINT_SIZE, 2 );

    // plot points
    myWindow_.showWidget("point cloud map", map_);

    /// Start event loop.
    myWindow_.spinOnce(100);
  }

};
} // namespace VIO
#endif /* Mesher_H_ */


//
//    /// Let's assume camera has the following properties
//    Point3f cam_pos(3.0f,3.0f,3.0f), cam_focal_point(3.0f,3.0f,2.0f), cam_y_dir(-1.0f,0.0f,0.0f);
//
//    /// We can get the pose of the cam using makeCameraPose
//    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
//
//    /// We can get the transformation matrix from camera coordinate system to global using
//    /// - makeTransformToGlobal. We need the axes of the camera
//    Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,-1.0f,0.0f), Vec3f(-1.0f,0.0f,0.0f), Vec3f(0.0f,0.0f,-1.0f), cam_pos);
//
//    /// Create a cloud widget.
//    Mat bunny_cloud = cvcloud_load();
//    viz::WCloud cloud_widget(bunny_cloud, viz::Color::green());
//
//    /// Pose of the widget in camera frame
//    Affine3f cloud_pose = Affine3f().translate(Vec3f(0.0f,0.0f,3.0f));
//    /// Pose of the widget in global frame
//    Affine3f cloud_pose_global = transform * cloud_pose;
//
//    /// Visualize camera frame
//    if (!camera_pov)
//    {
//        viz::WCameraPosition cpw(0.5); // Coordinate axes
//        viz::WCameraPosition cpw_frustum(Vec2f(0.889484, 0.523599)); // Camera frustum
//        myWindow.showWidget("CPW", cpw, cam_pose);
//        myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
//    }
//
//    /// Visualize widget
//    myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);
//
//    /// Set the viewer pose to that of camera
//    if (camera_pov)
//        myWindow.setViewerPose(cam_pose);
//
//    /// Start event loop.
//    myWindow.spin();
//
//    return 0;


