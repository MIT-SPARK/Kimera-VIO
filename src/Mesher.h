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
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace VIO {

class Mesher {

public:
  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static std::vector<cv::Vec6f> CreateMesh2D(const Frame& frame){
    // Rectangle to be used with Subdiv2D
    cv::Size size = frame.img_.size();
    cv::Rect rect(0, 0, size.width, size.height);

    // subdiv has the delaunay triangulation function
    cv::Subdiv2D subdiv(rect);

    // add points from Frame
    for(size_t i=0; i < frame.keypoints_.size(); i++)
      subdiv.insert(frame.keypoints_[i]);

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
  static void VisualizeMesh2D(const Frame& frame, const std::vector<cv::Vec6f> triangulation2D){
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

    for(size_t i=0; i < frame.keypoints_.size(); i++)
      cv::circle(img, frame.keypoints_[i], 2, points_color, CV_FILLED, CV_AA, 0);

    cv::imshow("Mesh Results", img);
    cv::waitKey(1000);
  }
};
} // namespace VIO
#endif /* Mesher_H_ */


