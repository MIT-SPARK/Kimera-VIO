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
    std::vector<cv::Vec6f> triangulation2D;
    subdiv.getTriangleList(triangulation2D);
    return triangulation2D;
  }
  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static void VisualizeMesh2D(const Frame& frame, const std::vector<cv::Vec6f> triangulation2D){
    cv::Scalar delaunay_color(255,255,255), points_color(0, 0, 255);

  }
};
} // namespace VIO
#endif /* Mesher_H_ */


