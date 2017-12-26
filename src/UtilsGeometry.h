/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   UtilsGeometry.h
 * @brief  Utilities to compute geometric quantities
 * @author Luca Carlone
 */

#ifndef UtilsGeometry_H_
#define UtilsGeometry_H_

#include <stdlib.h>
#include <opengv/point_cloud/methods.hpp>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <sys/time.h>

namespace VIO {

class UtilsGeometry {

public:
  /* ----------------------------------------------------------------------------- */
  // Open files with name output_filename, and checks that it is valid
  static double getRatioBetweenTangentialAndRadialDisplacement(std::vector<gtsam::Point3> points){
    // compute radial directions
    gtsam::Vector3 u_mean = gtsam::Vector3::Zero();
    double minNorm = std::numeric_limits<double>::max();
    size_t minNormInd;
    for(size_t i=0; i<points.size();i++){
      double norm_i = points.at(i).vector().norm();
      u_mean += points.at(i).vector() / norm_i; // average versors
      if (norm_i < minNorm){
        minNormInd=i; // point closest to the camera
        minNorm = norm_i;
      }
    }
    u_mean = u_mean / u_mean.norm();

    // compute elongation
    double min_r = std::numeric_limits<double>::max(), max_r = 0.0;
    double max_t = 0.0;
    for(size_t i=0; i<points.size();i++){
      // compute min and max radial elongation (parallel to u_mean)
      double radialElongation_i = u_mean.dot(points.at(i).vector());
      min_r = std::min(min_r, radialElongation_i);
      max_r = std::max(max_r, radialElongation_i);

      // compute min and max tangential elongation (orthogonal to u_mean)
      double tangentialElongation_i = (minNorm * u_mean - points.at(i).vector()).norm(); // distance with respect to a central point at distance minNorm
      max_t = std::max(max_t, tangentialElongation_i);
    }
    if(min_r<0 || max_r <0 || min_r > max_r){
      throw std::runtime_error("getRatioBetweenTangentialAndRadialDisplacement: negative radial components");
    }
    return  max_t / (max_r-min_r);
  }
};
} // namespace VIO
#endif /* UtilsGeometry_H_ */
