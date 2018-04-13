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
    double minZ = std::numeric_limits<double>::max();
    double maxZ = 0;
    for(size_t i=0; i<points.size();i++){
      double z_i = points.at(i).z();

      if (z_i < minZ) {
        minZ = z_i;
      }
      if (z_i > maxZ) {
        maxZ = z_i;
      }
    }

    std::vector<gtsam::Point3> points_rescaled;
    for(size_t i=0; i<points.size();i++){
      // Point rescaled is a projection of point on a virtual img plane at
      // distance minZ.
      points_rescaled.push_back((points.at(i) / points.at(i).z()) * minZ);
    }

    double max_t = 0.0;
    double tangential_elongation_1 = (points_rescaled.at(0) - points_rescaled.at(1)).norm();
    max_t = std::max(max_t, tangential_elongation_1);
    double tangential_elongation_2 = (points_rescaled.at(1) - points_rescaled.at(2)).norm();
    max_t = std::max(max_t, tangential_elongation_2);
    double tangential_elongation_3 = (points_rescaled.at(0) - points_rescaled.at(2)).norm();
    max_t = std::max(max_t, tangential_elongation_3);

    // Points must be in front of the camera, and min should be less than max.
    if(minZ<0 || maxZ<0 || minZ>maxZ){
      std::cout << "min_r = " << minZ << " max_r = " << maxZ << std::endl;
      for(size_t i=0;i<points.size();i++){ points.at(i).print("\n");}
      throw std::runtime_error("getRatioBetweenTangentialAndRadialDisplacement: negative radial components");
    }
    return  max_t / (maxZ-minZ);
  }
};
} // namespace VIO
#endif /* UtilsGeometry_H_ */
