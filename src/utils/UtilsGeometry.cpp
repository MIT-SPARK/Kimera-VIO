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
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/utils/UtilsGeometry.h"

#include <algorithm>
#include <limits>

#include <glog/logging.h>

namespace VIO {

/* -------------------------------------------------------------------------- */
// Open files with name output_filename, and checks that it is valid.
double UtilsGeometry::getRatioBetweenTangentialAndRadialDisplacement(
    const std::vector<gtsam::Point3>& points) {
  // Compute radial directions.
  double min_z = std::numeric_limits<double>::max();
  double max_z = 0;
  for (size_t i = 0; i < points.size(); i++) {
    double z_i = points.at(i).z();
    if (z_i < min_z) {
      min_z = z_i;
    }
    if (z_i > max_z) {
      max_z = z_i;
    }
  }

  std::vector<gtsam::Point3> points_rescaled;
  for (size_t i = 0; i < points.size(); i++) {
    // Point rescaled is a projection of point on a virtual img plane at
    // distance minZ.
    points_rescaled.push_back((points.at(i) / points.at(i).z()) * min_z);
  }

  double max_t = 0.0;
  double tangential_elongation_1 =
      (points_rescaled.at(0) - points_rescaled.at(1)).norm();
  max_t = std::max(max_t, tangential_elongation_1);
  double tangential_elongation_2 =
      (points_rescaled.at(1) - points_rescaled.at(2)).norm();
  max_t = std::max(max_t, tangential_elongation_2);
  double tangential_elongation_3 =
      (points_rescaled.at(0) - points_rescaled.at(2)).norm();
  max_t = std::max(max_t, tangential_elongation_3);

  // Points must be in front of the camera, and min should be less than max.
  if (min_z < 0 || max_z < 0 || min_z > max_z) {
    VLOG(100) << "min_r = " << min_z << " max_r = " << max_z << "\n";
    LOG(ERROR) << "Negative radial components! Points are behind camera.";
    return 0;
  }
  return max_t / (max_z - min_z);
}

}  // namespace VIO
