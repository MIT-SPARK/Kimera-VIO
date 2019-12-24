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
 * @author Luca Carlone, Antoni Rosinol
 */

#ifndef UtilsGeometry_H_
#define UtilsGeometry_H_

#include <gtsam/geometry/Point3.h>
#include <vector>

namespace VIO {

class UtilsGeometry {
 public:
  /* ------------------------------------------------------------------------ */
  // Open files with name output_filename, and checks that it is valid.
  static double getRatioBetweenTangentialAndRadialDisplacement(
      const std::vector<gtsam::Point3>& points);
};

}  // namespace VIO

#endif /* UtilsGeometry_H_ */
