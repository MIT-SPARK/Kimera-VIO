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
    return 0;
  }
};
} // namespace VIO
#endif /* UtilsGeometry_H_ */
