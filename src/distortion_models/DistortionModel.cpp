/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DistortionModel.cpp
 * @author Bernd Pfrommer
 */

#include "kimera-vio/distortion_models/DistortionModel.h"
#include "kimera-vio/distortion_models/Equidistant4.h"
#include "kimera-vio/distortion_models/RadTan4.h"
#include "kimera-vio/distortion_models/RadTan8.h"

#include <glog/logging.h>
#include <memory>
#include <string>
#include <vector>

namespace VIO {
// check if it is a valid model
bool DistortionModel::is_valid(const std::string& name, int num_dist_coeff) {
  if (name == "radtan" || name == "radial-tangential" || name == "plumb-bob" ||
      name == "plumb_bob") {
    if (num_dist_coeff == 4 || num_dist_coeff == 8) {
      return (true);
    } else {
      LOG(ERROR) << "distortion model " << name
                 << " invalid num coeff: " << num_dist_coeff << std::endl;
    }
  } else if (name == "equidistant" || name == "equdist") {
    if (num_dist_coeff == 4) {
      return (true);
    } else {
      LOG(ERROR) << "distortion model " << name
                 << " invalid num coeff: " << num_dist_coeff << std::endl;
    }
  } else {
    LOG(ERROR) << " invalid distortion model " << name << std::endl;
  }
  return (false);
}

// factory method for creating pinhole model
// with only px and py
DistortionModelPtr DistortionModel::make_pinhole(double fx,
                                                 double fy,
                                                 double cu,
                                                 double cv) {
  std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0};
  Intrinsics intrinsics = {fx, fy, cu, cv};
  return (std::shared_ptr<RadTan4>(new RadTan4(intrinsics, dist_coeffs)));
}

// factory method for creating distortion models
DistortionModelPtr DistortionModel::make(
    const std::string& name,       // radtan, equidistant etc...
    const Intrinsics& intrinsics,  // kx, ky, cu, cv
    const std::vector<double>& dist_coeffs) {
  if (name == "radtan" || name == "radial-tangential" || name == "plumb-bob" ||
      name == "plumb_bob") {
    switch (dist_coeffs.size()) {
      case 4:
        return (std::shared_ptr<RadTan4>(new RadTan4(intrinsics, dist_coeffs)));
      case 8:
        return (std::shared_ptr<RadTan8>(new RadTan8(intrinsics, dist_coeffs)));
      default:
        throw std::runtime_error("radtan: only 4 or 8 dist coeff supported");
    }
  } else if (name == "equidistant" || name == "equidist") {
    if (dist_coeffs.size() != 4) {
      throw std::runtime_error("equidistant: only 4 dist coeff supported");
      return (std::shared_ptr<Equidistant4>());
    } else {
      return (std::shared_ptr<Equidistant4>(
          new Equidistant4(intrinsics, dist_coeffs)));
    }
  }
  throw std::runtime_error("invalid distortion model");
}
}  // namespace VIO
