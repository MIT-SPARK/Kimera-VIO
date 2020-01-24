/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Equidistant4.cpp
 * @author Bernd Pfrommer
 */

#include "kimera-vio/distortion_models/Equidistant4.h"

#include <gtsam/geometry/PinholeCamera.h>
#include <vector>

namespace VIO {
Equidistant4::Equidistant4(const Intrinsics& intrinsics,
                           const std::vector<double>& distortion) {
  model_ = Cal3FS2(intrinsics[0],   // fx
                   intrinsics[1],   // fy
                   intrinsics[2],   // u0 (px)
                   intrinsics[3],   // v0 (py)
                   distortion[0],   // k1
                   distortion[1],   // k2
                   distortion[2],   // k3
                   distortion[3]);  // k4
}

gtsam::Point2 Equidistant4::uncalibrate(const gtsam::Point2& p) const {
  return (model_.uncalibrate(p));
}

bool Equidistant4::equals(const DistortionModelConstPtr& dm) const {
  Equidistant4ConstPtr eqm = std::dynamic_pointer_cast<const Equidistant4>(dm);
  if (!eqm) {
    return (false);
  }
  return model_.equals(eqm->model_);
}

void Equidistant4::print() const { model_.print(); }

bool Equidistant4::test(const double* intr, const double* dist) const {
  return (intr[0] == model_.fx() && intr[1] == model_.fy() &&
          intr[2] == model_.px() && intr[3] == model_.py() &&
          dist[0] == model_.k1() && dist[1] == model_.k2() &&
          dist[2] == model_.k3() && dist[3] == model_.k4());
}

gtsam::Point2 Equidistant4::project(const gtsam::Pose3& pose,
                                    const gtsam::Point3& p) const {
  gtsam::PinholeCamera<Cal3FS2> cam(pose, model_);
  return cam.project(p);
}

}  // namespace VIO
