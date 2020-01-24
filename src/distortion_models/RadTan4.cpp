/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RadTan4.cpp
 * @author Bernd Pfrommer
 */

#include "kimera-vio/distortion_models/RadTan4.h"

#include <gtsam/geometry/PinholeCamera.h>
#include <vector>

namespace VIO {
RadTan4::RadTan4(const Intrinsics& intrinsics,
                 const std::vector<double>& distortion) {
  model_ = gtsam::Cal3DS2(intrinsics[0],   // fx
                          intrinsics[1],   // fy
                          0.0,             // skew
                          intrinsics[2],   // u0
                          intrinsics[3],   // v0
                          distortion[0],   // k1
                          distortion[1],   // k2
                          distortion[2],   // p1
                          distortion[3]);  // p2
}
gtsam::Point2 RadTan4::uncalibrate(const gtsam::Point2& p) const {
  return (model_.uncalibrate(p));
}

bool RadTan4::equals(const DistortionModelConstPtr& dm) const {
  RadTan4ConstPtr rtm = std::dynamic_pointer_cast<const RadTan4>(dm);
  if (!rtm) {
    return (false);
  }
  return model_.equals(rtm->model_);
}

void RadTan4::print() const { model_.print(); }

bool RadTan4::test(const double* intr, const double* dist) const {
  return (intr[0] == model_.fx() && intr[1] == model_.fy() &&
          intr[2] == model_.px() && intr[3] == model_.py() &&
          dist[0] == model_.k1() && dist[1] == model_.k2() &&
          dist[2] == model_.p1() && dist[3] == model_.p2());
}

gtsam::Point2 RadTan4::project(const gtsam::Pose3& pose,
                               const gtsam::Point3& p) const {
  gtsam::PinholeCamera<gtsam::Cal3DS2> cam(pose, model_);
  return cam.project(p);
}

}  // namespace VIO
