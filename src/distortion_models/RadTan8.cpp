/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RadTan8.cpp
 * @author Bernd Pfrommer
 */

#include "kimera-vio/distortion_models/RadTan8.h"
#include <gtsam/geometry/PinholeCamera.h>
#include <vector>

namespace VIO {
RadTan8::RadTan8(const Intrinsics& intrinsics,
                 const std::vector<double>& distortion) {
  // assume distortion parameters are specified as
  // k1, k2, p1, p2, k3, k4, k5, k6
  // (opencv convention)
  std::vector<double> k;
  k.push_back(distortion[0]);  // k1
  k.push_back(distortion[1]);  // k2
  // [2] and [3] are p1 and p2.
  // now add the remaining k3 - k6's to
  for (size_t i = 4; i < distortion.size(); i++) {
    k.push_back(distortion[i]);
  }
  while (k.size() < 6) {
    k.push_back(0);  // fill unspecified k's with 0
  }
  model_ = Cal3DS3(intrinsics[0],  // fx
                   intrinsics[1],  // fy
                   intrinsics[2],  // u0
                   intrinsics[3],  // v0
                   distortion[2],  // p1
                   distortion[3],  // p2
                   &k[0]);         // k1 .. k6
}

gtsam::Point2 RadTan8::uncalibrate(const gtsam::Point2& p) const {
  return (model_.uncalibrate(p));
}

bool RadTan8::equals(const DistortionModelConstPtr& dm) const {
  RadTan8ConstPtr rtm = std::dynamic_pointer_cast<const RadTan8>(dm);
  if (!rtm) {
    return (false);
  }
  return model_.equals(rtm->model_);
}

void RadTan8::print() const { model_.print(); }

bool RadTan8::test(const double* intr, const double* dist) const {
  return (intr[0] == model_.fx() && intr[1] == model_.fy() &&
          intr[2] == model_.px() && intr[3] == model_.py() &&
          dist[0] == model_.k1() && dist[1] == model_.k2() &&
          dist[2] == model_.p1() && dist[3] == model_.p2());
}

gtsam::Point2 RadTan8::project(const gtsam::Pose3& pose,
                               const gtsam::Point3& p) const {
  gtsam::PinholeCamera<Cal3DS3> cam(pose, model_);
  return cam.project(p);
}

}  // namespace VIO
