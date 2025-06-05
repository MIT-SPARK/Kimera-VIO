/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssFactor.cpp
 * @brief  Gnss Factor class
 * @author Elizaveta Karaseva
 */

#include "kimera-vio/factors/GnssFactor.h"

#include <glog/logging.h>

#include <string>

namespace gtsam {

GnssFactor::GnssFactor(const Key& poseKey,
                       const Point3& measuredPosition,
                       const SharedNoiseModel& noiseModel)
    : Base(noiseModel, poseKey), position_(measuredPosition) {}
GnssFactor::~GnssFactor() = default;

gtsam::Vector GnssFactor::evaluateError(
    const gtsam::Pose3& pose,
    boost::optional<gtsam::Matrix&> H) const {
  if (H) {
    *H = (Matrix(3, 6) << Matrix3::Zero(), Matrix3::Identity()).finished();
  }
  auto err = pose.translation() - position_;
  return pose.translation() - position_;
}

gtsam::NonlinearFactor::shared_ptr GnssFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new GnssFactor(*this));
}

void GnssFactor::print(const std::string& s,
  const KeyFormatter& keyFormatter) const {
  LOG(INFO) << s << "GNSS Factor on pose " << keyFormatter(this->key()) << "\n";
  this->noiseModel_->print("  noise model: ");
  traits<Point3>::Print(position_, "  measured position: ");
}
}  // namespace gtsam
