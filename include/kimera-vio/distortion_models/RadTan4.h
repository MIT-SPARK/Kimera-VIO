/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RadTan4.h
 * @brief  implements original gtsam 4-coefficient radtan model
 * @author Bernd Pfrommer
 */

#ifndef INCLUDE_KIMERA_VIO_DISTORTION_MODELS_RADTAN4_H_
#define INCLUDE_KIMERA_VIO_DISTORTION_MODELS_RADTAN4_H_

#include <gtsam/geometry/Cal3DS2.h>
#include "kimera-vio/distortion_models/DistortionModel.h"

#include <memory>
#include <vector>

namespace VIO {
class RadTan4 : public DistortionModel {
 public:
  RadTan4(const Intrinsics& intrinsics, const std::vector<double>& dist_coeffs);
  // method inherited from DistortionModel base class
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const override;
  bool equals(const DistortionModelConstPtr& dm) const override;
  void print() const override;
  bool test(const double* intr, const double* dist) const override;
  gtsam::Point2 project(const gtsam::Pose3& pose,
                        const gtsam::Point3& p) const override;
  double fx() const override { return (model_.fx()); }
  double fy() const override { return (model_.fy()); }
  double px() const override { return (model_.px()); }
  double py() const override { return (model_.py()); }
  double skew() const override { return (model_.skew()); }

 private:
  gtsam::Cal3DS2 model_;
};

// pointers
typedef std::shared_ptr<RadTan4> RadTan4Ptr;
typedef std::shared_ptr<const RadTan4> RadTan4ConstPtr;
}  // namespace VIO
#endif  // INCLUDE_KIMERA_VIO_DISTORTION_MODELS_RADTAN4_H_
