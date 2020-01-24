/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RadTan8.h
 * @brief  implements radtan model with p1, p2, k1...k6
 * @author Bernd Pfrommer
 */

#ifndef INCLUDE_KIMERA_VIO_DISTORTION_MODELS_RADTAN8_H_
#define INCLUDE_KIMERA_VIO_DISTORTION_MODELS_RADTAN8_H_

#include <memory>
#include <vector>
#include "kimera-vio/distortion_models/DistortionModel.h"
#include "kimera-vio/gtsam_distortion/Cal3DS3.h"

namespace VIO {
class RadTan8 : public DistortionModel {
 public:
  RadTan8(const Intrinsics& intrinsics, const std::vector<double>& dist_coeffs);
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
  double skew() const override { return (0.0); }

 private:
  Cal3DS3 model_;
};

// pointers
typedef std::shared_ptr<RadTan8> RadTan8Ptr;
typedef std::shared_ptr<const RadTan8> RadTan8ConstPtr;
}  // namespace VIO
#endif  //  INCLUDE_KIMERA_VIO_DISTORTION_MODELS_RADTAN8_H_
