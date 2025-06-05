/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssFactor.h
 * @brief  Gnss Factor class
 * @author Elizaveta Karaseva
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>  // for string

namespace gtsam {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
  using OptionalMatrixType = boost::optional<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&>;
#else
  using OptionalMatrixType = gtsam::OptionalMatrixType;
#endif

/**
 * @brief Factor between a Pose3 (variable) and a GNSS-measured Point3 (absolute position).
 * Only position is used from the Pose3; orientation is ignored.
 */
class GnssFactor : public NoiseModelFactor1<Pose3> {
 public:
  using Base = NoiseModelFactor1<Pose3>;
  /// Default constructor (for serialization only)
  GnssFactor();

  virtual ~GnssFactor();

  /// Constructor
  GnssFactor(const Key& poseKey,
             const Point3& measuredPosition,
             const SharedNoiseModel& noiseModel);

  // /// Evaluate residual: pose.translation() - GNSS
  gtsam::Vector evaluateError(
    const gtsam::Pose3& pose,
    boost::optional<gtsam::Matrix&> H = boost::none) const override;

  /// Print
  void print(const std::string& s = "GnssFactor",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;
  Point3 getPosition() const { return position_; }

  virtual gtsam::NonlinearFactor::shared_ptr clone() const;

 protected:
  Point3 position_;
};

}  // namespace gtsam
