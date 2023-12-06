/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * @file PointPlaneFactor.h
 * @brief PointPlane Factor class
 * @author Antoni Rosinol
 * @date February 20, 2018
 */

#pragma once

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using GtsamJacobianType = boost::optional<gtsam::Matrix&>;
#define JACOBIAN_DEFAULT \
  {}
#else
using GtsamJacobianType = gtsam::OptionalMatrixType;
#define JACOBIAN_DEFAULT nullptr
#endif

/**
 * Factor to implement error between a point landmark and a plane.
 */
class PointPlaneFactor : public NoiseModelFactor2<Point3, OrientedPlane3> {
 public:
  using Base = NoiseModelFactor2<Point3, OrientedPlane3>;

  /// Constructor
  PointPlaneFactor();

  virtual ~PointPlaneFactor();

  /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose
  /// symbol
  PointPlaneFactor(const Key& pointKey,
                   const Key& planeKey,
                   const SharedNoiseModel& noiseModel);

  /// print
  virtual void print(
      const std::string& s = "RegularPlane3Factor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// evaluateError
  /// Hpoint: jacobian of h wrt point landmark
  /// Hplane: jacobian of h wrt plane
  virtual Vector evaluateError(
      const Point3& point,
      const OrientedPlane3& plane,
      GtsamJacobianType H_point = JACOBIAN_DEFAULT,
      GtsamJacobianType H_plane = JACOBIAN_DEFAULT) const;

  Key getPointKey() const { return pointKey_; }

  Key getPlaneKey() const { return planeKey_; }

  // Perform deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const;

 protected:
  Key pointKey_;
  Key planeKey_;
};

}  // namespace gtsam
