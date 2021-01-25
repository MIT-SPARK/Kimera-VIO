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

/**
 * Factor to implement error between a point landmark and a plane.
 */
class PointPlaneFactor: public NoiseModelFactor2<Point3, OrientedPlane3> {

protected:
  Key pointKey_;
  Key planeKey_;

  typedef NoiseModelFactor2<Point3, OrientedPlane3> Base;

public:

  /// Constructor
  PointPlaneFactor() {
  }
  virtual ~PointPlaneFactor() {}

  /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
  PointPlaneFactor(
      const Key& pointKey, const Key& planeKey, const SharedNoiseModel& noiseModel) :
      Base(noiseModel, pointKey, planeKey), pointKey_(pointKey), planeKey_(planeKey) {
  }

  /// print
  virtual void print(const std::string& s = "RegularPlane3Factor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// evaluateError
  /// Hpoint: jacobian of h wrt point landmark
  /// Hplane: jacobian of h wrt plane
  virtual Vector evaluateError(const Point3& point, const OrientedPlane3& plane,
                               boost::optional<Matrix&> H_point = boost::none,
                               boost::optional<Matrix&> H_plane = boost::none) const {
    Vector err(1);
    Unit3 plane_normal = plane.normal();
    double plane_distance = plane.distance();

    if (H_point) *H_point = plane_normal.unitVector().transpose();
    if (H_plane) {
      Matrix43 H_plane_retract;
      // Jacobian of plane retraction when v = Vector3::Zero(), to speed-up
      // computations.
      H_plane_retract << plane_normal.basis(), Vector3::Zero(), 0, 0, 1;
      Vector4 p;
      p << point, -1;
      *H_plane =  p.transpose() * H_plane_retract;
    }

    err << point.dot(plane_normal.unitVector()) - plane_distance;
    return (err);
  }

  inline Key getPointKey() const {
    return pointKey_;
  }

  inline Key getPlaneKey() const {
    return planeKey_;
  }


  // Perform deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const;
};

} // gtsam

