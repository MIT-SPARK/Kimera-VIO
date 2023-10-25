/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * @file ParallelPlaneRegularFactor.h
 * @brief ParallelPlaneRegular Factor class
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
 * Pure virtual class to implement different factors between planes.
 * (TODO, check BetweenFactor)
 */
class ParallelPlaneRegularFactor
    : public NoiseModelFactor2<OrientedPlane3, OrientedPlane3> {
 protected:
  //// Keys of the planes involved.
  Key plane1Key_;
  Key plane2Key_;

  //// Type of Factor.
  std::string factor_type_;

  //// Measured distance from plane2 to plane1, with the normal of plane2
  /// defining / the sign (a plane1 3 meters away from plane2 in the negative
  /// direction of / the normal of plane2 will have a measured distance of -2).
  // Careful with this convention, it is already encoded in the between factor
  // of GTSAM
  double measured_distance_from_plane2_to_plane1;

  typedef NoiseModelFactor2<OrientedPlane3, OrientedPlane3> Base;

 public:
  /// Constructor
  ParallelPlaneRegularFactor() {}
  virtual ~ParallelPlaneRegularFactor() {}

  ParallelPlaneRegularFactor(
      const Key& plane1Key,
      const Key& plane2Key,
      const SharedNoiseModel& noiseModel,
      const double& measured_distance_from_plane2_to_plane1 = 0)
      : Base(noiseModel, plane1Key, plane2Key),
        plane1Key_(plane1Key),
        plane2Key_(plane2Key),
        factor_type_("None, this is an abstract class."),
        measured_distance_from_plane2_to_plane1(
            measured_distance_from_plane2_to_plane1) {}

  /// print
  void print(const std::string& s = "ParallelPlaneRegularFactor",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// evaluateError
  /// Hplane1: jacobian of h wrt plane1
  /// Hplane2: jacobian of h wrt plane2
  Vector evaluateError(const OrientedPlane3& plane_1,
                       const OrientedPlane3& plane_2,
                       GtsamJacobianType H_plane_1 = JACOBIAN_DEFAULT,
                       GtsamJacobianType H_plane_2 = JACOBIAN_DEFAULT) const {
    return doEvaluateError(plane_1, plane_2, H_plane_1, H_plane_2);
  }

 private:
  //// See non-virtual interface idiom (NVI idiom) for reasons to do this.
  /// Basically it's to avoid re-defining default parameters in derived classes.
  /// See also Item 37 of Effective_C++.
  virtual Vector doEvaluateError(const OrientedPlane3& plane_1,
                                 const OrientedPlane3& plane_2,
                                 GtsamJacobianType H_plane_1,
                                 GtsamJacobianType H_plane_2) const = 0;
};

/**
 * Specialization of a ParallelPlaneRegularFactor using normals error
 * in Tangent Space of S² (geodesic).
 * The error metric only considers parallelism between planes,
 * no distance is enforced.
 */
class ParallelPlaneRegularTangentSpaceFactor
    : public ParallelPlaneRegularFactor {
 public:
  /// Constructor
  ParallelPlaneRegularTangentSpaceFactor() {}
  virtual ~ParallelPlaneRegularTangentSpaceFactor() {}

  ParallelPlaneRegularTangentSpaceFactor(const Key& plane1Key,
                                         const Key& plane2Key,
                                         const SharedNoiseModel& noiseModel)
      : ParallelPlaneRegularFactor(plane1Key, plane2Key, noiseModel) {
    this->factor_type_ = "ParallelPlaneRegularTangentSpaceFactor";
  }

 private:
  /// evaluateError
  /// Hplane1: jacobian of h wrt plane1
  /// Hplane2: jacobian of h wrt plane2
  virtual Vector doEvaluateError(const OrientedPlane3& plane_1,
                                 const OrientedPlane3& plane_2,
                                 GtsamJacobianType H_plane_1,
                                 GtsamJacobianType H_plane_2) const {
    Unit3 plane_normal_1 = plane_1.normal();
    Unit3 plane_normal_2 = plane_2.normal();
    Matrix22 H_n_1, H_n_2;
    Vector2 err;
    err = plane_normal_1.errorVector(plane_normal_2, H_n_1, H_n_2);
    if (H_plane_1) {
      Matrix23 tmp;
      tmp << H_n_1, Vector2::Zero();
      *H_plane_1 = tmp;
    }
    if (H_plane_2) {
      Matrix23 tmp;
      tmp << H_n_2, Vector2::Zero();
      *H_plane_2 = tmp;
    }
    return (err);
  }
};

/**
 * Specialization of a ParallelPlaneRegularFactor using normals error
 * in Tangent Space of S² (geodesic) and distance error in R.
 * The error metric considers parallelism between planes, as well as distance
 * between planes.
 */
class GeneralParallelPlaneRegularTangentSpaceFactor
    : public ParallelPlaneRegularFactor {
 public:
  /// Constructor
  GeneralParallelPlaneRegularTangentSpaceFactor() {}
  virtual ~GeneralParallelPlaneRegularTangentSpaceFactor() {}

  GeneralParallelPlaneRegularTangentSpaceFactor(
      const Key& plane1Key,
      const Key& plane2Key,
      const SharedNoiseModel& noiseModel,
      const double& measured_distance_from_plane2_to_plane1 = 0)
      : ParallelPlaneRegularFactor(plane1Key,
                                   plane2Key,
                                   noiseModel,
                                   measured_distance_from_plane2_to_plane1) {
    this->factor_type_ = "GeneralParallelPlaneRegularTangentSpaceFactor";
  }

 private:
  /// evaluateError
  /// Hplane1: jacobian of h wrt plane1
  /// Hplane2: jacobian of h wrt plane2
  virtual Vector doEvaluateError(const OrientedPlane3& plane_1,
                                 const OrientedPlane3& plane_2,
                                 GtsamJacobianType H_plane_1,
                                 GtsamJacobianType H_plane_2) const {
    Unit3 plane_normal_1 = plane_1.normal();
    Unit3 plane_normal_2 = plane_2.normal();
    Matrix22 H_n_1, H_n_2;
    Vector2 normal_err(
        plane_normal_1.errorVector(plane_normal_2, H_n_1, H_n_2));
    Vector3 err;
    err = Vector3(normal_err(0),
                  normal_err(1),
                  plane_1.distance() - plane_2.distance() -
                      measured_distance_from_plane2_to_plane1);
    if (H_plane_1) {
      Matrix33 a;
      a << H_n_1, Vector2::Zero(), 0, 0, 1;
      *H_plane_1 = a;
    }
    if (H_plane_2) {
      Matrix33 a;
      a << H_n_2, Vector2::Zero(), 0, 0, -1;
      *H_plane_2 = a;
    }
    return (err);
  }
};

/**
 * Specialization of a ParallelPlaneRegularFactor where normals error is in
 *  R³ (chordal).
 * The error metric only considers parallelism between planes,
 * no distance is enforced.
 */
class ParallelPlaneRegularBasicFactor : public ParallelPlaneRegularFactor {
 public:
  /// Constructor
  ParallelPlaneRegularBasicFactor() {}
  virtual ~ParallelPlaneRegularBasicFactor() {}

  ParallelPlaneRegularBasicFactor(const Key& plane1Key,
                                  const Key& plane2Key,
                                  const SharedNoiseModel& noiseModel)
      : ParallelPlaneRegularFactor(plane1Key, plane2Key, noiseModel) {
    this->factor_type_ = "ParallelPlaneRegularBasicFactor";
  }

 private:
  /// evaluateError
  /// Hplane1: jacobian of h wrt plane1
  /// Hplane2: jacobian of h wrt plane2
  virtual Vector doEvaluateError(const OrientedPlane3& plane_1,
                                 const OrientedPlane3& plane_2,
                                 GtsamJacobianType H_plane_1,
                                 GtsamJacobianType H_plane_2) const {
    const Unit3& plane_normal_1 = plane_1.normal();
    const Unit3& plane_normal_2 = plane_2.normal();
    Vector3 err(plane_normal_1.unitVector() - plane_normal_2.unitVector());

    if (H_plane_1) {
      // Jacobian of plane retraction when v = Vector3::Zero(), to speed-up
      // computations.
      Matrix33 tmp;
      tmp << plane_normal_1.basis(), Vector3::Zero();
      *H_plane_1 = tmp;
    }
    if (H_plane_2) {
      // Jacobian of plane retraction when v = Vector3::Zero(), to speed-up
      // computations.
      Matrix33 tmp;
      tmp << -plane_normal_2.basis(), Vector3::Zero();
      *H_plane_2 = tmp;
    }
    return (err);
  }
};

/**
 * Specialization of a ParallelPlaneRegularFactor where normals error is in
 *  R³ (chordal) and the distance error is in R.
 * The error metric considers parallelism between planes, as well as distance
 * between planes.
 */
class GeneralParallelPlaneRegularBasicFactor
    : public ParallelPlaneRegularFactor {
 public:
  /// Constructor
  GeneralParallelPlaneRegularBasicFactor() {}
  virtual ~GeneralParallelPlaneRegularBasicFactor() {}

  GeneralParallelPlaneRegularBasicFactor(
      const Key& plane1Key,
      const Key& plane2Key,
      const SharedNoiseModel& noiseModel,
      const double& measured_distance_from_plane2_to_plane1 = 0)
      : ParallelPlaneRegularFactor(plane1Key,
                                   plane2Key,
                                   noiseModel,
                                   measured_distance_from_plane2_to_plane1) {
    this->factor_type_ = "GeneralParallelPlaneRegularBasicFactor";
  }

 private:
  /// evaluateError
  /// Hplane1: jacobian of h wrt plane1
  /// Hplane2: jacobian of h wrt plane2
  virtual Vector doEvaluateError(const OrientedPlane3& plane_1,
                                 const OrientedPlane3& plane_2,
                                 GtsamJacobianType H_plane_1,
                                 GtsamJacobianType H_plane_2) const {
    const Unit3& plane_normal_1(plane_1.normal());
    const Unit3& plane_normal_2(plane_2.normal());
    Vector4 err(0.0, 0.0, 0.0, 0.0);
    err = Vector4(plane_normal_1.point3().x() - plane_normal_2.point3().x(),
                  plane_normal_1.point3().y() - plane_normal_2.point3().y(),
                  plane_normal_1.point3().z() - plane_normal_2.point3().z(),
                  plane_1.distance() - plane_2.distance() -
                      measured_distance_from_plane2_to_plane1);
    if (H_plane_1) {
      // Jacobian of plane retraction when v = Vector3::Zero(), to speed-up
      // computations.
      Matrix43 tmp;
      tmp << plane_normal_1.basis(), Vector3::Zero(), 0, 0, 1;
      *H_plane_1 = tmp;
    }
    if (H_plane_2) {
      // Jacobian of plane retraction when v = Vector3::Zero(), to speed-up
      // computations.
      Matrix43 tmp;
      tmp << -plane_normal_2.basis(), Vector3::Zero(), 0, 0, -1;
      *H_plane_2 = tmp;
    }
    return (err);
  }
};

}  // namespace gtsam
