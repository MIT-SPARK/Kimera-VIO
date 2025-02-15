/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * PointPlaneFactor.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Antoni Rosinol
 */

#include "kimera-vio/factors/PointPlaneFactor.h"

using namespace std;

namespace gtsam {

PointPlaneFactor::PointPlaneFactor() {}

PointPlaneFactor::~PointPlaneFactor() {}

PointPlaneFactor::PointPlaneFactor(const Key& pointKey,
                                   const Key& planeKey,
                                   const SharedNoiseModel& noiseModel)
    : Base(noiseModel, pointKey, planeKey),
      pointKey_(pointKey),
      planeKey_(planeKey) {}

void PointPlaneFactor::print(const string& s,
                             const KeyFormatter& keyFormatter) const {
  std::cout << s << " Factor on point " << keyFormatter(pointKey_)
            << ", and plane " << keyFormatter(planeKey_) << "\n";
  this->noiseModel_->print("  noise model: ");
}

Vector PointPlaneFactor::evaluateError(const Point3& point,
                                       const OrientedPlane3& plane,
                                       GtsamJacobianType H_point,
                                       GtsamJacobianType H_plane) const {
  Vector err(1);
  Unit3 plane_normal = plane.normal();
  double plane_distance = plane.distance();

  if (H_point) {
    *H_point = plane_normal.unitVector().transpose();
  }

  if (H_plane) {
    Matrix43 H_plane_retract;
    // Jacobian of plane retraction when v = Vector3::Zero(), to speed-up
    // computations.
    H_plane_retract << plane_normal.basis(), Vector3::Zero(), 0, 0, 1;
    Vector4 p;
    p << point, -1;
    *H_plane = p.transpose() * H_plane_retract;
  }

  err << point.dot(plane_normal.unitVector()) - plane_distance;
  return (err);
}

gtsam::NonlinearFactor::shared_ptr PointPlaneFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new PointPlaneFactor(*this));
}

}  // namespace gtsam
