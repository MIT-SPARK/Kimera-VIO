/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Camera.h
 * @brief  Class describing a monocular camera
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/Camera.h"

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <gtsam/geometry/Point2.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

Camera::Camera(const CameraParams& cam_params)
    : camera_impl_(nullptr),
      calibration_(cam_params.intrinsics_.at(0),
                   cam_params.intrinsics_.at(1),
                   0.0,  // No skew
                   cam_params.intrinsics_.at(2),
                   cam_params.intrinsics_.at(3)),
      cam_params_(cam_params) {
  camera_impl_ = VIO::make_unique<CameraImpl>(cam_params.body_Pose_cam_,
                                              calibration_);
  CHECK_NOTNULL(camera_impl_);
}

void Camera::project(const LandmarksCV& lmks, KeypointsCV* kpts) const {
  CHECK_NOTNULL(kpts)->clear();
  const auto& n_lmks = lmks.size();
  kpts->resize(n_lmks);
  // Can be greatly optimized with matrix mult or vectorization
  for (size_t i = 0u; i < n_lmks; i++) {
    project(lmks[i], &(*kpts)[i]);
  }
}

void Camera::project(const LandmarkCV& lmk, KeypointCV* kpt) const {
  CHECK_NOTNULL(kpt);
  // I believe this project call in gtsam is quite inefficient as it goes
  // through a cascade of calls... Only useful if you want gradients I guess...
  const gtsam::Point2& kp =
      CHECK_NOTNULL(camera_impl_)->project2(gtsam::Point3(lmk.x, lmk.y, lmk.z));
  *kpt = KeypointCV(kp.x(), kp.y());
}

void Camera::backProject(const KeypointsCV& kps,
                         const double& depth,
                         LandmarksCV* lmks) const {
  CHECK_NOTNULL(lmks)->clear();
  lmks->reserve(kps.size());
  for (const KeypointCV& kp : kps) {
    gtsam::Point2 z(kp.x, kp.y);
    gtsam::Point3 lmk = CHECK_NOTNULL(camera_impl_)->backproject(z, depth);
    lmks->push_back(LandmarkCV(lmk.x(), lmk.y(), lmk.z()));
  }
}

}  // namespace VIO
