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
    : cam_params_(cam_params),
      calibration_(cam_params.intrinsics_.at(0),
                   cam_params.intrinsics_.at(1),
                   0.0,  // No skew
                   cam_params.intrinsics_.at(2),
                   cam_params.intrinsics_.at(3)),
      undistorter_(nullptr),
      camera_impl_(nullptr) {
  // NOTE: no rectification, use camera matrix as P for cv::undistortPoints
  // see https://stackoverflow.com/questions/22027419/bad-results-when-undistorting-points-using-opencv-in-python
  cv::Mat P = cam_params.K_;
  cv::Mat R = cv::Mat::eye(3,3,CV_32FC1);
  undistorter_ = VIO::make_unique<UndistorterRectifier>(P, cam_params_, R);
  CHECK(undistorter_);

  camera_impl_ =
      VIO::make_unique<CameraImpl>(cam_params.body_Pose_cam_, calibration_);
  CHECK(camera_impl_);
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
  CHECK(camera_impl_);
  const gtsam::Point2& kp =
      camera_impl_->project2(gtsam::Point3(lmk.x, lmk.y, lmk.z));
  *kpt = KeypointCV(kp.x(), kp.y());
}

void Camera::backProject(const KeypointsCV& kps,
                         const Depths& depths,
                         LandmarksCV* lmks) const {
  CHECK_NOTNULL(lmks)->clear();
  lmks->reserve(kps.size());
  CHECK_EQ(kps.size(), depths.size());
  CHECK(camera_impl_);
  for (size_t i = 0u; i < kps.size(); i++) {
    LandmarkCV lmk;
    backProject(kps[i], depths[i], &lmk);
    lmks->push_back(lmk);
  }
}

void Camera::backProject(const KeypointCV& kp,
                         const Depth& depth,
                         LandmarkCV* lmk) const {
  CHECK_NOTNULL(lmk);
  CHECK(camera_impl_);
  CHECK_GT(depth, 0.0);
  CHECK_GE(kp.x, 0.0);
  CHECK_GE(kp.y, 0.0);
  CHECK_LT(kp.x, cam_params_.image_size_.width);
  CHECK_LT(kp.y, cam_params_.image_size_.height);
  gtsam::Point2 uv(kp.x, kp.y);
  gtsam::Point3 gtsam_lmk = camera_impl_->backproject(uv, depth);
  lmk->x = gtsam_lmk.x();
  lmk->y = gtsam_lmk.y();
  lmk->z = gtsam_lmk.z();
}

// TODO(marcus): copy-pasted from StereoCamera.
//  Maybe there should be some inheritance..
void Camera::undistortKeypoints(const KeypointsCV& keypoints,
                                StatusKeypointsCV* status_keypoints) const {
  KeypointsCV undistorted_keypoints;
  CHECK(undistorter_);
  undistorter_->undistortRectifyKeypoints(
      keypoints, &undistorted_keypoints);
  undistorter_->checkUndistortedRectifiedLeftKeypoints(
      keypoints, undistorted_keypoints, status_keypoints);
}

}  // namespace VIO
