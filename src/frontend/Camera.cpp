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
 * @author Marcus Abate
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
  switch (cam_params_.camera_model_) {
    case CameraModel::PINHOLE: {
      projectPinhole(lmks, kpts);
      break;
    }

    case CameraModel::OMNI: {
      projectOmni(lmks, kpts);
      break;
    }
  }
}

void Camera::project(const LandmarkCV& lmk, KeypointCV* kpt) const {
  switch (cam_params_.camera_model_) {
    case CameraModel::PINHOLE: {
      projectPinhole(lmk, kpt);
      break;
    }

    case CameraModel::OMNI: {
      projectOmni(lmk, kpt);
      break;
    }
  }
}

void Camera::backProject(const KeypointsCV& kps,
                         const Depths& depths,
                         LandmarksCV* lmks) const {
  switch (cam_params_.camera_model_) {
    case CameraModel::PINHOLE: {
      backProjectPinhole(kps, depths, lmks);
      break;
    }

    case CameraModel::OMNI: {
      backProjectOmni(kps, depths, lmks);
      break;
    }
  }
}

void Camera::backProject(const KeypointCV& kp,
                         const Depth& depth,
                         LandmarkCV* lmk) const {
  switch (cam_params_.camera_model_) {
    case CameraModel::PINHOLE: {
      backProjectPinhole(kp, depth, lmk);
      break;
    }

    case CameraModel::OMNI: {
      backProjectOmni(kp, depth, lmk);
      break;
    }
  }
}

void Camera::undistortKeypoints(
    const KeypointsCV& keypoints,
    StatusKeypointsCV* status_keypoints) const {
  KeypointsCV undistorted_keypoints;
  CHECK(undistorter_);

  switch (cam_params_.camera_model_) {
    case CameraModel::PINHOLE: {
      undistorter_->undistortRectifyKeypoints(keypoints,
                                              &undistorted_keypoints);
    } break;
    case CameraModel::OMNI: {
      // TODO(marcus): after gtsam camera model, this disappears and
      // the switch happens in UndistorterRectifier.
      Camera::UndistortKeypointsOmni(
          keypoints, cam_params_, cam_params_.K_, &undistorted_keypoints);
    } break;
    default: {
      LOG(FATAL) << "Camera: Unrecognized camera model.";
    }
  }
  undistorter_->checkUndistortedRectifiedLeftKeypoints(
      keypoints, undistorted_keypoints, status_keypoints);
}

void Camera::projectPinhole(const LandmarksCV& lmks, KeypointsCV* kpts) const {
  CHECK_NOTNULL(kpts)->clear();
  const auto& n_lmks = lmks.size();
  kpts->resize(n_lmks);
  // Can be greatly optimized with matrix mult or vectorization
  for (size_t i = 0u; i < n_lmks; i++) {
    projectPinhole(lmks[i], &(*kpts)[i]);
  }
}

void Camera::projectPinhole(const LandmarkCV& lmk, KeypointCV* kpt) const {
  CHECK_NOTNULL(kpt);
  // I believe this project call in gtsam is quite inefficient as it goes
  // through a cascade of calls... Only useful if you want gradients I guess...
  CHECK(camera_impl_);
  const gtsam::Point2& kp =
      camera_impl_->project2(gtsam::Point3(lmk.x, lmk.y, lmk.z));
  // What if the keypoint is out of the image bounds?
  *kpt = KeypointCV(kp.x(), kp.y());
}

void Camera::backProjectPinhole(const KeypointsCV& kps,
                                const Depths& depths,
                                LandmarksCV* lmks) const {
  CHECK_NOTNULL(lmks)->clear();
  lmks->reserve(kps.size());
  CHECK_EQ(kps.size(), depths.size());
  for (size_t i = 0u; i < kps.size(); i++) {
    LandmarkCV lmk;
    backProjectPinhole(kps[i], depths[i], &lmk);
    lmks->push_back(lmk);
  }
}

void Camera::backProjectPinhole(const KeypointCV& kp,
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

void Camera::projectOmni(const LandmarksCV& lmks, KeypointsCV* kpts) const {
  CHECK_NOTNULL(kpts)->clear();
  const auto& n_lmks = lmks.size();
  kpts->resize(n_lmks);
  // Can be greatly optimized with matrix mult or vectorization
  for (size_t i = 0u; i < n_lmks; i++) {
    projectOmni(lmks[i], &(*kpts)[i]);
  }
}

void Camera::projectOmni(const LandmarkCV& lmk, KeypointCV* kpt) const {
  LOG(FATAL) << "Camera: projectOmni not implemented!";
}

void Camera::backProjectOmni(const KeypointsCV& kps,
                             const Depths& depths,
                             LandmarksCV* lmks) const {
  Camera::BackProjectOmni(kps, depths, cam_params_, lmks);
}

void Camera::backProjectOmni(const KeypointCV& kp,
                             const Depth& depth,
                             LandmarkCV* lmk) const {
  Camera::BackProjectOmni(kp, depth, cam_params_, lmk);
}

void Camera::BackProjectOmni(const KeypointsCV& kps,
                             const Depths& depths,
                             const CameraParams& cam_params,
                             LandmarksCV* lmks) {
  CHECK_NOTNULL(lmks)->clear();
  lmks->reserve(kps.size());
  CHECK_EQ(kps.size(), depths.size());
  for (size_t i = 0u; i < kps.size(); i++) {
    LandmarkCV lmk;
    Camera::BackProjectOmni(kps[i], depths[i], cam_params, &lmk);
    lmks->push_back(lmk);
  }
}

void Camera::BackProjectOmni(const KeypointCV& kp,
                            const Depth& depth,
                            const CameraParams& cam_params,
                            LandmarkCV* lmk) {
  CHECK_NOTNULL(lmk);

  Eigen::Vector2d keypoint(kp.x, kp.y);
  const Eigen::Vector2d rectified =
      cam_params.omni_affine_inv_ *
      (keypoint - cam_params.omni_distortion_center_);
  const double rho = rectified.norm();

  Eigen::Vector3d out_bearing_vector;
  out_bearing_vector.head<2>() = rectified;

  CHECK_EQ(cam_params.distortion_coeff_.size(), 5);
  out_bearing_vector(2) = cam_params.distortion_coeff_.at(4);
  out_bearing_vector(2) =
      cam_params.distortion_coeff_.at(3) + out_bearing_vector(2) * rho;
  out_bearing_vector(2) =
      cam_params.distortion_coeff_.at(2) + out_bearing_vector(2) * rho;
  out_bearing_vector(2) =
      cam_params.distortion_coeff_.at(1) + out_bearing_vector(2) * rho;
  out_bearing_vector(2) =
      cam_params.distortion_coeff_.at(0) + out_bearing_vector(2) * rho;
  out_bearing_vector(2) = (1.0) * out_bearing_vector(2);
  CHECK_NE(out_bearing_vector(2), 0.0)
      << "Camera: backProjectOmni is trying to divide by zero!";

  lmk->x = out_bearing_vector(0) * depth / out_bearing_vector(2);
  lmk->y = out_bearing_vector(1) * depth / out_bearing_vector(2);
  lmk->z = depth;
}

void Camera::UndistortKeypointsOmni(const KeypointsCV& keypoints,
                                    const CameraParams& cam_params,
                                    boost::optional<cv::Mat> P,
                                    KeypointsCV* undistorted_keypoints) {
  CHECK_NOTNULL(undistorted_keypoints)->clear();
  Depths unit_depths;
  for (size_t i = 0; i < keypoints.size(); i++) {
    unit_depths.push_back(1.0);  // projection to canonical camera frame
  }
  LandmarksCV lmks;
  Camera::BackProjectOmni(keypoints, unit_depths, cam_params, &lmks);

  CHECK_EQ(keypoints.size(), lmks.size());
  for (size_t i = 0; i < lmks.size(); i++) {
    KeypointCV kpt_undistorted;
    if (P == boost::none) {
      kpt_undistorted = KeypointCV(lmks.at(i).x, lmks.at(i).y);
    } else {
      gtsam::Cal3_S2 cal = UtilsOpenCV::Cvmat2Cal3_S2(P.get());
      gtsam::Point2 kpt_undistorted_gtsam =
          cal.uncalibrate(gtsam::Point2(lmks.at(i).x, lmks.at(i).y));
      kpt_undistorted = KeypointCV(kpt_undistorted_gtsam.x(),
                                   kpt_undistorted_gtsam.y());
    }
    undistorted_keypoints->push_back(kpt_undistorted);
  }
}

}  // namespace VIO
