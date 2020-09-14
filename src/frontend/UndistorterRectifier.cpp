/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   UndistorterRectifier.cpp
 * @brief  Class to undistort (and rectify)
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/UndistorterRectifier.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

UndistorterRectifier::UndistorterRectifier(const cv::Mat& P,
                                           const CameraParams& cam_params,
                                           const cv::Mat& R)
    : P_(P), R_(R), cam_params_(cam_params) {
  initUndistortRectifyMaps(cam_params, R, P, &map_x_, &map_y_);
}

void UndistorterRectifier::undistortRectifyImage(const cv::Mat& img,
                                                 cv::Mat* undistorted_img) {
  CHECK_NOTNULL(undistorted_img);
  CHECK_EQ(map_x_.size, img.size);
  CHECK_EQ(map_y_.size, img.size);
  cv::remap(img,
            *undistorted_img,
            map_x_,
            map_y_,
            remap_interpolation_type_,
            remap_use_constant_border_type_ ? cv::BORDER_CONSTANT
                                            : cv::BORDER_REPLICATE);
}

void UndistorterRectifier::undistortRectifyKeypoints(
    const KeypointsCV& keypoints,
    KeypointsCV* undistorted_keypoints) {
  CHECK_NOTNULL(undistorted_keypoints)->clear();
  switch (cam_params_.distortion_model_) {
    case DistortionModel::RADTAN: {
      cv::undistortPoints(keypoints,
                          *undistorted_keypoints,
                          cam_params_.K_,
                          cam_params_.distortion_coeff_mat_,
                          R_,
                          P_);
    } break;
    case DistortionModel::EQUIDISTANT: {
      cv::fisheye::undistortPoints(keypoints,
                                   *undistorted_keypoints,
                                   cam_params_.K_,
                                   cam_params_.distortion_coeff_mat_,
                                   R_,
                                   P_);
    } break;
    default: { LOG(FATAL) << "Unknown distortion model."; }
  }
}

void UndistorterRectifier::checkUndistortedRectifiedLeftKeypoints(
    const KeypointsCV& distorted_kps,
    const KeypointsCV& undistorted_kps,
    StatusKeypointsCV* status_kps,
    const float& pixel_tol) {
  CHECK_NOTNULL(status_kps)->clear();
  CHECK_EQ(distorted_kps.size(), undistorted_kps.size());
  status_kps->reserve(distorted_kps.size());

  int invalid_count = 0;
  for (size_t i = 0u; i < undistorted_kps.size(); i++) {
    // cropToSize modifies keypoints, so we have to copy.
    KeypointCV distorted_kp = distorted_kps[i];
    KeypointCV undistorted_kp = undistorted_kps[i];
    bool cropped = UtilsOpenCV::cropToSize(&undistorted_kp, map_x_.size());

    // TODO(Toni): would be nicer to interpolate exact position.
    float expected_distorted_kp_x = map_x_.at<float>(
        std::round(undistorted_kp.y), std::round(undistorted_kp.x));
    float expected_distorted_kp_y = map_y_.at<float>(
        std::round(undistorted_kp.y), std::round(undistorted_kp.x));

    if (cropped) {
      VLOG(5) << "Undistorted Rectified keypoint out of image!\n"
               << "Keypoint undistorted: \n"
               << undistorted_kp << '\n'
               << "Image Size (map x size): " << map_x_.size;
      invalid_count += 1;
      status_kps->push_back(
          std::make_pair(KeypointStatus::NO_LEFT_RECT, undistorted_kp));
    } else {
      if (std::fabs(distorted_kp.x - expected_distorted_kp_x) > pixel_tol ||
          std::fabs(distorted_kp.y - expected_distorted_kp_y) > pixel_tol) {
        // Mark as invalid all pixels that were undistorted out of the frame
        // and for which the undistorted rectified keypoint remaps close to
        // the distorted unrectified pixel.
        VLOG(5) << "Pixel mismatch when checking undistortRectification! \n"
                 << "Actual undistorted Keypoint: \n"
                 << " - x: " << undistorted_kp.x << '\n'
                 << " - y: " << undistorted_kp.y << '\n'
                 << "Expected undistorted Keypoint: \n"
                 << " - x: " << expected_distorted_kp_x << '\n'
                 << " - y: " << expected_distorted_kp_y;
        // Invalid points.
        invalid_count += 1;
        status_kps->push_back(
            std::make_pair(KeypointStatus::NO_LEFT_RECT, undistorted_kp));
      } else {
        // Point is valid!
        status_kps->push_back(
            std::make_pair(KeypointStatus::VALID, undistorted_kp));
      }
    }
  }

  VLOG_IF(10, invalid_count > 0) << "undistortRectifyPoints: unable to match "
                                 << invalid_count << " keypoints";
}

void UndistorterRectifier::distortUnrectifyKeypoints(
    const StatusKeypointsCV& keypoints_rectified,
    KeypointsCV* keypoints_unrectified) const {
  CHECK(keypoints_unrectified);
  keypoints_unrectified->clear();
  keypoints_unrectified->reserve(keypoints_rectified.size());
  for (size_t i = 0; i < keypoints_rectified.size(); i++) {
    if (keypoints_rectified[i].first == KeypointStatus::VALID) {
      KeypointCV px = keypoints_rectified[i].second;
      auto x = map_x_.at<float>(round(px.y), round(px.x));
      auto y = map_y_.at<float>(round(px.y), round(px.x));
      keypoints_unrectified->push_back(KeypointCV(x, y));
    } else {
      keypoints_unrectified->push_back(KeypointCV(0.0, 0.0));
    }
  }
}

void UndistorterRectifier::initUndistortRectifyMaps(
    const CameraParams& cam_params,
    const cv::Mat& R,
    const cv::Mat& P,
    cv::Mat* map_x,
    cv::Mat* map_y) {
  CHECK_NOTNULL(map_x);
  CHECK_NOTNULL(map_y);
  static constexpr int kImageType = CV_32FC1;
  // static constexpr int kImageType = CV_16SC2;

  cv::Mat map_x_float, map_y_float;
  switch (cam_params.distortion_model_) {
    case DistortionModel::NONE: {
      map_x_float.create(cam_params.image_size_, kImageType);
      map_y_float.create(cam_params.image_size_, kImageType);
    } break;
    case DistortionModel::RADTAN: {
      cv::initUndistortRectifyMap(
          // Input
          cam_params.K_,
          cam_params.distortion_coeff_mat_,
          R,
          P,
          cam_params.image_size_,
          kImageType,
          // Output:
          map_x_float,
          map_y_float);
    } break;
    case DistortionModel::EQUIDISTANT: {
      cv::fisheye::initUndistortRectifyMap(
          // Input,
          cam_params.K_,
          cam_params.distortion_coeff_mat_,
          R,
          P,
          cam_params.image_size_,
          kImageType,
          // Output:
          map_x_float,
          map_y_float);
    } break;
    default: {
      LOG(FATAL) << "Unknown distortion model: "
                 << VIO::to_underlying(cam_params.distortion_model_);
    }
  }

  // TODO(marcus): can we add this in without causing errors like before?
  // The reason we convert from floating to fixed-point representations
  // of a map is that they can yield much faster (~2x) remapping operations.
  // cv::convertMaps(map_x_float, map_y_float, *map_x, *map_y, CV_16SC2, false);

  *map_x = map_x_float;
  *map_y = map_y_float;
}

}  // namespace VIO
