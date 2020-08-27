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
                                           const cv::Mat& R) {
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

void UndistorterRectifier::initUndistortRectifyMaps(
    const CameraParams& cam_params,
    const cv::Mat& R,
    const cv::Mat& P,
    cv::Mat* map_x,
    cv::Mat* map_y) {
  CHECK_NOTNULL(map_x);
  CHECK_NOTNULL(map_y);
  static constexpr int kImageType = CV_32FC1;
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

  // The reason we convert from floating to fixed-point representations
  // of a map is that they can yield much faster (~2x) remapping operations.
  cv::convertMaps(map_x_float, map_y_float, *map_x, *map_y, CV_16SC2, false);
}

}  // namespace VIO
