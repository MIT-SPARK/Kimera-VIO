/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdCamera.h
 * @brief  Class describing a RgbdCamera.
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/frontend/rgbd/RgbdFrame.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

// TODO(Toni): put this in utils rather. And make it template on unit type:
// can be either float or uint16_t...
// Encapsulate differences between processing float and uint16_t depths
template <typename T>
struct DepthTraits {};

template <>
struct DepthTraits<uint16_t> {
  static inline bool valid(uint16_t depth) { return depth != 0; }
  // Originally mm
  static inline float toMeters(uint16_t depth) { return depth * 0.001f; }
};

template <>
struct DepthTraits<float> {
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(float depth) { return depth; }
};

/**
 * @brief The RgbdCamera class An implementation of RGB-D camera.
 * Currently assumes left and depth cameras are registered and undistorted.
 */
class RgbdCamera : public Camera {
 public:
  KIMERA_POINTER_TYPEDEFS(RgbdCamera);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RgbdCamera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief RgbdCamera definition of what a Rgbd Camera is. Computes
   * rectification and undistortion parameters that can be readily applied
   * to stereo images.
   * @param left_cam_params
   * @param depth_cam_params
   */
  RgbdCamera(const CameraParams& cam_params) : Camera(cam_params) {}

  virtual ~RgbdCamera() = default;

 public:
  /**
   * @brief convertRgbdToPointcloud
   * @param intrinsics Intrinsics of the camera for which we generate the
   * pointcloud
   * @return A cv::Mat_<cv::Point3f> with the same size as the intensity/depth
   * image and with each entry representing the corresponding 3D point in the
   * camera frame of reference.
   */
  cv::Mat convertRgbdToPointcloud(const RgbdFrame& rgbd_frame) {
    CHECK(rgbd_frame.intensity_img_);
    CHECK(rgbd_frame.depth_img_);
    const auto& depth_type = rgbd_frame.depth_img_->depth_img_.type();
    if (depth_type == CV_16UC1) {
      return convert<uint16_t>(rgbd_frame.intensity_img_->img_,
                               rgbd_frame.depth_img_->depth_img_,
                               cam_params_.intrinsics_,
                               depth_factor_);
    } else if (depth_type == CV_32FC1) {
      return convert<float>(rgbd_frame.intensity_img_->img_,
                            rgbd_frame.depth_img_->depth_img_,
                            cam_params_.intrinsics_,
                            static_cast<float>(depth_factor_));

    } else {
      LOG(FATAL) << "Unrecognized depth image type.";
    }
  }

 private:
  template <typename T>
  cv::Mat convert(const cv::Mat& intensity_img,
                  const cv::Mat& depth_img,
                  const CameraParams::Intrinsics& intrinsics,
                  const T& depth_factor) {
    CHECK_EQ(intensity_img.type(), CV_8UC1);
    CHECK(depth_img.type() == CV_16UC1 || depth_img.type() == CV_32FC1);
    CHECK_EQ(depth_img.size(), intensity_img.size());

    int img_height = intensity_img.rows;
    int img_width = intensity_img.cols;

    // Use correct principal point from calibration
    float center_x = intrinsics.at(2u);
    float center_y = intrinsics.at(3u);

    // Combine unit conversion (if necessary) with scaling by focal length for
    // computing (X,Y)
    // 0.001f if uint16_t, 1.0f if floats in depth_msg
    // float unit_scaling = 0.001f;
    double unit_scaling = DepthTraits<T>::toMeters(T(1));
    float constant_x = unit_scaling / intrinsics.at(0u);
    float constant_y = unit_scaling / intrinsics.at(1u);
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    cv::Mat_<cv::Point3f> cloud_output =
        cv::Mat(img_height, img_width, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));

    for (int v = 0u; v < img_height; ++v) {
      for (int u = 0u; u < img_width; ++u) {
        T depth = depth_img.at<T>(v, u) * depth_factor;
        cv::Point3f& xyz = cloud_output.at<cv::Point3f>(v, u);

        // Check for invalid measurements
        // TODO(Toni): could clip depth here...
        if (!DepthTraits<T>::valid(depth)) {
          xyz.x = bad_point;
          xyz.y = bad_point;
          xyz.z = bad_point;
        } else {
          // Fill in XYZ
          xyz.x = (u - center_x) * depth * constant_x;
          xyz.y = (v - center_y) * depth * constant_y;
          xyz.z = depth * unit_scaling;
        }

        // TODO(Toni): perhaps use a cv::Point7f to store color as well?
        // uint8_t color = intensity_msg.at<uint8_t>(v, u);
        // Fill in color
        // xyzrgba.a = 255;
        // xyzrgba.r = color;
        // xyzrgba.g = color;
        // xyzrgba.b = color;
      }
    }

    return cloud_output;
  }

 protected:
  // TODO(Toni): put this in the DepthCameraParams struct
  uint16_t depth_factor_ = 1u;
};

}  // namespace VIO
