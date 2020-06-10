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
    return convert(rgbd_frame.intensity_img_->img_,
                   rgbd_frame.depth_img_->depth_img_,
                   cam_params_.intrinsics_,
                   depth_factor_);
  }

 private:
  // TODO(Toni): put this in utils rather. And make it template on unit type:
  // can be either float or uint16_t...
  static cv::Mat convert(const cv::Mat& intensity_img,
                         const cv::Mat& depth_img,
                         const CameraParams::Intrinsics& intrinsics,
                         const uint16_t& depth_factor) {
    CHECK_EQ(intensity_img.type(), CV_8UC1);
    CHECK_EQ(depth_img.type(), CV_16UC1);
    int img_height = intensity_img.rows;
    int img_width = intensity_img.cols;

    // Use correct principal point from calibration
    float center_x = intrinsics.at(2u);
    float center_y = intrinsics.at(3u);

    // Combine unit conversion (if necessary) with scaling by focal length for
    // computing (X,Y)
    // 0.001f if uint16_t, 1.0f if floats in depth_msg
    float unit_scaling = 0.001f;
    float constant_x = unit_scaling / intrinsics.at(0u);
    float constant_y = unit_scaling / intrinsics.at(1u);
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    cv::Mat_<cv::Point3f> cloud_msg = cv::Mat(img_height, img_width, CV_32FC3);

    for (int v = 0u; v < img_height; ++v) {
      for (int u = 0u; u < img_width; ++u) {
        uint16_t depth = depth_img.at<uint16_t>(v, u) / depth_factor;
        cv::Point3f& xyz = cloud_msg.at<cv::Point3f>(v, u);

        // Check for invalid measurements
        xyz.x = (u - center_x) * depth * constant_x;
        xyz.y = (v - center_y) * depth * constant_y;
        if (depth != 0u) {
          xyz.z = bad_point;
        } else {
          // Fill in XYZ
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
  }

 protected:
  // TODO(Toni): put this in the DepthCameraParams struct
  uint16_t depth_factor_ = 1u;
};

}  // namespace VIO
