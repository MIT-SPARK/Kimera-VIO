/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdCamera.cpp
 * @brief  Class describing a RgbdCamera.
 * @author Antoni Rosinol
 * @author Nathan Hughes
 */

#include "kimera-vio/frontend/RgbdCamera.h"

#include "kimera-vio/frontend/Camera.h"

namespace VIO {

template <typename T>
void convertToPcl(const cv::Mat& intensity_img,
                  const cv::Mat& depth_img,
                  const CameraParams::Intrinsics& intrinsics,
                  const T& depth_factor,
                  cv::Mat* cloud,
                  cv::Mat* colors) {
  CHECK_NOTNULL(cloud);
  CHECK_NOTNULL(colors);
  CHECK_EQ(intensity_img.type(), CV_8UC1);
  CHECK(depth_img.type() == CV_16UC1 || depth_img.type() == CV_32FC1);
  CHECK_EQ(depth_img.size(), intensity_img.size());

  int img_rows = intensity_img.rows;
  int img_cols = intensity_img.cols;

  // Use correct principal point from calibration
  float center_x = intrinsics.at(2u);
  float center_y = intrinsics.at(3u);

  // Combine unit conversion (if necessary) with scaling by focal length
  // for computing (X,Y) 0.001f if uint16_t, 1.0f if floats in depth_msg
  // float unit_scaling = 0.001f;
  double unit_scaling = DepthTraits<T>::toMeters(T(1));
  float constant_x = unit_scaling / intrinsics.at(0u);
  float constant_y = unit_scaling / intrinsics.at(1u);
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  cv::Mat_<cv::Point3f> cloud_out =
      cv::Mat(img_rows, img_cols, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
  cv::Mat colors_out =
      cv::Mat(img_rows, img_cols, CV_8UC3, cv::viz::Color::red());

  for (int v = 0u; v < img_rows; ++v) {
    for (int u = 0u; u < img_cols; ++u) {
      T depth = depth_img.at<T>(v, u) * depth_factor;
      cv::Point3f& xyz = cloud_out.at<cv::Point3f>(v, u);
      cv::Vec3b& color = colors_out.at<cv::Vec3b>(v, u);

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

        // Fill in color (grayscale for now)
        const auto& grey_value = intensity_img.at<uint8_t>(v, u);
        color = cv::Vec3b(grey_value, grey_value, grey_value);
      }
    }
  }

  *cloud = cloud_out;
  *colors = colors_out;
}

RgbdCamera::RgbdCamera(const CameraParams& cam_params) : Camera(cam_params) {}

void RgbdCamera::distortKeypoints(
    const StatusKeypointsCV& keypoints_undistorted,
    KeypointsCV* keypoints) const {
  undistorter_->distortUnrectifyKeypoints(keypoints_undistorted, keypoints);
}

StereoCalibPtr RgbdCamera::getFakeStereoCalib() const {
  return StereoCalibPtr(
      new gtsam::Cal3_S2Stereo(calibration_.fx(),
                               calibration_.fy(),
                               calibration_.skew(),
                               calibration_.px(),
                               calibration_.py(),
                               cam_params_.depth.virtual_baseline_));
}

gtsam::StereoCamera RgbdCamera::getFakeStereoCamera() const {
  return {gtsam::Pose3(), getFakeStereoCalib()};
}

void RgbdCamera::convertRgbdToPointcloud(const RgbdFrame& rgbd_frame,
                                         cv::Mat* cloud,
                                         cv::Mat* colors) {
  CHECK_NOTNULL(cloud);
  CHECK_NOTNULL(colors);
  const auto& depth_type = rgbd_frame.depth_img_.depth_img_.type();
  if (depth_type == CV_16UC1) {
    return convertToPcl<uint16_t>(rgbd_frame.intensity_img_.img_,
                                  rgbd_frame.depth_img_.depth_img_,
                                  cam_params_.intrinsics_,
                                  depth_factor_,
                                  cloud,
                                  colors);
  } else if (depth_type == CV_32FC1) {
    return convertToPcl<float>(rgbd_frame.intensity_img_.img_,
                               rgbd_frame.depth_img_.depth_img_,
                               cam_params_.intrinsics_,
                               static_cast<float>(depth_factor_),
                               cloud,
                               colors);

  } else {
    LOG(FATAL) << "Unrecognized depth image type.";
  }
}

}  // namespace VIO
