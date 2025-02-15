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
#include <opencv2/core.hpp>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/RgbdFrame.h"
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
   * @param left_cam_params color camera matrix
   */
  explicit RgbdCamera(const CameraParams& cam_params);

  virtual ~RgbdCamera() = default;

  /**
   * @brief Distort keypoints according to camera distortion model
   * @note used for creating fake keypoints in the hallucinated right frame.
   * May not be needed
   */
  void distortKeypoints(const StatusKeypointsCV& keypoints_undistorted,
                        KeypointsCV* keypoints) const;

  /**
   * @brief Get gtsam::Cal3_S2Stereo from rgbd camera and virtual baseline
   */
  StereoCalibPtr getFakeStereoCalib() const;

  gtsam::StereoCamera getFakeStereoCamera() const;

 public:
  /**
   * @brief convertRgbdToPointcloud
   * @param[in] rgbd_frame Frame holding RGB + Depth data
   * @param[out] cloud A cv::Mat_<cv::Point3f> with the same size as the
   * intensity/depth
   * image and with each entry per pixel representing the corresponding 3D
   * point in the camera frame of reference.
   * @param[out] colors A color for each point in the cloud. Same layout
   * than the pointcloud.
   */
  void convertRgbdToPointcloud(const RgbdFrame& rgbd_frame,
                               cv::Mat* cloud,
                               cv::Mat* colors);

 protected:
  // TODO(Toni): put this in the DepthCameraParams struct
  uint16_t depth_factor_ = 1u;
};

}  // namespace VIO
