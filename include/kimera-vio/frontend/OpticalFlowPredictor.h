/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpticalFlowPredictor.h
 * @brief  Class that predicts optical flow between two images. This is
 * helpful for the tracker to have a good initial guess for feature tracking.
 * @author Antoni Rosinol
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Rot3.h>

#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

class OpticalFlowPredictor {
 public:
  KIMERA_POINTER_TYPEDEFS(OpticalFlowPredictor);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpticalFlowPredictor);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OpticalFlowPredictor() = default;
  virtual ~OpticalFlowPredictor() = default;

  /**
   * @brief predictFlow Predicts optical flow for a set of image keypoints.
   * The optical flow determines the position of image features in consecutive
   * frames.
   * @param prev_kps: keypoints in previous (reference) image
   * @param cam1_R_cam2: rotation from camera 1 to camera 2.
   * @param next_kps: keypoints in next image.
   * @return true if flow could be determined successfully
   */
  virtual bool predictFlow(const KeypointsCV& prev_kps,
                           const gtsam::Rot3& cam1_R_cam2,
                           KeypointsCV* next_kps) = 0;
};

/**
 * @brief The NoOpticalFlowPredictor class just assumes that the camera
 * did not move and so the features on the previous frame remain at the same
 * pixel positions in the current frame.
 */
class NoOpticalFlowPredictor : public OpticalFlowPredictor {
 public:
  KIMERA_POINTER_TYPEDEFS(NoOpticalFlowPredictor);
  KIMERA_DELETE_COPY_CONSTRUCTORS(NoOpticalFlowPredictor);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NoOpticalFlowPredictor() = default;
  virtual ~NoOpticalFlowPredictor() = default;

  bool predictFlow(const KeypointsCV& prev_kps,
                   const gtsam::Rot3& /* inter_frame */,
                   KeypointsCV* next_kps) override {
    *CHECK_NOTNULL(next_kps) = prev_kps;
    return true;
  }
};

/**
 * @brief The RotationalOpticalFlowPredictor class predicts optical flow
 * by using a guess of inter-frame rotation and assumes no translation btw
 * frames.
 */
class RotationalOpticalFlowPredictor : public OpticalFlowPredictor {
 public:
  KIMERA_POINTER_TYPEDEFS(RotationalOpticalFlowPredictor);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RotationalOpticalFlowPredictor);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RotationalOpticalFlowPredictor(const cv::Matx33f& K, const cv::Size& img_size)
      : K_(K),
        K_inverse_(K.inv()),
        img_size_(0.0f, 0.0f, img_size.width, img_size.height) {}
  virtual ~RotationalOpticalFlowPredictor() = default;

  bool predictFlow(const KeypointsCV& prev_kps,
                   const gtsam::Rot3& cam1_R_cam2,
                   KeypointsCV* next_kps) override {
    CHECK_NOTNULL(next_kps);

    // Handle case when rotation is small: just copy prev_kps
    // Removed bcs even small rotations lead to huge optical flow at the borders
    // of the image.
    // Keep because then you save a lot of computation.
    static constexpr double kSmallRotationTol = 1e-4;
    if (std::abs(1.0 - std::abs(cam1_R_cam2.quaternion()[0])) <
        kSmallRotationTol) {
      *next_kps = prev_kps;
      return true;
    }

    // We use a new object in case next_kps is pointing to prev_kps!
    KeypointsCV predicted_kps;

    // R is a relative rotation which takes a vector from the last frame to
    // the current frame.
    cv::Matx33f R = UtilsOpenCV::gtsamMatrix3ToCvMat(cam1_R_cam2.matrix());
    // Get bearing vector for kpt, rotate knowing frame to frame rotation,
    // get keypoints again
    bool is_ok;
    cv::Matx33f H = K_ * R.inv(cv::DECOMP_LU, &is_ok) * K_inverse_;
    CHECK(is_ok);
    const size_t& n_kps = prev_kps.size();
    predicted_kps.reserve(n_kps);
    for (size_t i = 0u; i < n_kps; ++i) {
      // Create homogeneous keypoints.
      const KeypointCV& prev_kpt = prev_kps[i];
      cv::Vec3f p1(prev_kpt.x, prev_kpt.y, 1.0f);

      cv::Vec3f p2 = H * p1;

      // Project predicted bearing vectors to 2D again and re-homogenize.
      KeypointCV new_kpt;
      if (p2[2] > 0.0f) {
        new_kpt = KeypointCV(p2[0] / p2[2], p2[1] / p2[2]);
      } else {
        LOG(WARNING) << "Landmark behind the camera:\n"
                     << "- p1: " << p1 << '\n'
                     << "- p2: " << p2;
        new_kpt = prev_kpt;
      }
      // Check that keypoints remain inside the image boundaries!
      if (img_size_.contains(new_kpt)) {
        predicted_kps.push_back(new_kpt);
      } else {
        // Otw copy-paste previous keypoint.
        predicted_kps.push_back(prev_kpt);
      }
    }

    *next_kps = predicted_kps;
    return true;
  }

 private:
  const cv::Matx33f K_;          // Intrinsic matrix of camera
  const cv::Matx33f K_inverse_;  // Cached inverse of K
  const cv::Rect2f img_size_;
};

}  // namespace VIO
