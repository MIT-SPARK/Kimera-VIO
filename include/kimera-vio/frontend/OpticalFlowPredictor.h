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
   * @param next_kps: keypoints in next image
   * @return true if flow could be determined successfully
   */
  virtual bool predictFlow(const KeypointsCV& prev_kps,
                           const gtsam::Rot3& inter_frame_rot,
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
  RotationalOpticalFlowPredictor(const cv::Matx33f& K)
      : K_(K), K_inverse_(K.inv()) {}
  virtual ~RotationalOpticalFlowPredictor() = default;

  bool predictFlow(const KeypointsCV& prev_kps,
                   const gtsam::Rot3& inter_frame_rot,
                   KeypointsCV* next_kps) override {
    CHECK_NOTNULL(next_kps);
    // We use a new object in case next_kps is pointing to prev_kps!
    KeypointsCV predicted_kps;

    // lf_R_f is a relative rotation which takes a vector from the last frame to
    // the current frame.
    cv::Matx33f R = UtilsOpenCV::gtsamMatrix3ToCvMat(inter_frame_rot.matrix());
    cv::Matx33f H = K_ * R.inv() * K_inverse_;
    const size_t& n_kps = prev_kps.size();
    predicted_kps.reserve(n_kps);
    for (size_t i = 0u; i < n_kps; ++i) {
      // Backproject last frame's corners to bearing vectors
      cv::Vec3f p1(prev_kps[i].x, prev_kps[i].y, 1.0f);

      // Rotate bearing vectors to current frame
      cv::Vec3f p2 = H * p1;

      // Project predicted bearing vectors to 2D again
      if (p2[2] > 0.0f) {
        predicted_kps.push_back(cv::Point2f(p2[0] / p2[2], p2[1] / p2[2]));
      } else {
        LOG(WARNING) << "Optical flow prediction failed for keypoint:\n"
                     << "- p1:  " << p1 << '\n'
                     << "- p2: " << p2;
        // Projection failed, keep old corner
        predicted_kps.push_back(prev_kps[i]);
      }
    }

    // Check that keypoints remain inside the image boundaries!

    *next_kps = predicted_kps;
    return true;
  }

 private:
  const cv::Matx33f K_;
  const cv::Matx33f K_inverse_;
};

}  // namespace VIO
