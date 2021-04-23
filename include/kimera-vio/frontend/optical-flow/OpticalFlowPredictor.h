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
  virtual bool predictSparseFlow(const KeypointsCV& prev_kps,
                                 const gtsam::Rot3& cam1_R_cam2,
                                 KeypointsCV* next_kps) = 0;
  virtual cv::Mat predictDenseFlow(const gtsam::Rot3& cam1_R_cam2) = 0;
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

  bool predictSparseFlow(const KeypointsCV& prev_kps,
                         const gtsam::Rot3& /* inter_frame */,
                         KeypointsCV* next_kps) override;
  cv::Mat predictDenseFlow(const gtsam::Rot3& cam1_R_cam2) { return cv::Mat(); }
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
  RotationalOpticalFlowPredictor(const cv::Matx33f& K,
                                 const cv::Size& img_size);
  virtual ~RotationalOpticalFlowPredictor() = default;

  bool predictSparseFlow(const KeypointsCV& prev_kps,
                         const gtsam::Rot3& cam1_R_cam2,
                         KeypointsCV* next_kps) override;
  // NOT TESTED
  cv::Mat predictDenseFlow(const gtsam::Rot3& cam1_R_cam2) override;

 private:
  const cv::Matx33f K_;          // Intrinsic matrix of camera
  const cv::Matx33f K_inverse_;  // Cached inverse of K
  const cv::Rect2f img_size_;
};

}  // namespace VIO
