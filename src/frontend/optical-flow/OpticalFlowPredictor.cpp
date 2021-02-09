/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpticalFlowPredictor.cpp
 * @brief  Class that predicts optical flow between two images. This is
 * helpful for the tracker to have a good initial guess for feature tracking.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/optical-flow/OpticalFlowPredictor.h"

#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Rot3.h>

#include "kimera-vio/frontend/optical-flow/OpticalFlowVisualizer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

bool NoOpticalFlowPredictor::predictSparseFlow(
    const KeypointsCV& prev_kps,
    const gtsam::Rot3& /* inter_frame */,
    KeypointsCV* next_kps) {
  *CHECK_NOTNULL(next_kps) = prev_kps;
  return true;
}

RotationalOpticalFlowPredictor::RotationalOpticalFlowPredictor(
    const cv::Matx33f& K,
    const cv::Size& img_size)
    : K_(K),
      K_inverse_(K.inv()),
      img_size_(0.0f, 0.0f, img_size.width, img_size.height) {}

cv::Mat RotationalOpticalFlowPredictor::predictDenseFlow(
    const gtsam::Rot3& cam1_R_cam2) {
  // R is a relative rotation which takes a vector from the last frame to
  // the current frame.
  cv::Matx33f R = UtilsOpenCV::gtsamMatrix3ToCvMat(cam1_R_cam2.matrix());
  // Get bearing vector for kpt, rotate knowing frame to frame rotation,
  // get keypoints again
  cv::Matx33f H = K_ * R.t() * K_inverse_;

  // Visualize inferred optical flow
  OpticalFlowVisualizer optical_flow_viz;
  cv::Mat_<cv::Point2f> flow_img(img_size_.width, img_size_.height);
  for (int u = 0u; u < img_size_.width; u++) {
    for (int v = 0u; v < img_size_.height; v++) {
      cv::Vec3f p1(u, v, 1.0f);
      cv::Vec3f p2 = H * p1;
      cv::Vec3f flow = p2 - p1;
      flow_img.at<cv::Point2f>(v, u) = cv::Point2f(flow[0], flow[1]);
    }
  }
  cv::Mat color_flow = optical_flow_viz.drawOpticalFlow(flow_img);
  cv::Mat arrow_flow = optical_flow_viz.drawOpticalFlowArrows(flow_img);
  cv::imshow("Color Flow", color_flow);
  cv::imshow("Arrowed Flow", arrow_flow);
  cv::waitKey(1);
  return color_flow;
}

bool RotationalOpticalFlowPredictor::predictSparseFlow(
    const KeypointsCV& prev_kps,
    const gtsam::Rot3& cam1_R_cam2,
    KeypointsCV* next_kps) {
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

  // R is a relative rotation which takes a vector from the last frame to
  // the current frame.
  cv::Matx33f R = UtilsOpenCV::gtsamMatrix3ToCvMat(cam1_R_cam2.matrix());
  // Get bearing vector for kpt, rotate knowing frame to frame rotation,
  // get keypoints again
  cv::Matx33f H = K_ * R.t() * K_inverse_;

  // We use a new object in case next_kps is pointing to prev_kps!
  KeypointsCV predicted_kps;
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

}  // namespace VIO
