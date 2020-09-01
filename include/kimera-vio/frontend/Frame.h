/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Frame.h
 * @brief  Class describing a single image
 * @author Luca Carlone
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <cstdlib>
#include <numeric>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

////////////////////////////////////////////////////////////////////////////
// Class for storing/processing a single image
class Frame : public PipelinePayload {
 public:
  // TODO(Toni): do it please.
  // KIMERA_DELETE_COPY_CONSTRUCTORS(Frame);
  KIMERA_POINTER_TYPEDEFS(Frame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructors.
  /// @param img: does a shallow copy of the image by defaults,
  ///  if Frame should have ownership of the image, clone it.
  Frame(const FrameId& id,
        const Timestamp& timestamp,
        const CameraParams& cam_param,
        const cv::Mat& img)
      : PipelinePayload(timestamp),
        id_(id),
        cam_param_(cam_param),
        img_(img),
        isKeyframe_(false) {}

  // TODO(TONI): delete all copy constructors!!
  // Look at the waste of time this is :O
  Frame(const Frame& frame)
      : PipelinePayload(frame.timestamp_),
        id_(frame.id_),
        cam_param_(frame.cam_param_),
        img_(frame.img_),
        isKeyframe_(frame.isKeyframe_),
        keypoints_(frame.keypoints_),
        scores_(frame.scores_),
        landmarks_(frame.landmarks_),
        landmarks_age_(frame.landmarks_age_),
        versors_(frame.versors_),
        descriptors_(frame.descriptors_) {}

 public:
  /* ------------------------------------------------------------------------ */
  size_t getNrValidKeypoints() const {
    // TODO: can we cache this value?
    size_t count = 0;
    for (size_t i = 0; i < landmarks_.size(); i++) {
      if (landmarks_.at(i) != -1)  // It is valid.
        count += 1;
    }
    return count;
  }

  /* ------------------------------------------------------------------------ */
  KeypointsCV getValidKeypoints() const {
    // TODO: can we cache this result?
    KeypointsCV validKeypoints;
    CHECK_EQ(landmarks_.size(), keypoints_.size());
    for (size_t i = 0; i < landmarks_.size(); i++) {
      if (landmarks_.at(i) != -1) {  // It is valid.
        validKeypoints.push_back(keypoints_.at(i));
      }
    }
    return validKeypoints;
  }

  /* ------------------------------------------------------------------------ */
  static LandmarkId findLmkIdFromPixel(
      const KeypointCV& px,
      const KeypointsCV& keypoints,
      const LandmarkIds& landmarks,
      size_t* idx_in_keypoints = nullptr) {
    // Iterate over all current keypoints_.
    for (size_t i = 0; i < keypoints.size(); i++) {
      // If we have found the pixel px in the set of keypoints, return the
      // landmark id of the keypoint and return the index of it at keypoints_.
      const KeypointCV& keypoint = keypoints.at(i);
      if (keypoint.x == px.x && keypoint.y == px.y) {
        if (idx_in_keypoints) {
          *idx_in_keypoints = i;  // Return index.
        }
        return landmarks.at(i);
      }
    }
    // We did not find the keypoint.
    return -1;
  }

  /* ------------------------------------------------------------------------ */
  void print() const {
    LOG(INFO) << "Frame id: " << id_ << " at timestamp: " << timestamp_ << "\n"
              << "isKeyframe_: " << isKeyframe_ << "\n"
              << "nr keypoints_: " << keypoints_.size() << "\n"
              << "nr valid keypoints_: " << getNrValidKeypoints() << "\n"
              << "nr landmarks_: " << landmarks_.size() << "\n"
              << "nr versors_: " << versors_.size() << "\n"
              << "size descriptors_: " << descriptors_.size();
    cam_param_.print();
  }

  /* ------------------------------------------------------------------------ */
  static Vector3 calibratePixel(const KeypointCV& cv_px,
                                const CameraParams& cam_param) {
    // Calibrate pixel.
    // matrix of px with a single entry, i.e., a single pixel
    cv::Mat_<KeypointCV> uncalibrated_px(1, 1);
    uncalibrated_px(0) = cv_px;

    cv::Mat calibrated_px;
    if (cam_param.distortion_model_ == DistortionModel::RADTAN) {
      // TODO optimize this in just one call, the s in Points is there for
      // something I hope.
      cv::undistortPoints(uncalibrated_px,
                          calibrated_px,
                          cam_param.K_,
                          cam_param.distortion_coeff_mat_);
    } else if (cam_param.distortion_model_ == DistortionModel::EQUIDISTANT) {
      // TODO: Create unit test for fisheye / equidistant model
      cv::fisheye::undistortPoints(uncalibrated_px,
                                   calibrated_px,
                                   cam_param.K_,
                                   cam_param.distortion_coeff_mat_);
    } else {
      LOG(ERROR) << "Unknown distortion model.";
    }

    // Transform to unit vector.
    Vector3 versor(
        calibrated_px.at<float>(0, 0), calibrated_px.at<float>(0, 1), 1.0);

    // sanity check, try to distort point using gtsam and make sure you get
    // original pixel
    // gtsam::Point2 uncalibrated_px_gtsam =
    // cam_param.calibration_.uncalibrate(gtsam::Point2(versor(0),versor(1)));
    // gtsam::Point2 uncalibrated_px_opencv =
    // gtsam::Point2(uncalibrated_px.at<float>(0,0),uncalibrated_px.at<float>(0,1));
    // gtsam::Point2 px_mismatch  = uncalibrated_px_opencv -
    // uncalibrated_px_gtsam;
    //
    // if(px_mismatch.vector().norm() > 1){
    //  std::cout << "uncalibrated_px: \n" << uncalibrated_px << std::endl;
    //  std::cout << "uncalibrated_px_gtsam: \n" << uncalibrated_px_gtsam <<
    //  std::endl; std::cout << "px_mismatch: \n" << px_mismatch << std::endl;
    //  throw std::runtime_error("CalibratePixel: possible calibration
    //  mismatch");
    //}

    // Return unit norm vector
    return versor.normalized();
  }

 public:
  const FrameId id_;

  // These are non-const since they will be changed during rectification.
  // TODO(Toni): keep original and rectified params.
  CameraParams cam_param_;

  // Actual image stored by the class frame.
  // This must be const otw, we have to reimplement the copy ctor to allow
  // for deep copies.
  const cv::Mat img_;

  // Results of image processing.
  bool isKeyframe_ = false;

  // These containers must have same size.
  KeypointsCV keypoints_;
  std::vector<double> scores_;  // quality of extracted keypoints
  LandmarkIds landmarks_;
  //! How many consecutive *keyframes* saw the keypoint
  std::vector<size_t> landmarks_age_;
  //! in the ref frame of the UNRECTIFIED left frame
  BearingVectors versors_;
  //! Not currently used
  cv::Mat descriptors_;
};

}  // namespace VIO
