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
        isKeyframe_(false),
        keypoints_(),
        keypoints_undistorted_(),
        scores_(),
        landmarks_(),
        landmarks_age_(),
        versors_(),
        descriptors_() {}

  // TODO(TONI): delete all copy constructors!!
  // Look at the waste of time this is :O
  Frame(const Frame& frame)
      : PipelinePayload(frame.timestamp_),
        id_(frame.id_),
        cam_param_(frame.cam_param_),
        img_(frame.img_),
        isKeyframe_(frame.isKeyframe_),
        keypoints_(frame.keypoints_),
        keypoints_undistorted_(frame.keypoints_undistorted_),
        scores_(frame.scores_),
        landmarks_(frame.landmarks_),
        landmarks_age_(frame.landmarks_age_),
        versors_(frame.versors_),
        descriptors_(frame.descriptors_) {}

 public:
  /* ------------------------------------------------------------------------ */
  void checkFrame() const {
    const size_t nr_kpts = keypoints_.size();
    CHECK_EQ(keypoints_undistorted_.size(), nr_kpts);
    CHECK_EQ(scores_.size(), nr_kpts);
    CHECK_EQ(landmarks_.size(), nr_kpts);
    CHECK_EQ(landmarks_age_.size(), nr_kpts);
    CHECK_EQ(versors_.size(), nr_kpts);
    // CHECK_EQ(descriptors_.size(), nr_kpts);
  }
  size_t getNrValidKeypoints() const {
    // TODO: can we cache this value?
    size_t count = 0u;
    for (size_t i = 0u; i < landmarks_.size(); i++) {
      if (landmarks_.at(i) != -1)  // It is valid.
        count += 1;
    }
    return count;
  }

  /* ------------------------------------------------------------------------ */
  KeypointsCV getValidKeypoints() const {
    // TODO: can we cache this result?
    KeypointsCV valid_keypoints;
    CHECK_EQ(landmarks_.size(), keypoints_.size());
    for (size_t i = 0u; i < landmarks_.size(); i++) {
      if (landmarks_.at(i) != -1) {  // It is valid.
        valid_keypoints.push_back(keypoints_.at(i));
      }
    }
    return valid_keypoints;
  }

  /* ------------------------------------------------------------------------ */
  static LandmarkId findLmkIdFromPixel(const KeypointCV& px,
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

  // get a much smaller (and faster) copy of a frame for frame-to-frame RANSAC
  Frame::UniquePtr getRansacFrame() const {
    return Frame::UniquePtr(new Frame(
        id_, timestamp_, cam_param_, keypoints_, landmarks_, versors_));
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
  StatusKeypointsCV keypoints_undistorted_;
  std::vector<double> scores_;  // quality of extracted keypoints
  LandmarkIds landmarks_;
  //! How many consecutive *keyframes* saw the keypoint
  std::vector<size_t> landmarks_age_;
  //! in the ref frame of the RECTIFIED left frame
  BearingVectors versors_;
  //! Not currently used
  cv::Mat descriptors_;
  //! Optional mask for feature detection. Note that can change when the frame is const
  mutable cv::Mat detection_mask_;

 protected:
  Frame(const FrameId& id,
        const Timestamp& timestamp,
        const CameraParams& params,
        const KeypointsCV& keypoints,
        const LandmarkIds& landmarks,
        const BearingVectors& versors)
      : PipelinePayload(timestamp),
        id_(id),
        cam_param_(params),
        keypoints_(keypoints),
        landmarks_(landmarks),
        versors_(versors) {}
};

}  // namespace VIO
