/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector-definitions.h
 * @brief  Definitions for LoopClosureDetector
 * @author Marcus Abate
 */

 #pragma once

#include "common/vio_types.h"

#include <DBoW2/DBoW2.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>

namespace VIO {

using ID = std::int64_t; // TODO: use one of the common/vio_types.h options?

class LCDFrame {
public:
  LCDFrame(cv::Mat image, ID id, Timestamp timestamp):
    image_(image),
    id_(id),
    timestamp_(timestamp) {};

  inline ID getID() { return id_; };
  inline cv::Mat getImage() { return image_; };
  inline DBoW2::BowVector getDbowVector() { return dbow_vector_; }
  inline void setDescriptors(std::vector<cv::Mat>& descriptors) {
    descriptors_ = descriptors;
  }
  inline void setDbowVector(DBoW2::BowVector& bow_vec) {
    dbow_vector_ = bow_vec;
  }

private:
  cv::Mat image_;
  ID id_;
  Timestamp timestamp_;
  DBoW2::BowVector dbow_vector_;
  std::vector<cv::KeyPoint> keypoints_;
  std::vector<cv::Mat> descriptors_;

}; // class LCDFrame

struct Match {
public:
  Match(ID own_id, ID id, double score) :
    own_id_(own_id), id_(id), score_(score) {}
  inline ID getID() { return id_; }
  inline ID getOwnID() { return own_id_; }
  inline double getScore() { return score_; }

private:
  ID own_id_;
  ID id_;
  double score_;
}; // class Match


class FramesDescriptor {
public:
  FramesDescriptor(const std::string& vocabulary_path,
                   const int nfeatures, const float scaleFactor,
                   const int nlevels, const int edgeThreshold,
                   const int firstLevel, const int WTA_K, const int scoreType,
                   const int patchSize, const int fastThreshold) {
    // TODO: test performance with a new orb_detector_ for every extractOrb
    orb_detector_ = cv::ORB::create(nfeatures, scaleFactor, nlevels,
                                    edgeThreshold, firstLevel, WTA_K, scoreType,
                                    patchSize, fastThreshold);
    std::cout << "Loading vocabulary from " << vocabulary_path << std::endl;
    vocab_.reset(new OrbVocabulary()); // TODO: not super nice, would like to just go vocab_.reset(new OrbVocabulary(vocabulary_path));
    // TODO: add support for loading compressed files (should be standard)
    vocab_->loadFromTextFile(vocabulary_path);
    std::cout << "Loaded vocabulary with " << vocab_->size() << " visual words."
              << std::endl;
  }

  Match computeMatches(const double& threshold) {
    ID match_id = -1;
    double final_score = 0.0;
    double curr_score = 0.0;

    LCDFrame last_frame = frames_.back();

    for (size_t i; i<(frames_.size()-1); i++) {
      LCDFrame iter_frame = frames_.at(i);
      curr_score = vocab_->score(last_frame.getDbowVector(),
                                 iter_frame.getDbowVector());
      if (curr_score > threshold && curr_score > final_score) {
        final_score = curr_score;
        match_id = iter_frame.getID();
      }
    }
    Match match(last_frame.getID(), match_id, final_score);
    return match;
  }

  void addAndDescribeFrame(cv::Mat image, ID id, Timestamp timestamp) {
    LCDFrame frame(image, id, timestamp);
    describeFrame(frame);
    frames_.push_back(frame);
  }

  // Extracts ORB interest points and their descriptors
  void extractOrb(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,
                    std::vector<cv::Mat>& descriptors) {
    cv::Mat plain;
    orb_detector_->detectAndCompute(img, cv::Mat(), keypoints, plain);

    int L = orb_detector_->descriptorSize();
    descriptors.resize(plain.size().height);

    for (unsigned int i = 0; i < descriptors.size(); i++) {
      descriptors[i] = cv::Mat(1, L, plain.type()); // one row only
      plain.row(i).copyTo(descriptors[i].row(0));
    }
  }

  // Transforms the feature descriptors to a BoW representation of whole image
  void describeFrame(LCDFrame& frame) {

    cv::Mat img = frame.getImage();
    DBoW2::BowVector bow_vec;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> descriptors;

    // clock_t begin = clock();
    extractOrb(img, keypoints, descriptors);
    // clock_t end = clock();
    // double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000.0;
    // std::cout << "extractOrb took " << elapsed_ms << " ms." << std::endl;

    vocab_->transform(descriptors, bow_vec);
    frame.setDescriptors(descriptors);
    frame.setDbowVector(bow_vec);
  }

private: // TODO: this
  std::unique_ptr<OrbVocabulary> vocab_;
  std::vector<LCDFrame> frames_;
  cv::Ptr<cv::ORB> orb_detector_;
}; // class FramesDescriptor

struct LoopClosureDetectorInputPayload {
  LoopClosureDetectorInputPayload(const Timestamp& timestamp_kf,
                                  const StereoFrame stereo_frame)
    : timestamp_kf_(timestamp_kf),
      stereo_frame_(stereo_frame) {}

  const Timestamp timestamp_kf_;
  const StereoFrame stereo_frame_;

}; // struct LoopClosureInputPayload

struct LoopClosureDetectorOutputPayload {
  LoopClosureDetectorOutputPayload(const Timestamp& timestamp_kf,
                                   const FrameId& id_prev,
                                   const FrameId& id_curr,
                                   const gtsam::Pose3& W_Pose_Bprev,
                                   const gtsam::Pose3& W_Pose_Bcurr)
    : timestamp_kf_(timestamp_kf),
      id_prev_(id_prev),
      id_curr_(id_curr),
      W_Pose_Bprev_(W_Pose_Bprev),
      W_Pose_Bcurr_(W_Pose_Bcurr) {}

  const Timestamp timestamp_kf_;
  const FrameId id_prev_;
  const FrameId id_curr_;
  const gtsam::Pose3 W_Pose_Bprev_;
  const gtsam::Pose3 W_Pose_Bcurr_;
}; // struct LoopClosureOutputPayload
} // namespace VIO
