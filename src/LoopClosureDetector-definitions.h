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
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

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
  inline void setDescriptors(std::vector<std::vector<float>>& descriptors) {
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
  std::vector<std::vector<float>> descriptors_;

}; // class LCDFrame

class Match {
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
  // Computes a global representation for an image by using
  // the SURF feature descriptor in OpenCV and the bag of
  // words approach.
public:
  FramesDescriptor(const std::string& vocabulary_path) {
    std::cout << "Loading vocabulary from " << vocabulary_path << std::endl;
    vocab_.reset(new Surf64Vocabulary(vocabulary_path));
    std::cout << "Loaded vocabulary with " << vocab_->size() << " visual words."
              << std::endl;
  }

  Match computeMatches(double& threshold) {

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

  // Extracts SURF interest points and their descriptors
  void extractSurf(const cv::Mat& img, std::vector<cv::KeyPoint>& keys,
                    std::vector<std::vector<float>>& descriptors) {

    static cv::Ptr<cv::xfeatures2d::SURF> surf_detector =
        cv::xfeatures2d::SURF::create(400);

    surf_detector->setExtended(false);

    std::vector<float> plain;
    surf_detector->detectAndCompute(img, cv::Mat(), keys, plain);

    const int L = surf_detector->descriptorSize();
    descriptors.resize(plain.size() / L);

    unsigned int j = 0;
    for (unsigned int i = 0; i < plain.size(); i += L, ++j) {
      descriptors[j].resize(L);
      std::copy(plain.begin() + i, plain.begin() + i + L,
                descriptors[j].begin());
    }

  }

  // Transforms the feature descriptors to a BoW representation of whole image
  void describeFrame(LCDFrame& frame) {

    DBoW2::BowVector bow_vec;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<std::vector<float>> descriptors;
    cv::Mat img = frame.getImage();

    extractSurf(img, keypoints, descriptors);
    vocab_->transform(descriptors, bow_vec);

    frame.setDescriptors(descriptors);
    frame.setDbowVector(bow_vec);
  }

// private: // TODO: this
  std::unique_ptr<Surf64Vocabulary> vocab_;
  std::vector<LCDFrame> frames_;

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
