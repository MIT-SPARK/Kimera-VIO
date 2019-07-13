/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.h
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial Navigation.
 * IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying Perspective based on Smart Factors.
 * In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.
 *
 * @author Marcus Abate
 */

#ifndef LoopClosureDetector_H_
#define LoopClosureDetector_H_

#include <string>

#include <boost/shared_ptr.hpp> // used for opengv
#include "Tracker-definitions.h" // TODO we don't need all of it...
// #include <opengv/sac/Ransac.hpp>
// #include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
// #include <opengv/point_cloud/PointCloudAdapter.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include "UtilsOpenCV.h"

#include <DBoW2/DBoW2.h>
#include <DLoopDetector/DLoopDetector.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>

#include "utils/ThreadsafeQueue.h"
#include "utils/Statistics.h"
#include "utils/Timer.h"
#include "StereoFrame.h"
#include "LoopClosureDetectorParams.h"
#include "LoopClosureDetector-definitions.h"

namespace VIO {

class LoopClosureDetector {
public:
  LoopClosureDetector(const LoopClosureDetectorParams& lcd_params,
                      const bool log_output=false);

  virtual ~LoopClosureDetector() {
    LOG(INFO) << "LoopClosureDetector desctuctor called.";
  }

  bool spin(ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
            ThreadsafeQueue<LoopClosureDetectorOutputPayload>& output_queue,
            bool parallel_run = true);

  LoopClosureDetectorOutputPayload spinOnce(
      const std::shared_ptr<LoopClosureDetectorInputPayload>& input);

  inline void shutdown() {
    LOG_IF(WARNING, shutdown_) << "Shutdown requested, but LoopClosureDetector "
                                  "was already shutdown.";
    LOG(INFO) << "Shutting down LoopClosureDetector.";
    shutdown_ = true;
  }

  inline void restart() {
    LOG(INFO) << "Resetting shutdown LoopClosureDetector flag to false.";
    shutdown_ = false;
  }

  inline bool isWorking() const { return is_thread_working_; }

  inline LoopClosureDetectorParams& getLCDParams() { return lcd_params_; }

  inline LoopClosureDetectorParams* getLCDParamsMutable() {
    return &lcd_params_;
  }

  inline OrbVocabulary& getVocab() { return vocab_; }

  inline OrbLoopDetector& getLoopDetector() { return loop_detector_; }

  inline cv::Ptr<cv::ORB> getFeatureDetector() {
    return orb_feature_detector_;
  }

  inline bool getIntrinsicsFlag() const { return set_intrinsics_; }

  inline std::vector<cv::KeyPoint> getKeypointsAt(unsigned int id) const {
    return db_keypoints_[id];
  }

  inline cv::Mat getDescriptorsAt(unsigned int id) const {
    return db_descriptors_[id];
  }

  inline std::vector<gtsam::Vector3> get3DKeypointsAt(unsigned int id) const {
    return db_keypoints_3d_[id];
  }

  void print() const;

  // TODO: utils and reorder
  void computeAndDrawMatchesBetweenFrames(const cv::Mat& query_img,
    const cv::Mat& match_img, const size_t& query_id, const size_t& match_id);

// private: // TODO: make these private but fix testing to work for them
  void initLoopDetector();

  LoopClosureDetectorOutputPayload checkLoopClosure(StereoFrame& stereo_frame);

  DLoopDetector::DetectionResult processAndAddFrame(StereoFrame& stereo_frame);

  void rewriteStereoFrameFeatures(StereoFrame& stereo_frame,
    const std::vector<cv::KeyPoint>& keypoints);

  // TODO: This one definitely should be private
  gtsam::Pose3 computePoseStereoRansac(size_t query, size_t match);

  gtsam::Pose3 computePoseStereoNonlinearOpt(size_t query, size_t match);

  gtsam::Pose3 computePoseMonoRansac(size_t query, size_t match);

  gtsam::Pose3 computePoseStereoGiven2D(size_t query, size_t match,
      gtsam::Pose3& pose_2d);

  static gtsam::Pose3 mat2pose(const cv::Mat& R, const cv::Mat& t);

private:
  // Parameter members.
  LoopClosureDetectorParams lcd_params_;
  const bool log_output_ = {false};

  // Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};

  // Keypoint and descriptor database for RANSAC 3D.
  // TODO: loop_detector_ saves these things as well, it's a waste to save both. Refactor so that you aren't dependent on DLoopDetector
  std::vector<std::vector<cv::KeyPoint>> db_keypoints_;
  std::vector<cv::Mat> db_descriptors_; // TODO: make sure this is the best way of handling the need for MAT for matcher descriptor.
  std::vector<std::vector<gtsam::Vector3>> db_keypoints_3d_;

  // ORB extraction, matching and loop detection members.
  OrbVocabulary vocab_;
  OrbLoopDetector loop_detector_;
  cv::Ptr<cv::ORB> orb_feature_detector_;
  cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;

  // Pose recovery members.
  bool set_intrinsics_ = {false};

}; // class LoopClosureDetector

} // namespace VIO
#endif /* LoopClosureDetector_H_ */
