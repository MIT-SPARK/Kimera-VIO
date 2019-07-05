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

  void print() const;

private:
  void initLoopDetector();

  // TODO: better name for processImage
  DLoopDetector::DetectionResult processImage(const cv::Mat& img);

  void extractOrb(const cv::Mat& img,
                  std::vector<cv::KeyPoint>& keypoints,
                  std::vector<cv::Mat>& descriptors);

  LoopClosureDetectorOutputPayload processResult(
      const DLoopDetector::DetectionResult& loop_result,
      const Timestamp& timestamp_kf);

private:
  // Parameter members.
  LoopClosureDetectorParams lcd_params_;
  const bool log_output_ = {false};

  // Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};

  // ORB extraction and loop detection members.
  OrbVocabulary vocab_;
  OrbLoopDetector loop_detector_;
  cv::Ptr<cv::ORB> orb_feature_detector_;

  // Pose recovery members.
  bool set_intrinsics_ = {false};

}; // class LoopClosureDetector

} // namespace VIO
#endif /* LoopClosureDetector_H_ */
