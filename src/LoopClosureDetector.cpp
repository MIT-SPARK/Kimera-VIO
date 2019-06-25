/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.cpp
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial
 * Navigation. IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying
 * Perspective based on Smart Factors. In IEEE Intl. Conf. on Robotics and
 * Automation (ICRA), 2014.
 *
 * @author Marcus Abate
 */

#include "LoopClosureDetector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "LoggerMatlab.h"

namespace VIO {

bool LoopClosureDetector::spin(
          ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
          ThreadsafeQueue<LoopClosureDetectorOutputPayload>& output_queue,
          bool parallel_run) {
  LOG(INFO) << "Spinning LoopClosureDetector.";
  // TODO: Timing stuff??
  // utils::StatsCollector stat_pipeline_timing("Pipeline Overall Timing [ms]");
  // utils::StatsCollector stat_lcd_timing("LoopClosureDetector Timing [ms]");
  while (!shutdown_) {
    is_thread_working_ = false;
    std::shared_ptr<LoopClosureDetectorInputPayload> input = input_queue.popBlocking();
    is_thread_working_ = true;
    if (input) {
      auto tic = utils::Timer::tic();
      output_queue.push(spinOnce(input));
      auto spin_duration = utils::Timer::toc(tic).count();
      LOG(WARNING) << "Current LoopClosureDetector frequency: "
                   << 1000.0 / spin_duration << " Hz. ("
                   << spin_duration << " ms).";
      // stat_lcd_timing.AddSample(spin_duration);
    } else {
      LOG(WARNING) << "No LoopClosureDetector Input Payload received.";
    }

    if (!parallel_run) return true;
  }
  LOG(INFO) << "LoopClosureDetector successfully shutdown.";
  return true;
}

LoopClosureDetectorOutputPayload LoopClosureDetector::spinOnce(
    const std::shared_ptr<LoopClosureDetectorInputPayload>& input) {
  CHECK(input) << "No LoopClosureDetector Input Payload received.";

  // TODO: Store frame pose with this method so it can be recalled later
  // To do that we need to pull from the backend and grab poses there for the matching ID's
  // TODO: Frame.h says img_ is public but it probably shouldn't be.
  described_frames_.addAndDescribeFrame(input->stereo_frame_.getLeftFrame().img_, frame_count_,
                                        input->timestamp_kf_);
  Match match = described_frames_.computeMatches(match_threshold_);
  if (match.getID() != -1) {
    LOG(ERROR) << "\nLoopClosureDetector detected Loop Closure with score: "
              << match.getScore() << " at frame ID: " << match.getID() << "\n";
  }

  frame_count_ += 1;

  // TODO: get poses of the frames (need to talk to backend)
  gtsam::Pose3 pose_prev = input->stereo_frame_.getBPoseCamLRect();
  gtsam::Pose3 pose_curr = input->stereo_frame_.getBPoseCamLRect();

  LoopClosureDetectorOutputPayload output_payload(input->timestamp_kf_,
                                                  match.getID(),
                                                  match.getOwnID(),
                                                  pose_prev,
                                                  pose_curr);

  // TODO: add LoggerMatlab options
  // if (log_output_) {
  //   LoggerMatlab logger;
  //   logger.openLogFiles(13, "", true);
  //   logger.logLoopClosureDetectorResultsCSV(output_payload);
  //   logger.closeLogFiles(13);
  // }

  return output_payload;
}

} // namespace VIO
