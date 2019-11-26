/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Logger.h
 * @brief  Logging output information.
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
// Open files with name output_filename, and checks that it is valid
static void OpenFile(const std::string& output_filename,
                     std::ofstream* output_file,
                     bool append_mode = false) {
  CHECK_NOTNULL(output_file);
  output_file->open(output_filename.c_str(),
                    append_mode ? std::ios_base::app : std::ios_base::out);
  output_file->precision(20);
  CHECK(output_file->is_open()) << "Cannot open file: " << output_filename;
  CHECK(output_file->good()) << "File in bad state: " << output_filename;
}

// Wrapper for std::ofstream to open/close it when created/destructed.
class OfstreamWrapper {
 public:
  OfstreamWrapper(const std::string& filename,
                  const bool& open_file_in_append_mode = false);
  ~OfstreamWrapper();
  void closeAndOpenLogFile();

 public:
  std::ofstream ofstream_;

 private:
  void openLogFile(const std::string& output_file_name,
                   bool open_file_in_append_mode = false);

 private:
  const std::string filename_;
  const std::string output_path_;
  const bool open_file_in_append_mode = false;
};

class BackendLogger {
 public:
  BackendLogger();
  ~BackendLogger() = default;

  void logBackendOutput(const BackendOutput& output);
  void displayInitialStateVioInfo(const gtsam::Vector3& n_gravity_,
                                  const gtsam::Pose3& W_Pose_B_Lkf,
                                  const VioNavState& initial_state_gt,
                                  const ImuAccGyrS& imu_accgyr,
                                  const Timestamp& timestamp_k) const;

 private:
  void logBackendResultsCSV(const BackendOutput& output);
  void logSmartFactorsStats(const BackendOutput& output);
  void logBackendPimNavstates(const BackendOutput& output);
  void logBackendFactorsStats(const BackendOutput& output);
  void logBackendTiming(const BackendOutput& output);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_poses_vio_csv_;
  OfstreamWrapper output_smart_factors_stats_csv_;
  OfstreamWrapper output_pim_navstates_csv_;
  OfstreamWrapper output_backend_factors_stats_csv_;
  OfstreamWrapper output_backend_timing_csv_;

  gtsam::Pose3 W_Pose_Bprevkf_vio_;
  double timing_loggerBackend_;
};

class FrontendLogger {
 public:
  FrontendLogger();
  ~FrontendLogger() = default;

  void logFrontendStats(const Timestamp& timestamp_lkf,
                        const DebugTrackerInfo& tracker_info,
                        const TrackerStatusSummary& tracker_summary,
                        const size_t& nrKeypoints);
  void logFrontendRansac(const Timestamp& timestamp_lkf,
                         const gtsam::Pose3& relative_pose_body_mono,
                         const gtsam::Pose3& relative_pose_body_stereo);

 private:
  // StreamWrappers with filenames to which output is saved.
  OfstreamWrapper output_frontend_stats_;
  OfstreamWrapper output_frontend_ransac_mono_;
  OfstreamWrapper output_frontend_ransac_stereo_;
};

class VisualizerLogger {
 public:
  VisualizerLogger();
  ~VisualizerLogger() = default;

  void logLandmarks(const PointsWithId& lmks);
  void logLandmarks(const cv::Mat& lmks);
  void logMesh(const cv::Mat& lmks,
               const cv::Mat& colors,
               const cv::Mat& mesh,
               const double& timestamp,
               bool log_accumulated_mesh = false);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_mesh_;
  OfstreamWrapper output_landmarks_;
};

class PipelineLogger {
 public:
  PipelineLogger();
  ~PipelineLogger() = default;

  void logPipelineOverallTiming(const std::chrono::milliseconds& duration);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_pipeline_timing_;
};

class LoopClosureDetectorLogger {
 public:
  LoopClosureDetectorLogger();
  ~LoopClosureDetectorLogger() = default;

  void logTimestampMap(
      const std::unordered_map<VIO::FrameId, VIO::Timestamp>& ts_map);
  void logLCDResult(const LcdOutput& lcd_output);
  void logLoopClosure(const LcdOutput& lcd_output);
  void logOptimizedTraj(const LcdOutput& lcd_output);
  void logDebugInfo(const LcdDebugInfo& debug_info);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_lcd_;
  OfstreamWrapper output_traj_;
  OfstreamWrapper output_status_;
  FrameIDTimestampMap ts_map_;
};

}  // namespace VIO
