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
#include <unordered_map>

#include "datasource/ETH_parser.h"  // REMOVE THIS!!

namespace VIO {

class Logger {
 protected:
  Logger();
  virtual ~Logger();

  void openLogFile(const std::string& output_file_name,
                   bool open_file_in_append_mode = false);

  std::map<std::string, std::ofstream> filename_to_outstream_;

 private:
  void closeLogFiles();
  const std::string output_path_;
};

class BackendLogger : private Logger {
 public:
  BackendLogger();
  ~BackendLogger() = default;

  void logBackendResultsCSV(const VioBackEndOutputPayload& vio_output);
  void displayInitialStateVioInfo(const gtsam::Vector3& n_gravity_,
                                  const gtsam::Pose3& W_Pose_B_Lkf,
                                  const gtNavState& initialStateGT,
                                  const ImuAccGyrS& imu_accgyr,
                                  const Timestamp& timestamp_k) const;
  void logBackendResults(
      const ETHDatasetParser& dataset,
      const TrackerStatusSummary& tracker_status_summary,
      const gtsam::Pose3& relative_pose_body_mono,
      const Tracker& tracker,
      const gtsam::Pose3& relative_pose_body_stereo,
      const std::shared_ptr<VioBackEndOutputPayload>& vio_output,
      const double& horizon,
      const Timestamp& timestamp_lkf,
      const Timestamp& timestamp_k,
      const size_t& k);

 private:
  // Filenames to be saved in the output folder.
  const std::string output_poses_vio_filename_csv_ = "output_posesVIO.csv";
  const std::string output_landmarks_filename_ = "output_landmarks.txt";

  gtsam::Pose3 W_Pose_Bprevkf_vio_;
  double timing_loggerBackend_;
};

class FrontendLogger : private Logger {
 public:
  FrontendLogger();
  ~FrontendLogger() = default;

  void logFrontendResults(const TrackerStatusSummary& tracker_summary,
                          const size_t& nrKeypoints);

 private:
  // Filenames to be saved in the output folder.
  const std::string output_frontend_filename_ = "output_frontend.csv";
};

class VisualizerLogger : private Logger {
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
  const std::string output_mesh_filename_ = "output_mesh.ply";
  const std::string output_landmarks_filename_ = "output_landmarks.txt";
};

class PipelineLogger : private Logger {
 public:
  PipelineLogger();
  ~PipelineLogger() = default;

  void logPipelineOverallTiming(const std::chrono::milliseconds& duration);

 private:
  // Filenames to be saved in the output folder.
  const std::string output_pipeline_timing_ = "output_timingOverall.csv";
};

}  // namespace VIO
