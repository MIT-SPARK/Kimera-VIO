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

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/loopclosure/LcdOutputPacket.h"
#include "kimera-vio/mesh/Mesh.h"

namespace VIO {

/* -------------------------------------------------------------------------- */
// Open files with name output_filename, and checks that it is valid
static inline void OpenFile(const std::string& output_filename,
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
  KIMERA_POINTER_TYPEDEFS(OfstreamWrapper);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OfstreamWrapper);
  OfstreamWrapper(const std::string& filename,
                  const bool& open_file_in_append_mode = false);
  virtual ~OfstreamWrapper();
  void closeAndOpenLogFile();

 public:
  std::ofstream ofstream_;
  const std::string filename_;
  const std::string output_path_;
  const bool open_file_in_append_mode = false;

 protected:
  void openLogFile(const std::string& output_file_name,
                   bool open_file_in_append_mode = false);
};

/**
 * @brief Logs ground-truth info from Euroc dataset, just copy-paste
 * ground-truth csv file.
 */
class EurocGtLogger {
 public:
  KIMERA_POINTER_TYPEDEFS(EurocGtLogger);
  KIMERA_DELETE_COPY_CONSTRUCTORS(EurocGtLogger);
  EurocGtLogger();
  virtual ~EurocGtLogger() = default;

  /**
   * @brief logGtData Simply copy-pastes the file given as input to the
   * file output_gt_poses.csv inside the output folder.
   * @param gt_file file where the data.csv file in Euroc is located.
   * Usually inside `mav0/state_groundtruth_estimate0/data.csv`.
   */
  void logGtData(const std::string& gt_file);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_gt_poses_csv_;
};

class BackendLogger {
 public:
  KIMERA_POINTER_TYPEDEFS(BackendLogger);
  KIMERA_DELETE_COPY_CONSTRUCTORS(BackendLogger);
  BackendLogger();
  virtual ~BackendLogger() = default;

  void logBackendOutput(const BackendOutput& output);
  void displayInitialStateVioInfo(const gtsam::Vector3& n_gravity_,
                                  const gtsam::Pose3& W_Pose_B_Lkf,
                                  const VioNavState& initial_state_gt,
                                  const ImuAccGyrS& imu_accgyr,
                                  const Timestamp& timestamp_k) const;

  void logBackendExtOdom(const BackendInput& input);

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
  OfstreamWrapper output_backend_external_odometry_;

  gtsam::Pose3 W_Pose_Bprevkf_vio_;
  double timing_loggerbackend_;
  bool is_header_written_poses_vio_ = false;
  bool is_header_written_smart_factors_ = false;
  bool is_header_written_pim_navstates_ = false;
  bool is_header_written_backend_factors_stats_ = false;
  bool is_header_written_backend_timing_ = false;
  bool is_header_written_external_odometry_ = false;
};

class FrontendLogger {
 public:
  KIMERA_POINTER_TYPEDEFS(FrontendLogger);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FrontendLogger);
  FrontendLogger();
  virtual ~FrontendLogger() = default;

  void logFrontendStats(const Timestamp& timestamp_lkf,
                        const DebugTrackerInfo& tracker_info,
                        const TrackerStatusSummary& tracker_summary,
                        const size_t& nrKeypoints);
  void logFrontendRansac(const Timestamp& timestamp_lkf,
                         const gtsam::Pose3& relative_pose_body_mono,
                         const gtsam::Pose3& relative_pose_body_stereo);
  void logFrontendImg(const FrameId& kf_id,
                      const cv::Mat& img,
                      const std::string& img_name_prepend,
                      const std::string& dir_name,
                      bool disp_img,
                      bool save_img);
  void logFrontendTemporalCal(const Timestamp& timestamp_vision,
                              const Timestamp& timestamp_imu,
                              const double& vision_relative_angle_norm,
                              const double& image_relative_angle_norm,
                              bool not_enough_data,
                              bool not_enough_variance,
                              const double& result);

 private:
  // StreamWrappers with filenames to which output is saved.
  OfstreamWrapper output_frontend_stats_;
  OfstreamWrapper output_frontend_ransac_mono_;
  OfstreamWrapper output_frontend_ransac_stereo_;
  OfstreamWrapper output_frontend_temporal_cal_;
  std::string output_frontend_img_path_;
  bool is_header_written_frontend_stats_ = false;
  bool is_header_written_ransac_mono_ = false;
  bool is_header_written_ransac_stereo_ = false;
  bool is_header_written_temporal_cal_ = false;
};

class MesherLogger {
 public:
  KIMERA_POINTER_TYPEDEFS(MesherLogger);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MesherLogger);
  MesherLogger();
  virtual ~MesherLogger() = default;

  /**
   * @brief serializeMesh logs the mesh into a file that can be later read.
   * @param mesh Mesh to be serialized to file (this should be const, but the
   * serialization function needs to be non-const to be able to deserialize).
   */
  template <typename T>
  void serializeMesh(Mesh<T>& mesh, const std::string& filename) {
    mesh.save(output_path_ + '/' + filename);
  }

  /**
   * @brief deserializeMesh reads the serialized mesh from a file.
   * @param filename File where the mesh was serialized
   * @param mesh Mesh where to store deserialized data
   */
  template <typename T>
  void deserializeMesh(const std::string& filename, Mesh<T>* mesh) const {
    CHECK_NOTNULL(mesh);
    mesh->load(output_path_ + '/' + filename);
  }

 protected:
  std::string output_path_;
  bool is_header_written_ = false;
};

class VisualizerLogger {
 public:
  KIMERA_POINTER_TYPEDEFS(VisualizerLogger);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisualizerLogger);
  VisualizerLogger();
  virtual ~VisualizerLogger() = default;

  void logLandmarks(const PointsWithId& lmks);
  void logLandmarks(const cv::Mat& lmks);
  /**
   * @brief logMesh as a ply file
   * @param lmks Landmarks (Vertices of the mesh)
   * @param colors Colors of the vertices
   * @param polygons_mesh  Mesh polygons
   * @param timestamp the mesh timestamp
   * @param log_accumulated_mesh whether to append the mesh vertices/faces
   */
  void logMesh(const cv::Mat& lmks,
               const cv::Mat& colors,
               const cv::Mat& polygons_mesh,
               const double& timestamp,
               bool log_accumulated_mesh = false);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_mesh_;
  OfstreamWrapper output_landmarks_;
  bool is_header_written_mesh_ = false;
};

class PipelineLogger {
 public:
  KIMERA_POINTER_TYPEDEFS(PipelineLogger);
  KIMERA_DELETE_COPY_CONSTRUCTORS(PipelineLogger);
  PipelineLogger();
  virtual ~PipelineLogger() = default;

  void logPipelineOverallTiming(const std::chrono::milliseconds& duration);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_pipeline_timing_;
};

class LoopClosureDetectorLogger {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureDetectorLogger);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LoopClosureDetectorLogger);
  LoopClosureDetectorLogger();
  virtual ~LoopClosureDetectorLogger() = default;

  void logTimestampMap(
      const std::unordered_map<VIO::FrameId, VIO::Timestamp>& ts_map);
  void logLCDResult(const LcdOutput& lcd_output);
  void logLoopClosure(const LcdOutput& lcd_output);
  void logGeometricVerification(const Timestamp& timestamp_query,
                                const Timestamp& timestamp_match,
                                const gtsam::Pose3& camRef_Pose_camCur);
  void logPoseRecovery(const Timestamp& timestamp_query,
                       const Timestamp& timestamp_match,
                       const gtsam::Pose3& bodyRef_Pose_bodyCur);
  void logOptimizedTraj(const LcdOutput& lcd_output);
  void logDebugInfo(const LcdDebugInfo& debug_info);

 private:
  // Filenames to be saved in the output folder.
  OfstreamWrapper output_lcd_;
  OfstreamWrapper output_traj_;
  OfstreamWrapper output_status_;
  OfstreamWrapper output_geom_verif_;
  OfstreamWrapper output_pose_recovery_;
  FrameIDTimestampMap ts_map_;
  bool is_header_written_lcd_ = false;
  bool is_header_written_status_ = false;
  bool is_header_written_geom_verif_ = false;
  bool is_header_written_pose_recovery_ = false;
};

}  // namespace VIO
