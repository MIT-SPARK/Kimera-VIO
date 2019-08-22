/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoggerMatlab.cpp
 * @brief  Logging output information.
 * @author Antoni Rosinol, Luca Carlone
 */

#include "logging/Logger.h"

#include <boost/foreach.hpp>
#include <memory>

#include "StereoVisionFrontEnd-definitions.h"
#include "UtilsOpenCV.h"
#include "utils/Statistics.h"
#include "utils/Timer.h"

DEFINE_string(output_path, "./", "Path where to store VIO's log output.");

namespace VIO {

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
Logger::Logger() : output_path_(FLAGS_output_path) {}

Logger::~Logger() { closeLogFiles(); }

void Logger::openLogFile(const std::string& output_file_name,
                         bool open_file_in_append_mode) {
  CHECK(!output_file_name.empty());
  CHECK(filename_to_outstream_.find(output_file_name) ==
        filename_to_outstream_.end())
      << "Using an already existing filename.";
  UtilsOpenCV::OpenFile(output_path_ + '/' + output_file_name,
                        &filename_to_outstream_[output_file_name],
                        open_file_in_append_mode);
}

void Logger::closeLogFiles() {
  for (auto& x : filename_to_outstream_) {
    VLOG(1) << "Closing output file: " << x.first.c_str();
    x.second.close();
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
BackendLogger::BackendLogger() : Logger() {
  openLogFile(output_landmarks_filename_, true);
  openLogFile(output_poses_vio_filename_csv_, true);
}

void BackendLogger::displayInitialStateVioInfo(
    const gtsam::Vector3& n_gravity_,
    const gtsam::Pose3& W_Pose_B_Lkf,
    const gtNavState& initialStateGT,
    const ImuAccGyrS& imu_accgyr,
    const Timestamp& timestamp_k) const {
  initialStateGT.print("initialStateGT\n");
  const gtsam::Vector3& rpy_gt =
      initialStateGT.pose_.rotation().rpy();  // such that R = Rot3::Ypr(y,p,r)
  LOG(INFO) << "yaw= " << rpy_gt(2) << ' ' << "pitch= " << rpy_gt(1) << ' '
            << "roll= " << rpy_gt(0);
  gtsam::Vector3 localGravity =
      initialStateGT.pose_.rotation().inverse().matrix() * n_gravity_;
  LOG(INFO) << "gravity in global frame: \n"
            << n_gravity_ << '\n'
            << "gravity in local frame: \n"
            << localGravity << '\n'
            << "expected initial acc measurement (no bias correction): \n"
            << -localGravity << '\n'
            << "expected initial acc measurement: \n"
            << -localGravity + initialStateGT.imu_bias_.accelerometer() << '\n'
            << "actual initial acc measurement: \n"
            << imu_accgyr.block<3, 1>(0, 0) << '\n'
            << "expected initial gyro measurement: \n"
            << initialStateGT.imu_bias_.gyroscope() << '\n'
            << "actual initial gyro measurement: \n"
            << imu_accgyr.block<3, 1>(3, 0);

  double vioRotError, vioTranError;
  std::tie(vioRotError, vioTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(initialStateGT.pose_,
                                                       W_Pose_B_Lkf);
  CHECK_LE(vioRotError, 1e-4);
  CHECK_LE(vioTranError, 1e-4)
      << "stereoVIOExample: wrong initialization (ground truth initialization)";
}

void BackendLogger::logBackendResultsCSV(
    const VioBackEndOutputPayload& vio_output) {
  // We log the poses in csv format for later alignement and analysis.
  static bool is_header_written = false;
  std::ofstream& output_stream =
      filename_to_outstream_.at(output_poses_vio_filename_csv_);
  if (!is_header_written) {
    output_stream << "timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz,"
                     " bgx, bgy, bgz, bax, bay, baz"
                  << std::endl;
    is_header_written = true;
  }
  const auto& w_pose_blkf_trans =
      vio_output.W_Pose_Blkf_.translation().transpose();
  const auto& w_pose_blkf_rot = vio_output.W_Pose_Blkf_.rotation().quaternion();
  const auto& w_vel_blkf = vio_output.W_Vel_Blkf_.transpose();
  const auto& imu_bias_gyro = vio_output.imu_bias_lkf_.gyroscope().transpose();
  const auto& imu_bias_acc =
      vio_output.imu_bias_lkf_.accelerometer().transpose();
  output_stream << vio_output.timestamp_kf_ << ", " << w_pose_blkf_trans.x()
                << ", " << w_pose_blkf_trans.y() << ", "
                << w_pose_blkf_trans.z() << ", " << w_pose_blkf_rot(1)
                << ", "                        // q_x
                << w_pose_blkf_rot(2) << ", "  // q_y
                << w_pose_blkf_rot(3) << ", "  // q_z
                << w_pose_blkf_rot(0) << ", "  // q_w
                << w_vel_blkf(0) << ", " << w_vel_blkf(1) << ", "
                << w_vel_blkf(2) << ", " << imu_bias_gyro(0) << ", "
                << imu_bias_gyro(1) << ", " << imu_bias_gyro(2) << ", "
                << imu_bias_acc(0) << ", " << imu_bias_acc(1) << ", "
                << imu_bias_acc(2) << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
VisualizerLogger::VisualizerLogger() : Logger() {
  openLogFile(output_mesh_filename_);
  openLogFile(output_landmarks_filename_);
}

void VisualizerLogger::logLandmarks(const PointsWithId& lmks) {
  // Absolute vio errors
  std::ofstream& output_landmarks_stream =
      filename_to_outstream_.at(output_landmarks_filename_);
  output_landmarks_stream << "Id\t"
                          << "x\t"
                          << "y\t"
                          << "z\n";
  for (const PointWithId& point : lmks) {
    output_landmarks_stream << point.first << "\t" << point.second.x() << "\t"
                            << point.second.y() << "\t" << point.second.z()
                            << "\n";
  }
  output_landmarks_stream << std::endl;
}

void VisualizerLogger::logLandmarks(const cv::Mat& lmks) {
  // cv::Mat each row has a lmk with x, y, z.
  // Absolute vio errors
  std::ofstream& output_landmarks_stream =
      filename_to_outstream_.at(output_landmarks_filename_);
  output_landmarks_stream << "x\t"
                          << "y\t"
                          << "z\n";
  for (int i = 0; i < lmks.rows; i++) {
    output_landmarks_stream << lmks.at<float>(i, 0) << "\t"
                            << lmks.at<float>(i, 1) << "\t"
                            << lmks.at<float>(i, 2) << "\n";
  }
  output_landmarks_stream << std::endl;
}

void VisualizerLogger::logMesh(const cv::Mat& lmks,
                               const cv::Mat& colors,
                               const cv::Mat& mesh,
                               const double& timestamp,
                               bool log_accumulated_mesh) {
  std::ofstream& output_mesh_stream =
      filename_to_outstream_.at(output_mesh_filename_);
  CHECK(output_mesh_stream) << "Output File Mesh: error writing.";
  // Number of vertices in the mesh.
  int vertex_count = lmks.rows;
  // Number of faces in the mesh.
  int faces_count = std::round(mesh.rows / 4);
  // First, write header, but only once.
  static bool is_header_written = false;
  if (!is_header_written || !log_accumulated_mesh) {
    output_mesh_stream << "ply\n"
                       << "format ascii 1.0\n"
                       << "comment Mesh for SPARK VIO at timestamp "
                       << timestamp << "\n"
                       << "element vertex " << vertex_count << "\n"
                       << "property float x\n"
                       << "property float y\n"
                       << "property float z\n"
                       << "property uchar red\n"  // Start of vertex color.
                       << "property uchar green\n"
                       << "property uchar blue\n"
                       << "element face " << faces_count << "\n"
                       << "property list uchar int vertex_indices\n"
                       << "end_header\n";
    is_header_written = true;
  }

  // Second, log vertices.
  for (int i = 0; i < lmks.rows; i++) {
    output_mesh_stream << lmks.at<float>(i, 0) << " "  // Log vertices x y z.
                       << lmks.at<float>(i, 1) << " " << lmks.at<float>(i, 2)
                       << " " << int(colors.at<uint8_t>(i, 0))
                       << " "  // Log vertices colors.
                       << int(colors.at<uint8_t>(i, 1)) << " "
                       << int(colors.at<uint8_t>(i, 2)) << " \n";
  }
  // Finally, log faces.
  for (int i = 0; i < faces_count; i++) {
    // Assumes the mesh is made of triangles
    int index = i * 4;
    output_mesh_stream << mesh.at<int32_t>(index) << " "
                       << mesh.at<int32_t>(index + 1) << " "
                       << mesh.at<int32_t>(index + 2) << " "
                       << mesh.at<int32_t>(index + 3) << " \n";
  }
  output_mesh_stream << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
FrontendLogger::FrontendLogger() : Logger() {
  openLogFile(output_frontend_results_filename_, true);
  openLogFile(output_frontend_statistics_filename_, true);
  openLogFile(output_frontend_ransac_mono_, true);
  openLogFile(output_frontend_ransac_stereo_, true);
}

void FrontendLogger::logFrontendResults(
    const TrackerStatusSummary& tracker_summary,
    const size_t& nrKeypoints) {
  // We log frontend results in csv format.
  static bool is_header_written = false;

  std::ofstream& output_frontend_stream =
      filename_to_outstream_.at(output_frontend_results_filename_);
  if (!is_header_written) {
    output_frontend_stream << "mono_status, stereo_status, nr_keypoints"
                           << std::endl;
    is_header_written = true;
  }

  // Mono status.
  output_frontend_stream << TrackerStatusSummary::asString(
                                tracker_summary.kfTrackingStatus_mono_)
                         << ", ";
  // Stereo status.
  output_frontend_stream << TrackerStatusSummary::asString(
                                tracker_summary.kfTrackingStatus_stereo_)
                         << ", ";
  // Nr of keypoints.
  output_frontend_stream << nrKeypoints << std::endl;;
}

void FrontendLogger::logTrackerStatistics(
    const DebugTrackerInfo& tracker_info) {
  // We log frontend tracker statistics in csv format.
  static bool is_header_written = false;

  std::ofstream& output_frontend_stream =
      filename_to_outstream_.at(output_frontend_statistics_filename_);
  if (!is_header_written) {
    output_frontend_stream << "nrDetFeat, nrTrackFeat, nrMoIn, nrMoPu, nrStIn, "
                           << "nrStPu, moRaIt, stRaIt, nrVaRKP, nrNoLRKP, "
                           << "nrNoRRKP, nrNoDRKP, nrFaARKP, featDetTime, "
                           << "featTrackTime, moRanTime, stRanTime, "
                           << "featSelTime, extCorn, needNCorn"
                           << std::endl;
    is_header_written = true;
  }

  // Feature detection, tracking and ransac.
  output_frontend_stream << tracker_info.nrDetectedFeatures_ << ", "
                         << tracker_info.nrTrackerFeatures_ << ", "
                         << tracker_info.nrMonoInliers_ << ", "
                         << tracker_info.nrMonoPutatives_ << ", "
                         << tracker_info.nrStereoInliers_ << ", "
                         << tracker_info.nrStereoPutatives_ << ", "
                         << tracker_info.monoRansacIters_ << ", "
                         << tracker_info.stereoRansacIters_ << ", "

  // Performance of sparse-stereo-matching and ransac.
                         << tracker_info.nrValidRKP_ << ", "
                         << tracker_info.nrNoLeftRectRKP_ << ", "
                         << tracker_info.nrNoRightRectRKP_ << ", "
                         << tracker_info.nrNoDepthRKP_ << ", "
                         << tracker_info.nrFailedArunRKP_ << ", "

  // Info about timing.
                         << tracker_info.featureDetectionTime_ << ", "
                         << tracker_info.featureTrackingTime_ << ", "
                         << tracker_info.monoRansacTime_ << ", "
                         << tracker_info.stereoRansacTime_ << ", "

  // Info about feature selector.
                         << tracker_info.featureSelectionTime_ << ", "
                         << tracker_info.extracted_corners_ << ", "
                         << tracker_info.need_n_corners_ << ", "
                         << std::endl;
}

void FrontendLogger::logFrontendRansac(
    const gtsam::Pose3& relative_pose_body_mono,
    const gtsam::Pose3& relative_pose_body_stereo,
    const Timestamp& timestamp_lkf) {
  // We log the poses in csv format for later alignement and analysis.
  static bool is_header_written = false;
  std::ofstream& output_stream_mono =
      filename_to_outstream_.at(output_frontend_ransac_mono_);
  if (!is_header_written) {
    output_stream_mono << "timestamp_lkf, x, y, z, qx, qy, qz, qw, vx, vy, vz, "
                       << "bgx, bgy, bgz, bax, bay, baz" << std::endl;
    is_header_written = true;
  }

  // Log relative mono poses; pose from previous keyframe to current keyframe,
  // in previous-keyframe coordinates. These are not cumulative trajectories.
  const auto& mono_tran = relative_pose_body_mono.translation();
  const auto& mono_quat = relative_pose_body_mono.rotation().toQuaternion();

  output_stream_mono << timestamp_lkf << ", " << mono_tran.x() << ", "
                     << mono_tran.y() << ", " << mono_tran.z() << ", "
                     << mono_quat.x() << ", " << mono_quat.y() << ", "
                     << mono_quat.z() << ", " << mono_quat.w() << ", "
                     << 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", "
                     << 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " << 0
                     << std::endl;

  is_header_written = false;
  std::ofstream& output_stream_stereo =
     filename_to_outstream_.at(output_frontend_ransac_stereo_);
  if (!is_header_written) {
   output_stream_stereo << "timestamp_lkf, x, y, z, qx, qy, qz, qw, vx, vy, "
                        << "vz, bgx, bgy, bgz, bax, bay, baz" << std::endl;
   is_header_written = true;
  }

  // Log relative stereo poses; pose from previous keyframe to current keyframe,
  // in previous-keyframe coordinates. These are not cumulative trajectories.
  const auto& stereo_tran = relative_pose_body_stereo.translation();
  const auto& stereo_quat = relative_pose_body_stereo.rotation().toQuaternion();

  output_stream_stereo << timestamp_lkf   << ", " << stereo_tran.x() << ", "
                       << stereo_tran.y() << ", " << stereo_tran.z() << ", "
                       << stereo_quat.x() << ", " << stereo_quat.y() << ", "
                       << stereo_quat.z() << ", " << stereo_quat.w() << ", "
                       << 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", "
                       << 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " << 0
                       << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
PipelineLogger::PipelineLogger() : Logger() {
  openLogFile(output_pipeline_timing_);
}

void PipelineLogger::logPipelineOverallTiming(
    const std::chrono::milliseconds& duration) {
  // Add header.
  std::ofstream& outputFile_timingOverall_ =
      filename_to_outstream_.at(output_pipeline_timing_);
  outputFile_timingOverall_ << "vio_overall_time [ms]" << std::endl;
  outputFile_timingOverall_ << duration.count();
}

}  // namespace VIO
