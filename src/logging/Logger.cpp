/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Logger.cpp
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
// This constructor will directly open the log file when called.
OfstreamWrapper::OfstreamWrapper(const std::string& filename,
                                 const bool& open_file_in_append_mode)
    : filename_(filename), output_path_(FLAGS_output_path) {
  openLogFile(filename);
}

// This destructor will directly close the log file when the wrapper is
// destructed. So no need to explicitly call .close();
OfstreamWrapper::~OfstreamWrapper() {
  LOG(INFO) << "Closing output file: " << filename_.c_str();
  ofstream_.close();
};

void OfstreamWrapper::openLogFile(const std::string& output_file_name,
                                  bool open_file_in_append_mode) {
  CHECK(!output_file_name.empty());
  LOG(INFO) << "Opening output file: " << output_file_name.c_str();
  UtilsOpenCV::OpenFile(output_path_ + '/' + output_file_name,
                        &ofstream_,
                        open_file_in_append_mode);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
BackendLogger::BackendLogger()
    : output_poses_vio_csv_("output_posesVIO.csv"),
      output_smart_factors_stats_csv_("output_smartFactors.csv"),
      output_pim_navstates_csv_("output_pim_navstates.csv"),
      output_backend_factors_stats_csv_("output_backendFactors.csv"),
      output_backend_timing_csv_("output_backendTiming.csv"){};

void BackendLogger::logBackendOutput(const VioBackEndOutputPayload& output) {
  logBackendResultsCSV(output);
  logBackendFactorsStats(output);
  logBackendPimNavstates(output);
  logSmartFactorsStats(output);
  logBackendTiming(output);
}

void BackendLogger::displayInitialStateVioInfo(
    const gtsam::Vector3& n_gravity_,
    const gtsam::Pose3& W_Pose_B_Lkf,
    const VioNavState& initial_state_gt,
    const ImuAccGyrS& imu_accgyr,
    const Timestamp& timestamp_k) const {
  initial_state_gt.print("initialStateGT\n");
  const gtsam::Vector3& rpy_gt = initial_state_gt.pose_.rotation()
                                     .rpy();  // such that R = Rot3::Ypr(y,p,r)
  LOG(INFO) << "yaw= " << rpy_gt(2) << ' ' << "pitch= " << rpy_gt(1) << ' '
            << "roll= " << rpy_gt(0);
  gtsam::Vector3 localGravity =
      initial_state_gt.pose_.rotation().inverse().matrix() * n_gravity_;
  LOG(INFO) << "gravity in global frame: \n"
            << n_gravity_ << '\n'
            << "gravity in local frame: \n"
            << localGravity << '\n'
            << "expected initial acc measurement (no bias correction): \n"
            << -localGravity << '\n'
            << "expected initial acc measurement: \n"
            << -localGravity + initial_state_gt.imu_bias_.accelerometer()
            << '\n'
            << "actual initial acc measurement: \n"
            << imu_accgyr.block<3, 1>(0, 0) << '\n'
            << "expected initial gyro measurement: \n"
            << initial_state_gt.imu_bias_.gyroscope() << '\n'
            << "actual initial gyro measurement: \n"
            << imu_accgyr.block<3, 1>(3, 0);

  double vioRotError, vioTranError;
  std::tie(vioRotError, vioTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(initial_state_gt.pose_,
                                                       W_Pose_B_Lkf);
  CHECK_LE(vioRotError, 1e-4);
  CHECK_LE(vioTranError, 1e-4)
      << "stereoVIOExample: wrong initialization (ground truth initialization)";
}

void BackendLogger::logBackendResultsCSV(
    const VioBackEndOutputPayload& vio_output) {
  // We log the poses in csv format for later alignement and analysis.
  std::ofstream& output_stream = output_poses_vio_csv_.ofstream_;

  // First, write header, but only once.
  static bool is_header_written = false;
  if (!is_header_written) {
    output_stream << "timestamp,x,y,z,qx,qy,qz,qw,vx,vy,vz,"
                  << "bgx,bgy,bgz,bax,bay,baz"
                  << std::endl;
    is_header_written = true;
  }
  // TODO(marcus): everything on EVO and evaluation needs to change for the new
  // qw before qx paradigm!
  const auto& w_pose_blkf_trans =
      vio_output.W_Pose_Blkf_.translation().transpose();
  const auto& w_pose_blkf_rot = vio_output.W_Pose_Blkf_.rotation().quaternion();
  const auto& w_vel_blkf = vio_output.W_Vel_Blkf_.transpose();
  const auto& imu_bias_gyro = vio_output.imu_bias_lkf_.gyroscope().transpose();
  const auto& imu_bias_acc =
      vio_output.imu_bias_lkf_.accelerometer().transpose();
  output_stream << vio_output.timestamp_kf_ << ","  //
                << w_pose_blkf_trans.x() << ","     //
                << w_pose_blkf_trans.y() << ","     //
                << w_pose_blkf_trans.z() << ","     //
                << w_pose_blkf_rot(1) << ","        // q_x
                << w_pose_blkf_rot(2) << ","        // q_y
                << w_pose_blkf_rot(3) << ","        // q_z
                << w_pose_blkf_rot(0) << ","        // q_w
                << w_vel_blkf(0) << ","             //
                << w_vel_blkf(1) << ","             //
                << w_vel_blkf(2) << ","             //
                << imu_bias_gyro(0) << ","          //
                << imu_bias_gyro(1) << ","          //
                << imu_bias_gyro(2) << ","          //
                << imu_bias_acc(0) << ","           //
                << imu_bias_acc(1) << ","           //
                << imu_bias_acc(2)                  //
                << std::endl;
}

void BackendLogger::logSmartFactorsStats(
    const VioBackEndOutputPayload& output) {
  std::ofstream& output_stream = output_smart_factors_stats_csv_.ofstream_;

  // First, write header, but only once.
  static bool is_header_written = false;
  if (!is_header_written) {
    output_stream << "cur_kf_id,timestamp_kf,numSF,"
                  << "numValid,numDegenerate,numFarPoints,numOutliers,"
                  << "numCheirality,numNonInitialized,meanPixelError,"
                  << "maxPixelError,meanTrackLength,maxTrackLength,"
                  << "nrElementsInMatrix,nrZeroElementsInMatrix"
                  << std::endl;
    is_header_written = true;
  }

  output_stream << output.cur_kf_id_ << ","
                << output.timestamp_kf_ << ","
                << output.debug_info_.numSF_ << ","
                << output.debug_info_.numValid_ << ","
                << output.debug_info_.numDegenerate_ << ","
                << output.debug_info_.numFarPoints_ << ","
                << output.debug_info_.numOutliers_ << ","
                << output.debug_info_.numCheirality_ << ","
                << output.debug_info_.numNonInitialized_ << ","
                << output.debug_info_.meanPixelError_ << ","
                << output.debug_info_.maxPixelError_ << ","
                << output.debug_info_.meanTrackLength_ << ","
                << output.debug_info_.maxTrackLength_ << ","
                << output.debug_info_.nrElementsInMatrix_ << ","
                << output.debug_info_.nrZeroElementsInMatrix_
                << std::endl;
}

void BackendLogger::logBackendPimNavstates(
    const VioBackEndOutputPayload& output) {
  std::ofstream& output_stream = output_pim_navstates_csv_.ofstream_;

  // First, write header, but only once.
  static bool is_header_written = false;
  if (!is_header_written) {
    output_stream << "timestamp_kf,x,y,z,qw,qx,qy,qz,vx,vy,vz"
                  << std::endl;
    is_header_written = true;
  }

  const gtsam::Pose3& pose = output.debug_info_.navstate_k_.pose();
  const gtsam::Point3& position = pose.translation();
  const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();
  const gtsam::Velocity3& velocity = output.debug_info_.navstate_k_.velocity();

  output_stream << output.timestamp_kf_ << ","
                << position.x() << ","
                << position.y() << ","
                << position.z() << ","
                << quaternion.w() << ","
                << quaternion.x() << ","
                << quaternion.y() << ","
                << quaternion.z() << ","
                << velocity.x() << ","
                << velocity.y() << ","
                << velocity.z()
                << std::endl;
}

void BackendLogger::logBackendTiming(const VioBackEndOutputPayload& output) {
  std::ofstream& output_stream = output_backend_timing_csv_.ofstream_;

  // First, write header, but only once.
  static bool is_header_written = false;
  if (!is_header_written) {
    output_stream << "cur_kf_id,factorsAndSlotsTime,preUpdateTime,"
                  << "updateTime,updateSlotTime,extraIterationsTime,"
                  << "linearizeTime,linearSolveTime,retractTime,"
                  << "linearizeMarginalizeTime,marginalizeTime"
                  << std::endl;
    is_header_written = true;
  }

  // Log timing for benchmarking and performance profiling.
  output_stream << output.cur_kf_id_ << ","
                << output.debug_info_.factorsAndSlotsTime_ << ","
                << output.debug_info_.preUpdateTime_ << ","
                << output.debug_info_.updateTime_ << ","
                << output.debug_info_.updateSlotTime_ << ","
                << output.debug_info_.extraIterationsTime_ << ","
                << output.debug_info_.linearizeTime_ << ","
                << output.debug_info_.linearSolveTime_ << ","
                << output.debug_info_.retractTime_ << ","
                << output.debug_info_.linearizeMarginalizeTime_ << ","
                << output.debug_info_.marginalizeTime_
                << std::endl;
}

void BackendLogger::logBackendFactorsStats(
    const VioBackEndOutputPayload& output) {
  std::ofstream& output_stream = output_backend_factors_stats_csv_.ofstream_;

  // First, write header, but only once.
  static bool is_header_written = false;
  if (!is_header_written) {
    output_stream << "cur_kf_id,numAddedSmartF,numAddedImuF,numAddedNoMotionF,"
                  << "numAddedConstantF,numAddedBetweenStereoF,state_size,"
                  << "landmark_count"
                  << std::endl;
    is_header_written = true;
  }

  // Log timing for benchmarking and performance profiling.
  // Statistics about factors added to the graph.
  output_stream << output.cur_kf_id_ << ","
                << output.debug_info_.numAddedSmartF_ << ","
                << output.debug_info_.numAddedImuF_ << ","
                << output.debug_info_.numAddedNoMotionF_ << ","
                << output.debug_info_.numAddedConstantVelF_ << ","
                << output.debug_info_.numAddedBetweenStereoF_ << ","
                << output.state_.size() << ","
                << output.landmark_count_
                << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
VisualizerLogger::VisualizerLogger()
    : output_mesh_("output_mesh.ply"),
      output_landmarks_("output_landmarks.txt"){};

void VisualizerLogger::logLandmarks(const PointsWithId& lmks) {
  // Absolute vio errors
  std::ofstream& output_landmarks_stream = output_landmarks_.ofstream_;
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
  std::ofstream& output_landmarks_stream = output_landmarks_.ofstream_;
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
  std::ofstream& output_mesh_stream = output_mesh_.ofstream_;
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
FrontendLogger::FrontendLogger()
    : output_frontend_stats_("output_frontend_stats.csv"),
      output_frontend_ransac_mono_("output_frontend_ransac_mono.csv"),
      output_frontend_ransac_stereo_("output_frontend_ransac_stereo.csv") {};

void FrontendLogger::logFrontendStats(
    const Timestamp& timestamp_lkf,
    const DebugTrackerInfo& tracker_info,
    const TrackerStatusSummary& tracker_summary,
    const size_t& nrKeypoints) {
  // We log frontend results in csv format.
  std::ofstream& output_stream_stats = output_frontend_stats_.ofstream_;

  static bool is_header_written = false;
  if (!is_header_written) {
    output_stream_stats << "timestamp_lkf,mono_status,stereo_status,"
                        << "nr_keypoints,nrDetectedFeatures,nrTrackerFeatures,"
                        << "nrMonoInliers,nrMonoPutatives,nrStereoInliers,"
                        << "nrStereoPutatives,monoRansacIters,"
                        << "stereoRansacIters,nrValidRKP,nrNoLeftRectRKP,"
                        << "nrNoRightRectRKP,nrNoDepthRKP,nrFailedArunRKP,"
                        << "featureDetectionTime,featureTrackingTime,"
                        << "monoRansacTime,stereoRansacTime,"
                        << "featureSelectionTime,extracted_corners,"
                        << "need_n_corners"
                        << std::endl;
    is_header_written = true;
  }

  output_stream_stats << timestamp_lkf << ","
  // Mono status.
                      << TrackerStatusSummary::asString(
                              tracker_summary.kfTrackingStatus_mono_) << ","
  // Stereo status.
                      << TrackerStatusSummary::asString(
                              tracker_summary.kfTrackingStatus_stereo_) << ","
  // Nr of keypoints.
                      << nrKeypoints << ","
  // Feature detection, tracking and ransac.
                      << tracker_info.nrDetectedFeatures_ << ","
                      << tracker_info.nrTrackerFeatures_ << ","
                      << tracker_info.nrMonoInliers_ << ","
                      << tracker_info.nrMonoPutatives_ << ","
                      << tracker_info.nrStereoInliers_ << ","
                      << tracker_info.nrStereoPutatives_ << ","
                      << tracker_info.monoRansacIters_ << ","
                      << tracker_info.stereoRansacIters_ << ","
  // Performance of sparse-stereo-matching and ransac.
                      << tracker_info.nrValidRKP_ << ","
                      << tracker_info.nrNoLeftRectRKP_ << ","
                      << tracker_info.nrNoRightRectRKP_ << ","
                      << tracker_info.nrNoDepthRKP_ << ","
                      << tracker_info.nrFailedArunRKP_ << ","
  // Info about timing.
                      << tracker_info.featureDetectionTime_ << ","
                      << tracker_info.featureTrackingTime_ << ","
                      << tracker_info.monoRansacTime_ << ","
                      << tracker_info.stereoRansacTime_ << ","
  // Info about feature selector.
                      << tracker_info.featureSelectionTime_ << ","
                      << tracker_info.extracted_corners_ << ","
                      << tracker_info.need_n_corners_
                      << std::endl;
}

void FrontendLogger::logFrontendRansac(
    const Timestamp& timestamp_lkf,
    const gtsam::Pose3& relative_pose_body_mono,
    const gtsam::Pose3& relative_pose_body_stereo) {
  // We log the relative poses in csv format for later analysis.
  std::ofstream& output_stream_mono = output_frontend_ransac_mono_.ofstream_;
  std::ofstream& output_stream_stereo =
      output_frontend_ransac_stereo_.ofstream_;

  static bool is_header_written = false;
  if (!is_header_written) {
    output_stream_mono << "timestamp_lkf,x,y,z,qw,qx,qy,qz"
                       << std::endl;
    output_stream_stereo << "timestamp_lkf,x,y,z,qw,qx,qy,qz"
                         << std::endl;
    is_header_written = true;
  }

  // Log relative mono poses; pose from previous keyframe to current keyframe,
  // in previous-keyframe coordinates. These are not cumulative trajectories.
  const gtsam::Point3& mono_tran = relative_pose_body_mono.translation();
  const gtsam::Quaternion& mono_quat = relative_pose_body_mono.rotation().toQuaternion();

  output_stream_mono << timestamp_lkf << ","
                     << mono_tran.x() << ","
                     << mono_tran.y() << ","
                     << mono_tran.z() << ","
                     << mono_quat.w() << ","
                     << mono_quat.x() << ","
                     << mono_quat.y() << ","
                     << mono_quat.z()
                     << std::endl;

  // Log relative stereo poses; pose from previous keyframe to current keyframe,
  // in previous-keyframe coordinates. These are not cumulative trajectories.
  const gtsam::Point3& stereo_tran = relative_pose_body_stereo.translation();
  const gtsam::Quaternion& stereo_quat = relative_pose_body_stereo.rotation().toQuaternion();

  output_stream_stereo << timestamp_lkf   << ","
                       << stereo_tran.x() << ","
                       << stereo_tran.y() << ","
                       << stereo_tran.z() << ","
                       << stereo_quat.w() << ","
                       << stereo_quat.x() << ","
                       << stereo_quat.y() << ","
                       << stereo_quat.z()
                       << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
PipelineLogger::PipelineLogger()
    : output_pipeline_timing_("output_timingOverall.csv"){};

void PipelineLogger::logPipelineOverallTiming(
    const std::chrono::milliseconds& duration) {
  // Add header.
  std::ofstream& outputFile_timingOverall_ = output_pipeline_timing_.ofstream_;
  outputFile_timingOverall_ << "vio_overall_time [ms]" << std::endl;
  outputFile_timingOverall_ << duration.count();
}

}  // namespace VIO
