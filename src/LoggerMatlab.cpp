/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoggerMatlab.cpp
 * @brief  Logging information for matlab stats and visualizations
 * @author Luca Carlone, Antoni Rosinol
 */

#include <LoggerMatlab.h>
#include <boost/foreach.hpp>
#include <memory>

#include "StereoVisionFrontEnd-definitions.h"
#include "UtilsOpenCV.h"
#include "utils/Statistics.h"
#include "utils/Timer.h"

DEFINE_string(output_path, "./", "Path where to store VIO's log output.");

namespace VIO {

Logger::Logger() : output_path_(FLAGS_output_path) {}

Logger::~Logger() { closeLogFiles(); }

void Logger::openLogFile(const std::string& output_file_name,
                         bool open_file_in_append_mode) {
  CHECK(!output_file_name.empty());
  CHECK(filename_to_outstream_.find(output_file_name) ==
        filename_to_outstream_.end())
      << "Using an already existing filename.";
  UtilsOpenCV::OpenFile(output_path_ + output_file_name,
                        &filename_to_outstream_[output_file_name],
                        open_file_in_append_mode);
}

void Logger::closeLogFiles() {
  for (auto& x : filename_to_outstream_) {
    VLOG(1) << "Closing output file: " << x.first.c_str();
    x.second.close();
  }
}

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
FrontendLogger::FrontendLogger() : Logger() {
  openLogFile(output_frontend_filename_);
}

void FrontendLogger::logFrontendResults(
    const TrackerStatusSummary& tracker_summary,
    const size_t& nrKeypoints) {
  // We log frontend results in csv format.
  static bool is_header_written = false;

  std::ofstream& output_frontend_stream =
      filename_to_outstream_.at(output_frontend_filename_);
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
  // Nr of keypoints
  output_frontend_stream << nrKeypoints << ", ";
}

VisualizerLogger::VisualizerLogger() : Logger() {
  openLogFile(output_mesh_filename_);
  openLogFile(output_landmarks_filename_);
}
////////////////////////////////////////////////////////////////////////////////
LoggerMatlab::LoggerMatlab() : output_path_(FLAGS_output_path) {}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::openLogFiles(int i, const std::string& output_file_name,
                                bool open_file_in_append_mode) {
  // Store output data and debug info:
  if (i == 0 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ +
            (output_file_name.empty() ? "/output.txt" : output_file_name),
        &outputFile_,
        open_file_in_append_mode);
  if (i == 1 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_posesVIO.txt"
                                                 : output_file_name),
        &outputFile_posesVIO_,
        open_file_in_append_mode);
  if (i == 2 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_posesGT.txt"
                                                 : output_file_name),
        &outputFile_posesGT_,
        open_file_in_append_mode);
  if (i == 3 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_landmarks.txt"
                                                 : output_file_name),
        &outputFile_landmarks_,
        open_file_in_append_mode);
  if (i == 4 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_normals.txt"
                                                 : output_file_name),
        &outputFile_normals_,
        open_file_in_append_mode);
  if (i == 5 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_smartFactors.txt"
                                                 : output_file_name),
        &outputFile_smartFactors_,
        open_file_in_append_mode);
  if (i == 6 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_timingVIO.txt"
                                                 : output_file_name),
        &outputFile_timingVIO_,
        open_file_in_append_mode);
  if (i == 7 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_timingTracker.txt"
                                                 : output_file_name),
        &outputFile_timingTracker_,
        open_file_in_append_mode);
  if (i == 8 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_statsTracker.txt"
                                                 : output_file_name),
        &outputFile_statsTracker_,
        open_file_in_append_mode);
  if (i == 9 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_statsFactors.txt"
                                                 : output_file_name),
        &outputFile_statsFactors_,
        open_file_in_append_mode);
  if (i == 10 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ +
            (output_file_name.empty() ? "/output_mesh.ply" : output_file_name),
        &outputFile_mesh_,
        open_file_in_append_mode);
  if (i == 11 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_timingOverall.csv"
                                                 : output_file_name),
        &outputFile_timingOverall_,
        open_file_in_append_mode);
  if (i == 12 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_frontend.csv"
                                                 : output_file_name),
        &outputFile_frontend_,
        open_file_in_append_mode);
  if (i == 13 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_posesVIO.csv"
                                                 : output_file_name),
        &outputFile_posesVIO_csv_,
        open_file_in_append_mode);
  if (i == 14 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ + (output_file_name.empty()
                                              ? "/output_posesVIO_pipeline.csv"
                                              : output_file_name),
                          &outputFile_posesVIO_csv_pipeline_,
                          open_file_in_append_mode);
  if (i == 15 || i == -1)
    UtilsOpenCV::OpenFile(
        output_path_ + (output_file_name.empty() ? "/output_initPerformance.csv"
                                                 : output_file_name),
        &outputFile_initPerformance_, open_file_in_append_mode);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::closeLogFiles(int i) {
  if (i == 0 || i == -1) outputFile_.close();
  if (i == 1 || i == -1) outputFile_posesVIO_.close();
  if (i == 2 || i == -1) outputFile_posesGT_.close();
  if (i == 3 || i == -1) outputFile_landmarks_.close();
  if (i == 4 || i == -1) outputFile_normals_.close();
  if (i == 5 || i == -1) outputFile_smartFactors_.close();
  if (i == 6 || i == -1) outputFile_timingVIO_.close();
  if (i == 7 || i == -1) outputFile_timingTracker_.close();
  if (i == 8 || i == -1) outputFile_statsTracker_.close();
  if (i == 9 || i == -1) outputFile_statsFactors_.close();
  if (i == 10 || i == -1) outputFile_mesh_.close();
  if (i == 11 || i == -1) outputFile_timingOverall_.close();
  if (i == 12 || i == -1) outputFile_frontend_.close();
  if (i == 13 || i == -1) outputFile_posesVIO_csv_.close();
  if (i == 14 || i == -1) outputFile_posesVIO_csv_pipeline_.close();
  if (i == 15 || i == -1) outputFile_initPerformance_.close();
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logLandmarks(const PointsWithId& lmks) {
  // Absolute vio errors
  if (outputFile_landmarks_) {
    outputFile_landmarks_ << "Id"
                          << "\t"
                          << "x"
                          << "\t"
                          << "y"
                          << "\t"
                          << "z\n";
    for (const PointWithId& point : lmks) {
      outputFile_landmarks_ << point.first << "\t" << point.second.x() << "\t"
                            << point.second.y() << "\t" << point.second.z()
                            << "\n";
    }
    outputFile_landmarks_ << std::endl;
  } else {
    throw std::runtime_error("Output File Landmarks: error writing.");
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logLandmarks(const cv::Mat& lmks) {
  // cv::Mat each row has a lmk with x, y, z.
  // Absolute vio errors
  if (outputFile_landmarks_) {
    outputFile_landmarks_ << "x"
                          << "\t"
                          << "y"
                          << "\t"
                          << "z" << std::endl;
    for (int i = 0; i < lmks.rows; i++) {
      outputFile_landmarks_ << lmks.at<float>(i, 0) << "\t"
                            << lmks.at<float>(i, 1) << "\t"
                            << lmks.at<float>(i, 2) << "\n";
    }
    outputFile_landmarks_ << std::endl;
  } else {
    throw std::runtime_error("Output File Landmarks: error writing.");
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logNormals(const std::vector<cv::Point3f>& normals) {
  if (outputFile_normals_) {
    outputFile_normals_ << "x"
                        << "\t"
                        << "y"
                        << "\t"
                        << "z" << std::endl;
    for (const cv::Point3f& normal : normals) {
      outputFile_normals_ << normal.x << "\t" << normal.y << "\t" << normal.z
                          << "\n";
    }
    outputFile_normals_.flush();
  } else {
    throw std::runtime_error("Output File Normals: error writing.");
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void VisualizerLogger::logMesh(const cv::Mat& lmks,
                               const cv::Mat& colors,
                               const cv::Mat& mesh,
                               const double& timestamp,
                               bool log_accumulated_mesh) {
  std::ofstream& output_ = filename_to_outstream_.at(output_mesh_filename_);
  // Number of vertices in the mesh.
  int vertex_count = lmks.rows;
  // Number of faces in the mesh.
  int faces_count = std::round(mesh.rows / 4);
  // First, write header, but only once.
  static bool is_header_written = false;
  if (!is_header_written || !log_accumulated_mesh) {
    output_ << "ply\n"
            << "format ascii 1.0\n"
            << "comment Mesh for SPARK VIO at timestamp " << timestamp << "\n"
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

    // Second, log vertices.
    for (int i = 0; i < lmks.rows; i++) {
      output_ << lmks.at<float>(i, 0) << " "  // Log vertices x y z.
              << lmks.at<float>(i, 1) << " " << lmks.at<float>(i, 2) << " "
              << int(colors.at<uint8_t>(i, 0)) << " "  // Log vertices colors.
              << int(colors.at<uint8_t>(i, 1)) << " "
              << int(colors.at<uint8_t>(i, 2)) << " \n";
    }
    // Finally, log faces.
    for (int i = 0; i < faces_count; i++) {
      // Assumes the mesh is made of triangles
      int index = i * 4;
      output_ << mesh.at<int32_t>(index) << " " << mesh.at<int32_t>(index + 1)
              << " " << mesh.at<int32_t>(index + 2) << " "
              << mesh.at<int32_t>(index + 3) << " \n";
    }
    output_ << std::endl;
  } else {
    throw std::runtime_error("Output File Mesh: error writing.");
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logInitializationResultsCSV(
    const InitializationPerformance& perf,
    const double& ba_duration,
    const double& alignment_duration,
    const bool& is_successful) {
  // We log the poses in csv format for later analysis.
  static bool is_header_written = false;
  if (!is_header_written) {
    outputFile_initPerformance_
        << "timestamp,N_frames,avg_relRotErrorBA,avg_relTranErrorBA,"
           "ba_duration(ms),alignment_duration(ms),"
           "init_roll(deg),init_pitch(deg),init_yaw(deg),"
           "init_body_vx,init_body_vy,init_body_vz,"
           "init_gyro_bx,init_gyro_by,init_gyro_bz,"
           "init_grav_bx,init_grav_by,init_grav_bz,"
           "gt_roll(deg),gt_pitch(deg),gt_yaw(deg),"
           "gt_body_vx,gt_body_vy,gt_body_vz,"
           "gt_gyro_bx,gt_gyro_by,gt_gyro_bz,"
           "gt_grav_bx,gt_grav_by,gt_grav_bz,"
           "is_successful,"
        << std::endl;
    is_header_written = true;
  }
  outputFile_initPerformance_
      << perf.init_timestamp_ << ", " << perf.init_n_frames_ << ", "
      << perf.avg_rotationErrorBA_ << ", " << perf.avg_tranErrorBA_ << ", "
      << ba_duration << ", " << alignment_duration << ", "
      << perf.init_nav_state_.pose().rotation().pitch() * 180.0 / M_PI << ", "
      << perf.init_nav_state_.pose().rotation().roll() * 180.0 / M_PI << ", "
      << perf.init_nav_state_.pose().rotation().yaw() * 180.0 / M_PI << ", "
      << perf.init_nav_state_.velocity_.x() << ", "
      << perf.init_nav_state_.velocity_.y() << ", "
      << perf.init_nav_state_.velocity_.z() << ", "
      << perf.init_nav_state_.imu_bias_.gyroscope().x() << ", "
      << perf.init_nav_state_.imu_bias_.gyroscope().y() << ", "
      << perf.init_nav_state_.imu_bias_.gyroscope().z() << ", "
      << perf.init_gravity_.x() << ", " << perf.init_gravity_.y() << ", "
      << perf.init_gravity_.z() << ", "
      << perf.gt_nav_state_.pose().rotation().pitch() * 180.0 / M_PI << ", "
      << perf.gt_nav_state_.pose().rotation().roll() * 180.0 / M_PI << ", "
      << perf.gt_nav_state_.pose().rotation().yaw() * 180.0 / M_PI << ", "
      << perf.gt_nav_state_.velocity_.x() << ", "
      << perf.gt_nav_state_.velocity_.y() << ", "
      << perf.gt_nav_state_.velocity_.z() << ", "
      << perf.gt_nav_state_.imu_bias_.gyroscope().x() << ", "
      << perf.gt_nav_state_.imu_bias_.gyroscope().y() << ", "
      << perf.gt_nav_state_.imu_bias_.gyroscope().z() << ", "
      << perf.gt_gravity_.x() << ", " << perf.gt_gravity_.y() << ", "
      << perf.gt_gravity_.z() << ", " << is_successful << ", " << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logBackendResults(
    const ETHDatasetParser& dataset,
    const TrackerStatusSummary& tracker_status_summary,
    const gtsam::Pose3& relative_pose_body_mono, const Tracker& tracker,
    const gtsam::Pose3& relative_pose_body_stereo,
    const std::shared_ptr<VioBackEndOutputPayload>& vio_output,
    const double& horizon, const Timestamp& timestamp_lkf,
    const Timestamp& timestamp_k, const size_t& k) {
  // Log how long it takes to log the backend.
  double start_time = UtilsOpenCV::GetTimeInSeconds();
  CHECK(vio_output);
  double vioRotError, vioTranError;
  gtsam::Pose3 W_Pose_Bkf_gt = (dataset.getGroundTruthState(timestamp_k)).pose_;
  std::tie(vioRotError, vioTranError) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(
          W_Pose_Bkf_gt, vio_output->W_Pose_Blkf_);
  LOG(INFO) << "vioRotError " << vioRotError << ", vioTranError "
            << vioTranError;

  // Absolute vio errors
  outputFile_ << vio_output->cur_kf_id_ << " " << vioRotError << " "
              << vioTranError << " " << vio_output->landmark_count_ << " ";

  // RPY vio errors
  gtsam::Vector3 rpy_gt =
      W_Pose_Bkf_gt.rotation().rpy();  // such that R = Rot3::Ypr(y,p,r)
  gtsam::Vector3 rpy_vio = vio_output->W_Pose_Blkf_.rotation().rpy();
  outputFile_ << rpy_gt(0) << " " << rpy_gt(1) << " " << rpy_gt(2) << " "
              << rpy_vio(0) << " " << rpy_vio(1) << " " << rpy_vio(2) << " ";

  // relative vio errors
  double rel_rot_error, rel_tran_error;
  gtsam::Pose3 Bprevkf_Pose_Bkf_vio =
      W_Pose_Bprevkf_vio_.between(vio_output->W_Pose_Blkf_);
  boost::tie(rel_rot_error, rel_tran_error) =
      dataset.computePoseErrors(Bprevkf_Pose_Bkf_vio, true, timestamp_lkf,
                                timestamp_k);  // always VALID = TRUE
  outputFile_ << rel_rot_error << " " << rel_tran_error << " ";

  // relative imu rotation errors
  double relativeRotError_imu_wrt_gt, relativeRotError_imu_wrt_5point;
  gtsam::Pose3 Bprevkf_Pose_Bkf_imuPreint(
      vio_output->debug_info_.imuR_lkf_kf,
      gtsam::Point3());  // rotation from imu preintegration, no translation
  boost::tie(relativeRotError_imu_wrt_gt, rel_tran_error) =
      dataset.computePoseErrors(Bprevkf_Pose_Bkf_imuPreint, true, timestamp_lkf,
                                timestamp_k);  // always VALID = TRUE
  std::tie(relativeRotError_imu_wrt_5point, rel_tran_error) =
      UtilsOpenCV::ComputeRotationAndTranslationErrors(
          Bprevkf_Pose_Bkf_imuPreint, relative_pose_body_mono);
  outputFile_ << relativeRotError_imu_wrt_gt << " "
              << relativeRotError_imu_wrt_5point << " ";

  // relative imu prediction errors
  gtsam::Pose3 Bprevkf_Pose_Bkf_imuPredict =
      W_Pose_Bprevkf_vio_.between(vio_output->debug_info_.navstate_k_.pose());
  boost::tie(rel_rot_error, rel_tran_error) = dataset.computePoseErrors(
      Bprevkf_Pose_Bkf_imuPredict, true, timestamp_lkf,
      timestamp_k);  // always VALID = TRUE
  outputFile_ << rel_rot_error << " " << rel_tran_error << " ";

  // check consistency of stereo translation estimate
  gtsam::Vector3 Tstereo = relative_pose_body_stereo.translation().vector();
  gtsam::Vector3 Tgt =
      dataset.getGroundTruthRelativePose(timestamp_lkf, timestamp_k)
          .translation()
          .vector();
  gtsam::Matrix3 infoMat = tracker_status_summary.infoMatStereoTranslation_;
  outputFile_ << (Tstereo - Tgt).norm() << " "
              << (Tstereo - Tgt).transpose() * infoMat * (Tstereo - Tgt) << " "
              << std::endl;

  // debug smart factors:
  outputFile_smartFactors_
      << vio_output->cur_kf_id_ << " " << k << " "
      << UtilsOpenCV::NsecToSec(
             timestamp_k)  // keyframe id, frame id, timestamp
      << " " << vio_output->debug_info_.numSF_ << " "
      << vio_output->debug_info_.numValid_ << " "
      << vio_output->debug_info_.numDegenerate_ << " "
      << vio_output->debug_info_.numFarPoints_ << " "
      << vio_output->debug_info_.numOutliers_ << " "
      << vio_output->debug_info_.numCheirality_ << " "
      << vio_output->debug_info_.meanPixelError_ << " "
      << vio_output->debug_info_.maxPixelError_ << " "
      << vio_output->debug_info_.meanTrackLength_ << " "
      << vio_output->debug_info_.maxTrackLength_ << " "
      << vio_output->debug_info_.nrElementsInMatrix_ << " "
      << vio_output->debug_info_.nrZeroElementsInMatrix_ << std::endl;

  // we log the camera since we will display camera poses in matlab
  gtsam::Pose3 W_Pose_camlkf_vio =
      vio_output->W_Pose_Blkf_.compose(vio_output->B_Pose_leftCam_);
  outputFile_posesVIO_ << vio_output->cur_kf_id_ << " "
                       << W_Pose_camlkf_vio.translation().transpose() << " "
                       << W_Pose_camlkf_vio.rotation().matrix().row(0) << " "
                       << W_Pose_camlkf_vio.rotation().matrix().row(1) << " "
                       << W_Pose_camlkf_vio.rotation().matrix().row(2) << " "
                       << vio_output->W_Vel_Blkf_.transpose() << " "
                       << vio_output->imu_bias_lkf_.accelerometer().transpose()
                       << " "
                       << vio_output->imu_bias_lkf_.gyroscope().transpose()
                       << std::endl;
  // we log the camera since we will display camera poses in matlab
  gtsam::Pose3 W_Pose_camlkf_gt =
      W_Pose_Bkf_gt.compose(vio_output->B_Pose_leftCam_);
  Vector3 W_Vel_camlkf_gt =
      (dataset.getGroundTruthState(timestamp_k)).velocity_;
  gtsam::imuBias::ConstantBias imu_bias_lkf_gt =
      (dataset.getGroundTruthState(timestamp_k)).imu_bias_;
  outputFile_posesGT_ << vio_output->cur_kf_id_ << " "
                      << W_Pose_camlkf_gt.translation().transpose() << " "
                      << W_Pose_camlkf_gt.rotation().matrix().row(0) << " "
                      << W_Pose_camlkf_gt.rotation().matrix().row(1) << " "
                      << W_Pose_camlkf_gt.rotation().matrix().row(2) << " "
                      << W_Vel_camlkf_gt.transpose() << " "
                      << imu_bias_lkf_gt.accelerometer().transpose() << " "
                      << imu_bias_lkf_gt.gyroscope().transpose() << std::endl;

  // log timing for benchmarking and performance profiling
  outputFile_timingVIO_ << vio_output->cur_kf_id_ << " "
                        << vio_output->debug_info_.factorsAndSlotsTime_ << " "
                        << vio_output->debug_info_.preUpdateTime_ << " "
                        << vio_output->debug_info_.updateTime_ << " "
                        << vio_output->debug_info_.updateSlotTime_ << " "
                        << vio_output->debug_info_.extraIterationsTime_ << " "
                        << vio_output->debug_info_.printTime_ << " "
                        << vio_output->debug_info_.linearizeTime_ << " "
                        << vio_output->debug_info_.linearSolveTime_ << " "
                        << vio_output->debug_info_.retractTime_ << " "
                        << vio_output->debug_info_.linearizeMarginalizeTime_
                        << " " << vio_output->debug_info_.marginalizeTime_
                        << " " << vio_output->debug_info_.imuPreintegrateTime_
                        << std::endl;

  outputFile_timingTracker_ << vio_output->cur_kf_id_ << " "
                            << tracker.debugInfo_.featureDetectionTime_ << " "
                            << tracker.debugInfo_.featureTrackingTime_ << " "
                            << tracker.debugInfo_.monoRansacTime_ << " "
                            << tracker.debugInfo_.stereoRansacTime_ << " "
                            << tracker.debugInfo_.monoRansacIters_ << " "
                            << tracker.debugInfo_.stereoRansacIters_
                            << std::endl;
  // This should be done in the feature selection own logger!!
  // stereoTracker.tracker_.debugInfo_.featureSelectionTime_ << " " <<
  // std::endl;

  // log performance of tracker (currently we only log info at keyframes!!)
  outputFile_statsTracker_ << vio_output->cur_kf_id_ << " "
                           << tracker.debugInfo_.nrDetectedFeatures_ << " "
                           << tracker.debugInfo_.nrTrackerFeatures_ << " "
                           << tracker.debugInfo_.nrMonoInliers_ << " "
                           << tracker.debugInfo_.nrMonoPutatives_ << " "
                           << tracker.debugInfo_.nrStereoInliers_ << " "
                           << tracker.debugInfo_.nrStereoPutatives_ << " "
                           << tracker.debugInfo_.nrValidRKP_ << " "
                           << tracker.debugInfo_.nrNoLeftRectRKP_ << " "
                           << tracker.debugInfo_.nrNoRightRectRKP_ << " "
                           << tracker.debugInfo_.nrNoDepthRKP_ << " "
                           << tracker.debugInfo_.nrFailedArunRKP_ << " "
                           << tracker.debugInfo_.need_n_corners_ << " "
                           << tracker.debugInfo_.extracted_corners_
                           << std::endl;

  // Statistics about factors added to the graph.
  outputFile_statsFactors_ << vio_output->cur_kf_id_ << " "
                           << vio_output->debug_info_.numAddedSmartF_ << " "
                           << vio_output->debug_info_.numAddedImuF_ << " "
                           << vio_output->debug_info_.numAddedNoMotionF_ << " "
                           << vio_output->debug_info_.numAddedConstantVelF_
                           << " "
                           << vio_output->debug_info_.numAddedBetweenStereoF_
                           << " " << vio_output->state_.size() << " "
                           <<  // current number of states
      3 * std::min(double(vio_output->cur_kf_id_ + 1),
                   horizon / (tracker.trackerParams_.intra_keyframe_time_) + 1)
                           << std::endl;  // expected nr of states
}

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
