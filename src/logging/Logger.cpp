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
 * @author Antoni Rosinol
 */

#include "kimera-vio/logging/Logger.h"

#include <fstream>
#include <memory>
#include <string>

#include <boost/filesystem.hpp>  // to create folders
#include <boost/foreach.hpp>

#include <gflags/gflags.h>

#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DEFINE_string(output_path, "./", "Path where to store VIO's log output.");

namespace VIO {

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
// This constructor will directly open the log file when called.
OfstreamWrapper::OfstreamWrapper(const std::string& filename,
                                 const bool& open_file_in_append_mode)
    : filename_(filename), output_path_(FLAGS_output_path) {
  openLogFile(filename, open_file_in_append_mode);
}

// This destructor will directly close the log file when the wrapper is
// destructed. So no need to explicitly call .close();
OfstreamWrapper::~OfstreamWrapper() {
  LOG(INFO) << "Closing output file: " << filename_.c_str();
  ofstream_.close();
}

void OfstreamWrapper::closeAndOpenLogFile() {
  ofstream_.close();
  CHECK(!filename_.empty());
  OpenFile(output_path_ + '/' + filename_, &ofstream_, false);
}

void OfstreamWrapper::openLogFile(const std::string& output_file_name,
                                  bool open_file_in_append_mode) {
  CHECK(!output_file_name.empty());
  LOG(INFO) << "Opening output file: " << output_file_name.c_str();
  OpenFile(output_path_ + '/' + output_file_name,
           &ofstream_,
           open_file_in_append_mode);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
EurocGtLogger::EurocGtLogger() : output_gt_poses_csv_("traj_gt.csv") {}

void EurocGtLogger::logGtData(const std::string& file_path) {
  std::ifstream f_in(file_path.c_str());
  CHECK(f_in.is_open()) << "Cannot open file: " << file_path;
  // Drop first line, we want to use our own header.
  std::string dummy_header;
  std::getline(f_in, dummy_header);

  std::ofstream& output_stream = output_gt_poses_csv_.ofstream_;
  // First, write header
  output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,"
                << "bgx,bgy,bgz,bax,bay,baz" << std::endl;
  // Then, copy all gt data to file
  output_stream << f_in.rdbuf();

  // Clean
  f_in.close();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
BackendLogger::BackendLogger()
    : output_poses_vio_csv_("traj_vio.csv"),
      output_smart_factors_stats_csv_("output_smartFactors.csv"),
      output_pim_navstates_csv_("output_pim_navstates.csv"),
      output_backend_factors_stats_csv_("output_backendFactors.csv"),
      output_backend_timing_csv_("output_backendTiming.csv") {}

void BackendLogger::logBackendOutput(const BackendOutput& output) {
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

void BackendLogger::logBackendResultsCSV(const BackendOutput& vio_output) {
  // We log the poses in csv format for later alignement and analysis.
  std::ofstream& output_stream = output_poses_vio_csv_.ofstream_;
  bool& is_header_written = is_header_written_poses_vio_;

  // First, write header, but only once.
  if (!is_header_written) {
    output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,"
                  << "bgx,bgy,bgz,bax,bay,baz" << std::endl;
    is_header_written = true;
  }
  const auto& cached_state = vio_output.W_State_Blkf_;
  const auto& w_pose_blkf_trans = cached_state.pose_.translation().transpose();
  const auto& w_pose_blkf_rot = cached_state.pose_.rotation().quaternion();
  const auto& w_vel_blkf = cached_state.velocity_.transpose();
  const auto& imu_bias_gyro = cached_state.imu_bias_.gyroscope().transpose();
  const auto& imu_bias_acc = cached_state.imu_bias_.accelerometer().transpose();
  output_stream << cached_state.timestamp_ << ","  //
                << w_pose_blkf_trans.x() << ","    //
                << w_pose_blkf_trans.y() << ","    //
                << w_pose_blkf_trans.z() << ","    //
                << w_pose_blkf_rot(0) << ","       // q_w
                << w_pose_blkf_rot(1) << ","       // q_x
                << w_pose_blkf_rot(2) << ","       // q_y
                << w_pose_blkf_rot(3) << ","       // q_z
                << w_vel_blkf(0) << ","            //
                << w_vel_blkf(1) << ","            //
                << w_vel_blkf(2) << ","            //
                << imu_bias_gyro(0) << ","         //
                << imu_bias_gyro(1) << ","         //
                << imu_bias_gyro(2) << ","         //
                << imu_bias_acc(0) << ","          //
                << imu_bias_acc(1) << ","          //
                << imu_bias_acc(2)                 //
                << std::endl;
}

void BackendLogger::logSmartFactorsStats(const BackendOutput& output) {
  std::ofstream& output_stream = output_smart_factors_stats_csv_.ofstream_;
  bool& is_header_written = is_header_written_smart_factors_;

  // First, write header, but only once.
  if (!is_header_written) {
    output_stream << "#cur_kf_id,timestamp_kf,numSF,"
                  << "numValid,numDegenerate,numFarPoints,numOutliers,"
                  << "numCheirality,numNonInitialized,meanPixelError,"
                  << "maxPixelError,meanTrackLength,maxTrackLength,"
                  << "nrElementsInMatrix,nrZeroElementsInMatrix" << std::endl;
    is_header_written = true;
  }

  output_stream << output.cur_kf_id_ << "," << output.W_State_Blkf_.timestamp_
                << "," << output.debug_info_.numSF_ << ","
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
                << output.debug_info_.nrZeroElementsInMatrix_ << std::endl;
}

void BackendLogger::logBackendPimNavstates(const BackendOutput& output) {
  std::ofstream& output_stream = output_pim_navstates_csv_.ofstream_;
  bool& is_header_written = is_header_written_pim_navstates_;

  // First, write header, but only once.
  if (!is_header_written) {
    output_stream << "#timestamp_kf,x,y,z,qw,qx,qy,qz,vx,vy,vz" << std::endl;
    is_header_written = true;
  }

  const gtsam::Pose3& pose = output.debug_info_.navstate_k_.pose();
  const gtsam::Point3& position = pose.translation();
  const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();
  const gtsam::Velocity3& velocity = output.debug_info_.navstate_k_.velocity();

  output_stream << output.W_State_Blkf_.timestamp_ << "," << position.x() << ","
                << position.y() << "," << position.z() << "," << quaternion.w()
                << "," << quaternion.x() << "," << quaternion.y() << ","
                << quaternion.z() << "," << velocity.x() << "," << velocity.y()
                << "," << velocity.z() << std::endl;
}

void BackendLogger::logBackendTiming(const BackendOutput& output) {
  std::ofstream& output_stream = output_backend_timing_csv_.ofstream_;
  bool& is_header_written = is_header_written_backend_timing_;

  // First, write header, but only once.
  if (!is_header_written) {
    output_stream << "#cur_kf_id,factorsAndSlotsTime,preUpdateTime,"
                  << "updateTime,updateSlotTime,extraIterationsTime,"
                  << "linearizeTime,linearSolveTime,retractTime,"
                  << "linearizeMarginalizeTime,marginalizeTime" << std::endl;
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
                << output.debug_info_.marginalizeTime_ << std::endl;
}

void BackendLogger::logBackendFactorsStats(const BackendOutput& output) {
  std::ofstream& output_stream = output_backend_factors_stats_csv_.ofstream_;
  bool& is_header_written = is_header_written_backend_factors_stats_;

  // First, write header, but only once.
  if (!is_header_written) {
    output_stream << "#cur_kf_id,numAddedSmartF,numAddedImuF,numAddedNoMotionF,"
                  << "numAddedConstantF,numAddedBetweenStereoF,state_size,"
                  << "landmark_count" << std::endl;
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
                << output.state_.size() << "," << output.landmark_count_
                << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

MesherLogger::MesherLogger() : output_path_(FLAGS_output_path) {}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
VisualizerLogger::VisualizerLogger()
    : output_mesh_("output_mesh.ply"),
      output_landmarks_("output_landmarks.txt") {}

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
  bool& is_header_written = is_header_written_mesh_;

  CHECK(output_mesh_stream) << "Output File Mesh: error writing.";
  // Number of vertices in the mesh.
  int vertex_count = lmks.rows;
  // Number of faces in the mesh.
  int faces_count = std::round(mesh.rows / 4);
  // First, write header, but only once.
  if (!is_header_written || !log_accumulated_mesh) {
    output_mesh_stream << "ply\n"
                       << "format ascii 1.0\n"
                       << "comment Mesh from KIMERA VIO at timestamp "
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
      output_frontend_ransac_stereo_("output_frontend_ransac_stereo.csv"),
      output_frontend_img_path_(FLAGS_output_path + "/frontend_images/") {
  // Create output directories for images.
  boost::filesystem::create_directory(
      boost::filesystem::path(output_frontend_img_path_.c_str()));
  boost::filesystem::create_directory(boost::filesystem::path(
      (output_frontend_img_path_ + "monoFeatureTracksLeftImg").c_str()));
  boost::filesystem::create_directory(boost::filesystem::path(
      (output_frontend_img_path_ + "monoTrackingUnrectifiedImg").c_str()));
  boost::filesystem::create_directory(boost::filesystem::path(
      (output_frontend_img_path_ + "monoTrackingRectifiedImg").c_str()));
  boost::filesystem::create_directory(boost::filesystem::path(
      (output_frontend_img_path_ + "stereoMatchingUnrectifiedImg").c_str()));
  boost::filesystem::create_directory(boost::filesystem::path(
      (output_frontend_img_path_ + "stereoMatchingRectifiedImg").c_str()));
}

void FrontendLogger::logFrontendStats(
    const Timestamp& timestamp_lkf,
    const DebugTrackerInfo& tracker_info,
    const TrackerStatusSummary& tracker_summary,
    const size_t& nrKeypoints) {
  // We log Frontend results in csv format.
  std::ofstream& output_stream_stats = output_frontend_stats_.ofstream_;
  bool& is_header_written = is_header_written_frontend_stats_;

  if (!is_header_written) {
    output_stream_stats << "#timestamp_lkf,mono_status,stereo_status,"
                        << "nr_keypoints,nrDetectedFeatures,nrTrackerFeatures,"
                        << "nrMonoInliers,nrMonoPutatives,nrStereoInliers,"
                        << "nrStereoPutatives,monoRansacIters,"
                        << "stereoRansacIters,nrValidRKP,nrNoLeftRectRKP,"
                        << "nrNoRightRectRKP,nrNoDepthRKP,nrFailedArunRKP,"
                        << "featureDetectionTime,featureTrackingTime,"
                        << "monoRansacTime,stereoRansacTime,"
                        << "featureSelectionTime,extracted_corners,"
                        << "need_n_corners" << std::endl;
    is_header_written = true;
  }

  output_stream_stats
      << timestamp_lkf
      << ","
      // Mono status.
      << TrackerStatusSummary::asString(tracker_summary.kfTrackingStatus_mono_)
      << ","
      // Stereo status.
      << TrackerStatusSummary::asString(
             tracker_summary.kfTrackingStatus_stereo_)
      << ","
      // Nr of keypoints.
      << nrKeypoints
      << ","
      // Feature detection, tracking and ransac.
      << tracker_info.nrDetectedFeatures_ << ","
      << tracker_info.nrTrackerFeatures_ << "," << tracker_info.nrMonoInliers_
      << "," << tracker_info.nrMonoPutatives_ << ","
      << tracker_info.nrStereoInliers_ << "," << tracker_info.nrStereoPutatives_
      << "," << tracker_info.monoRansacIters_ << ","
      << tracker_info.stereoRansacIters_
      << ","
      // Performance of sparse-stereo-matching and ransac.
      << tracker_info.nrValidRKP_ << "," << tracker_info.nrNoLeftRectRKP_ << ","
      << tracker_info.nrNoRightRectRKP_ << "," << tracker_info.nrNoDepthRKP_
      << "," << tracker_info.nrFailedArunRKP_
      << ","
      // Info about timing.
      << tracker_info.featureDetectionTime_ << ","
      << tracker_info.featureTrackingTime_ << ","
      << tracker_info.monoRansacTime_ << "," << tracker_info.stereoRansacTime_
      << ","
      // Info about feature selector.
      << tracker_info.featureSelectionTime_ << ","
      << tracker_info.extracted_corners_ << "," << tracker_info.need_n_corners_
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
  bool& is_header_written = is_header_written_ransac_mono_;

  if (!is_header_written) {
    output_stream_mono << "#timestamp_lkf,x,y,z,qw,qx,qy,qz" << std::endl;
    output_stream_stereo << "#timestamp_lkf,x,y,z,qw,qx,qy,qz" << std::endl;
    is_header_written = true;
  }

  // Log relative mono poses; pose from previous keyframe to current keyframe,
  // in previous-keyframe coordinates. These are not cumulative trajectories.
  const gtsam::Point3& mono_tran = relative_pose_body_mono.translation();
  const gtsam::Quaternion& mono_quat =
      relative_pose_body_mono.rotation().toQuaternion();

  output_stream_mono << timestamp_lkf << "," << mono_tran.x() << ","
                     << mono_tran.y() << "," << mono_tran.z() << ","
                     << mono_quat.w() << "," << mono_quat.x() << ","
                     << mono_quat.y() << "," << mono_quat.z() << std::endl;

  // Log relative stereo poses; pose from previous keyframe to current keyframe,
  // in previous-keyframe coordinates. These are not cumulative trajectories.
  const gtsam::Point3& stereo_tran = relative_pose_body_stereo.translation();
  const gtsam::Quaternion& stereo_quat =
      relative_pose_body_stereo.rotation().toQuaternion();

  output_stream_stereo << timestamp_lkf << "," << stereo_tran.x() << ","
                       << stereo_tran.y() << "," << stereo_tran.z() << ","
                       << stereo_quat.w() << "," << stereo_quat.x() << ","
                       << stereo_quat.y() << "," << stereo_quat.z()
                       << std::endl;
}

void FrontendLogger::logFrontendImg(const FrameId& kf_id,
                                    const cv::Mat& img,
                                    const std::string& img_name_prepend,
                                    const std::string& dir_name,
                                    bool disp_img,
                                    bool save_img) {
  // We save the images to the output folder so that they can be visualized.

  std::string img_name = output_frontend_img_path_ + dir_name +
                         img_name_prepend + std::to_string(kf_id) + ".png";

  // Show image.
  if (disp_img) {
    cv::imshow(img_name_prepend, img);
    cv::waitKey(1);
  }

  // Write image to disk.
  if (save_img) {
    LOG(INFO) << "Writing image: " << img_name;
    cv::imwrite(img_name, img);
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
PipelineLogger::PipelineLogger()
    : output_pipeline_timing_("output_timingOverall.csv") {}

void PipelineLogger::logPipelineOverallTiming(
    const std::chrono::milliseconds& duration) {
  // Add header.
  std::ofstream& outputFile_timingOverall_ = output_pipeline_timing_.ofstream_;
  outputFile_timingOverall_ << "vio_overall_time [ms]" << std::endl;
  outputFile_timingOverall_ << duration.count();

  VIO::utils::Statistics::WriteAllSamplesToCsvFile(FLAGS_output_path + '/' +
                                                   "StatisticsVIO.csv");
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
LoopClosureDetectorLogger::LoopClosureDetectorLogger()
    : output_lcd_("output_lcd_result.csv"),
      output_traj_("traj_pgo.csv"),
      output_status_("output_lcd_status.csv"),
      ts_map_() {}

void LoopClosureDetectorLogger::logTimestampMap(
    const std::unordered_map<VIO::FrameId, VIO::Timestamp>& ts_map) {
  ts_map_ = ts_map;
}

void LoopClosureDetectorLogger::logLCDResult(const LcdOutput& lcd_output) {
  logLoopClosure(lcd_output);
  logOptimizedTraj(lcd_output);
}

void LoopClosureDetectorLogger::logLoopClosure(const LcdOutput& lcd_output) {
  // We log loop-closure results in csv format.
  std::ofstream& output_stream_lcd = output_lcd_.ofstream_;
  bool& is_header_written = is_header_written_lcd_;

  if (!is_header_written) {
    output_stream_lcd << "#timestamp_kf,timestamp_query,timestamp_match,isLoop,"
                      << "matchKfId,queryKfId,x,y,z,qw,qx,qy,qz" << std::endl;
    is_header_written = true;
  }

  const gtsam::Point3& rel_trans = lcd_output.relative_pose_.translation();
  const gtsam::Quaternion& rel_quat =
      lcd_output.relative_pose_.rotation().toQuaternion();

  output_stream_lcd << lcd_output.timestamp_ << ","
                    << lcd_output.timestamp_query_ << ","
                    << lcd_output.timestamp_match_ << ","
                    << lcd_output.is_loop_closure_ << ","
                    << lcd_output.id_match_ << "," << lcd_output.id_recent_
                    << "," << rel_trans.x() << "," << rel_trans.y() << ","
                    << rel_trans.z() << "," << rel_quat.w() << ","
                    << rel_quat.x() << "," << rel_quat.y() << ","
                    << rel_quat.z() << std::endl;
}

void LoopClosureDetectorLogger::logOptimizedTraj(const LcdOutput& lcd_output) {
  // We close and reopen log file to clear contents completely.
  output_traj_.closeAndOpenLogFile();
  // We log the full optimized trajectory in csv format.
  std::ofstream& output_stream_traj = output_traj_.ofstream_;

  // TODO(marcus): set the append to false on this one and overwrite EVERY TIME

  bool is_header_written = false;
  if (!is_header_written) {
    output_stream_traj << "#timestamp_kf,x,y,z,qw,qx,qy,qz" << std::endl;
    is_header_written = true;
  }

  const gtsam::Values& traj = lcd_output.states_;

  for (size_t i = 1; i < traj.size(); i++) {
    const gtsam::Pose3& pose = traj.at<gtsam::Pose3>(i);
    const gtsam::Point3& trans = pose.translation();
    const gtsam::Quaternion& quat = pose.rotation().toQuaternion();

    output_stream_traj << ts_map_.at(i) << "," << trans.x() << "," << trans.y()
                       << "," << trans.z() << "," << quat.w() << "," << quat.x()
                       << "," << quat.y() << "," << quat.z() << std::endl;
  }
}

void LoopClosureDetectorLogger::logDebugInfo(const LcdDebugInfo& debug_info) {
  // We log the loop-closure result of every key frame in csv format.
  std::ofstream& output_stream_status = output_status_.ofstream_;
  bool& is_header_written = is_header_written_status_;

  if (!is_header_written) {
    output_stream_status << "#timestamp_kf,lcd_status,query_id,match_id,"
                         << "mono_input_size,mono_inliers,mono_iters,"
                         << "stereo_input_size,stereo_inliers,stereo_iters,"
                         << "pgo_size,pgo_lc_count,pgo_lc_inliers" << std::endl;
    is_header_written = true;
  }

  output_stream_status << debug_info.timestamp_ << ","
                       << LoopResult::asString(debug_info.loop_result_.status_)
                       << "," << debug_info.loop_result_.query_id_ << ","
                       << debug_info.loop_result_.match_id_ << ","
                       << debug_info.mono_input_size_ << ","
                       << debug_info.mono_inliers_ << ","
                       << debug_info.mono_iter_ << ","
                       << debug_info.stereo_input_size_ << ","
                       << debug_info.stereo_inliers_ << ","
                       << debug_info.stereo_iter_ << "," << debug_info.pgo_size_
                       << "," << debug_info.pgo_lc_count_ << ","
                       << debug_info.pgo_lc_inliers_ << std::endl;
}

}  // namespace VIO
