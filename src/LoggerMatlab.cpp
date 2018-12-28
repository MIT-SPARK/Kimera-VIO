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
#include <memory>
#include <boost/foreach.hpp>

DEFINE_string(output_path, "./", "Path where to store VIO's log output.");

namespace VIO {

////////////////////////////////////////////////////////////////////////////////
LoggerMatlab::LoggerMatlab()
  : output_path_(FLAGS_output_path) {}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::openLogFiles(int i, const std::string& output_file_name,
                                bool open_file_in_append_mode) {
  // Store output data and debug info:
  if (i == 0 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output.txt":output_file_name),
                          outputFile_, open_file_in_append_mode);
  if (i == 1 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_posesVIO.txt":output_file_name),
                          outputFile_posesVIO_, open_file_in_append_mode);
  if (i == 1 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_posesVIO.csv" : output_file_name),
                          outputFile_posesVIO_csv_, open_file_in_append_mode);
  if (i == 2 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_posesGT.txt" : output_file_name),
                          outputFile_posesGT_, open_file_in_append_mode);
  if (i == 3 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_landmarks.txt" : output_file_name),
                          outputFile_landmarks_, open_file_in_append_mode);
  if (i == 4 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_normals.txt" : output_file_name),
                          outputFile_normals_, open_file_in_append_mode);
  if (i == 5 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_smartFactors.txt" : output_file_name),
                          outputFile_smartFactors_, open_file_in_append_mode);
  if (i == 6 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_timingVIO.txt" : output_file_name),
                          outputFile_timingVIO_, open_file_in_append_mode);
  if (i == 7 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_timingTracker.txt" : output_file_name),
                          outputFile_timingTracker_, open_file_in_append_mode);
  if (i == 8 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_statsTracker.txt" : output_file_name),
                          outputFile_statsTracker_, open_file_in_append_mode);
  if (i == 9 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_statsFactors.txt" : output_file_name),
                          outputFile_statsFactors_, open_file_in_append_mode);
  if (i == 10 || i == -1)
    UtilsOpenCV::OpenFile(output_path_ +
                          (output_file_name.empty()?
                            "/output_mesh.ply" : output_file_name),
                          outputFile_mesh_, open_file_in_append_mode);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::closeLogFiles(int i) {
  if (i == 0 || i == -1)
    outputFile_.close();
  if (i == 1 || i == -1)
    outputFile_posesVIO_.close();
  if (i == 1 || i == -1)
    outputFile_posesVIO_csv_.close();
  if (i == 2 || i == -1)
    outputFile_posesGT_.close();
  if (i == 3 || i == -1)
    outputFile_landmarks_.close();
  if (i == 4 || i == -1)
    outputFile_normals_.close();
  if (i == 5 || i == -1)
    outputFile_smartFactors_.close();
  if (i == 6 || i == -1)
    outputFile_timingVIO_.close();
  if (i == 7 || i == -1)
    outputFile_timingTracker_.close();
  if (i == 8 || i == -1)
    outputFile_statsTracker_.close();
  if (i == 9 || i == -1)
    outputFile_statsFactors_.close();
  if (i == 10 || i == -1)
    outputFile_mesh_.close();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logFrontendResults(const ETHDatasetParser& dataset,
                                      const StereoVisionFrontEnd& stereoTracker,
                                      const Timestamp timestamp_lkf,
                                      const Timestamp timestamp_k) {
  // If it's a keyframe, check pose estimate.
  bool isValid = stereoTracker.trackerStatusSummary_.kfTrackingStatus_mono_ != Tracker::INVALID;
  double relativeRotError,relativeTranError;
  // MONO ERROR
  boost::tie(relativeRotError,relativeTranError) = dataset.computePoseErrors(stereoTracker.getRelativePoseBodyMono(), isValid, timestamp_lkf, timestamp_k, true); // true = comparison up to scale
  int nrKeypoints = stereoTracker.stereoFrame_km1_->left_frame_.getNrValidKeypoints();
  outputFile_ << stereoTracker.trackerStatusSummary_.kfTrackingStatus_mono_ << " "
              <<  relativeRotError << " " << relativeTranError << " " << nrKeypoints << " ";
  // STEREO ERROR
  isValid = stereoTracker.trackerStatusSummary_.kfTrackingStatus_stereo_ != Tracker::INVALID;
  boost::tie(relativeRotError,relativeTranError) = dataset.computePoseErrors( stereoTracker.getRelativePoseBodyStereo(), isValid, timestamp_lkf, timestamp_k);
  outputFile_ << stereoTracker.trackerStatusSummary_.kfTrackingStatus_stereo_
              << " " <<  relativeRotError << " " << relativeTranError << " " << nrKeypoints << " ";
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logLandmarks(const VioBackEnd::PointsWithId& lmks) {
  // Absolute vio errors
  if (outputFile_landmarks_) {
    outputFile_landmarks_ << "Id" << "\t" << "x" << "\t" << "y" << "\t" << "z\n";
    for (const VioBackEnd::PointWithId& point: lmks) {
      outputFile_landmarks_ << point.first << "\t" << point.second.x()
                            << "\t" << point.second.y()
                            << "\t" << point.second.z() << "\n";
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
    outputFile_landmarks_ << "x" << "\t" << "y" << "\t" << "z" << std::endl;
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
    outputFile_normals_ << "x" << "\t" << "y" << "\t" << "z" << std::endl;
    for (const cv::Point3f& normal: normals) {
      outputFile_normals_ << normal.x << "\t"
                          << normal.y << "\t"
                          << normal.z << "\n";
    }
    outputFile_normals_.flush();
  } else {
    throw std::runtime_error("Output File Normals: error writing.");
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logMesh(const cv::Mat& lmks,
                           const cv::Mat& colors,
                           const cv::Mat& mesh,
                           const double& timestamp,
                           bool log_accumulated_mesh) {
  if (outputFile_mesh_) {
    // Number of vertices in the mesh.
    int vertex_count = lmks.rows;
    // Number of faces in the mesh.
    int faces_count = std::round(mesh.rows / 4);
    // First, write header, but only once.
    static bool is_header_written = false;
    if (!is_header_written || !log_accumulated_mesh) {
      outputFile_mesh_ << "ply\n"
                       << "format ascii 1.0\n"
                       << "comment Mesh for SPARK VIO at timestamp " << timestamp
                       << "\n"
                       << "element vertex " << vertex_count << "\n"
                       << "property float x\n"
                       << "property float y\n"
                       << "property float z\n"
                       << "property uchar red\n" // Start of vertex color.
                       << "property uchar green\n"
                       << "property uchar blue\n"
                       << "element face " << faces_count << "\n"
                       << "property list uchar int vertex_indices\n"
                       << "end_header\n";
      is_header_written = true;
    }

    // Second, log vertices.
    for (int i = 0; i < lmks.rows; i++) {
      outputFile_mesh_ << lmks.at<float>(i, 0) << " " // Log vertices x y z.
                       << lmks.at<float>(i, 1) << " "
                       << lmks.at<float>(i, 2) << " "
                       << int(colors.at<uint8_t>(i, 0)) << " " // Log vertices colors.
                       << int(colors.at<uint8_t>(i, 1)) << " "
                       << int(colors.at<uint8_t>(i, 2)) << " \n";
    }
    // Finally, log faces.
    for (int i = 0; i < faces_count; i++) {
      // Assumes the mesh is made of triangles
      int index = i * 4;
      outputFile_mesh_ << mesh.at<int32_t>(index) << " "
                       << mesh.at<int32_t>(index + 1) << " "
                       << mesh.at<int32_t>(index + 2) << " "
                       << mesh.at<int32_t>(index + 3) << " \n";
    }
    outputFile_mesh_ << std::endl;
  } else {
    throw std::runtime_error("Output File Mesh: error writing.");
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::logBackendResults(const ETHDatasetParser& dataset,
                                     const StereoVisionFrontEnd& stereoTracker,
                                     const std::shared_ptr<VioBackEnd>& vio,
                                     const Timestamp timestamp_lkf,
                                     const Timestamp timestamp_k,
                                     const size_t k){

  double vioRotError,vioTranError;
  gtsam::Pose3 W_Pose_Bkf_gt = (dataset.getGroundTruthState(timestamp_k)).pose;
  std::tie(vioRotError,vioTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(W_Pose_Bkf_gt, vio->W_Pose_Blkf_);
  std::cout << "vioRotError " << vioRotError << ", vioTranError " << vioTranError << std::endl;

  // Absolute vio errors
  outputFile_ << vio->cur_kf_id_ << " " <<  vioRotError << " " << vioTranError << " " << vio->landmark_count_ << " ";

  // RPY vio errors
  gtsam::Vector3 rpy_gt = W_Pose_Bkf_gt.rotation().rpy(); // such that R = Rot3::Ypr(y,p,r)
  gtsam::Vector3 rpy_vio = vio->W_Pose_Blkf_.rotation().rpy();
  outputFile_ << rpy_gt(0) << " " <<  rpy_gt(1) << " " << rpy_gt(2) << " "
              << rpy_vio(0) << " " <<  rpy_vio(1) << " " << rpy_vio(2)  << " ";

  // relative vio errors
  double relativeRotError,relativeTranError;
  gtsam::Pose3 Bprevkf_Pose_Bkf_vio = W_Pose_Bprevkf_vio_.between(vio->W_Pose_Blkf_);
  boost::tie(relativeRotError,relativeTranError) =
      dataset.computePoseErrors(Bprevkf_Pose_Bkf_vio, true, timestamp_lkf, timestamp_k); // always VALID = TRUE
  outputFile_ << relativeRotError << " " << relativeTranError << " ";

  // relative imu rotation errors
  double relativeRotError_imu_wrt_gt, relativeRotError_imu_wrt_5point;
  gtsam::Pose3 Bprevkf_Pose_Bkf_imuPreint( vio->debug_info_.imuR_lkf_kf, gtsam::Point3() );// rotation from imu preintegration, no translation
  boost::tie(relativeRotError_imu_wrt_gt,relativeTranError) =
      dataset.computePoseErrors(Bprevkf_Pose_Bkf_imuPreint, true, timestamp_lkf, timestamp_k); // always VALID = TRUE
  std::tie(relativeRotError_imu_wrt_5point,relativeTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(
        Bprevkf_Pose_Bkf_imuPreint, stereoTracker.getRelativePoseBodyMono());
  outputFile_ << relativeRotError_imu_wrt_gt << " " << relativeRotError_imu_wrt_5point << " ";

  // relative imu prediction errors
  gtsam::Pose3 Bprevkf_Pose_Bkf_imuPredict = W_Pose_Bprevkf_vio_.between(vio->debug_info_.navstate_k_.pose());
  boost::tie(relativeRotError,relativeTranError) =
      dataset.computePoseErrors(Bprevkf_Pose_Bkf_imuPredict, true, timestamp_lkf, timestamp_k); // always VALID = TRUE
  outputFile_ << relativeRotError << " " << relativeTranError << " ";

  // check consistency of stereo translation estimate
  gtsam::Vector3 Tstereo = stereoTracker.getRelativePoseBodyStereo().translation().vector();
  gtsam::Vector3 Tgt = dataset.getGroundTruthRelativePose(timestamp_lkf, timestamp_k).translation().vector();
  gtsam::Matrix3 infoMat = stereoTracker.trackerStatusSummary_.infoMatStereoTranslation_;
  outputFile_ << (Tstereo - Tgt).norm() << " " << (Tstereo - Tgt).transpose() * infoMat * (Tstereo - Tgt) << " " << std::endl;

  // debug smart factors:
  outputFile_smartFactors_ << vio->cur_kf_id_ << " " << k << " " << UtilsOpenCV::NsecToSec(timestamp_k) // keyframe id, frame id, timestamp
                           << " " << vio->debug_info_.numSF_ << " " << vio->debug_info_.numValid_
                           << " " << vio->debug_info_.numDegenerate_ << " " << vio->debug_info_.numFarPoints_
                           << " " << vio->debug_info_.numOutliers_ << " " << vio->debug_info_.numCheirality_
                           << " " << vio->debug_info_.meanPixelError_ << " " << vio->debug_info_.maxPixelError_
                           << " " << vio->debug_info_.meanTrackLength_ << " " << vio->debug_info_.maxTrackLength_
                           << " " << vio->debug_info_.nrElementsInMatrix_ << " " << vio->debug_info_.nrZeroElementsInMatrix_ <<  std::endl;

  // we log the camera since we will display camera poses in matlab
  gtsam::Pose3 W_Pose_camlkf_vio = vio->W_Pose_Blkf_.compose(vio->B_Pose_leftCam_);
  outputFile_posesVIO_ << vio->cur_kf_id_ << " " << W_Pose_camlkf_vio.translation().transpose() << " " <<
                          W_Pose_camlkf_vio.rotation().matrix().row(0) << " " <<
                          W_Pose_camlkf_vio.rotation().matrix().row(1) << " " <<
                          W_Pose_camlkf_vio.rotation().matrix().row(2) << " " <<
                          vio->W_Vel_Blkf_.transpose()               << " " <<
                          vio->imu_bias_lkf_.accelerometer().transpose() << " " <<
                          vio->imu_bias_lkf_.gyroscope().transpose() << std::endl;
  // We log the poses in csv format for later alignement and analysis.
  static bool is_header_written = false;
  if (!is_header_written) {
    outputFile_posesVIO_csv_
        << "timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz,"
           " bgx, bgy, bgz, bax, bay, baz" << std::endl;
    is_header_written = true;
  }
  outputFile_posesVIO_csv_
      //TODO Luca: is W_Vel_Blkf_ at timestamp_lkf or timestamp_kf?
      // I just want to log latest vio estimate and correct timestamp...
      << timestamp_lkf                                     << ", "
      << vio->W_Pose_Blkf_.translation().transpose().x()   << ", "
      << vio->W_Pose_Blkf_.translation().transpose().y()   << ", "
      << vio->W_Pose_Blkf_.translation().transpose().z()   << ", "
      << vio->W_Pose_Blkf_.rotation().quaternion()(1)      << ", " // q_x
      << vio->W_Pose_Blkf_.rotation().quaternion()(2)      << ", " // q_y
      << vio->W_Pose_Blkf_.rotation().quaternion()(3)      << ", " // q_z
      << vio->W_Pose_Blkf_.rotation().quaternion()(0)      << ", " // q_w
      << vio->W_Vel_Blkf_.transpose()(0)                   << ", "
      << vio->W_Vel_Blkf_.transpose()(1)                   << ", "
      << vio->W_Vel_Blkf_.transpose()(2)                   << ", "
      << vio->imu_bias_lkf_.gyroscope().transpose()(0)     << ", "
      << vio->imu_bias_lkf_.gyroscope().transpose()(1)     << ", "
      << vio->imu_bias_lkf_.gyroscope().transpose()(2)     << ", "
      << vio->imu_bias_lkf_.accelerometer().transpose()(0) << ", "
      << vio->imu_bias_lkf_.accelerometer().transpose()(1) << ", "
      << vio->imu_bias_lkf_.accelerometer().transpose()(2) << std::endl;

  // we log the camera since we will display camera poses in matlab
  gtsam::Pose3 W_Pose_camlkf_gt = W_Pose_Bkf_gt.compose(vio->B_Pose_leftCam_);
  Vector3 W_Vel_camlkf_gt = (dataset.getGroundTruthState(timestamp_k)).velocity;
  gtsam::imuBias::ConstantBias imu_bias_lkf_gt = (dataset.getGroundTruthState(timestamp_k)).imuBias;
  outputFile_posesGT_ << vio->cur_kf_id_ << " " << W_Pose_camlkf_gt.translation().transpose() << " " <<
                         W_Pose_camlkf_gt.rotation().matrix().row(0) << " " <<
                         W_Pose_camlkf_gt.rotation().matrix().row(1) << " " <<
                         W_Pose_camlkf_gt.rotation().matrix().row(2) << " " <<
                         W_Vel_camlkf_gt.transpose()               << " " <<
                         imu_bias_lkf_gt.accelerometer().transpose() << " " <<
                         imu_bias_lkf_gt.gyroscope().transpose() << std::endl;

  // log timing for benchmarking and performance profiling
  outputFile_timingVIO_ << vio->cur_kf_id_ << " " <<
                           vio->debug_info_.factorsAndSlotsTime_ << " " <<
                           vio->debug_info_.preUpdateTime_ << " " <<
                           vio->debug_info_.updateTime_ << " " <<
                           vio->debug_info_.updateSlotTime_ << " " <<
                           vio->debug_info_.extraIterationsTime_ << " " <<
                           vio->debug_info_.printTime_ << " " <<
                           timing_loadStereoFrame_ << " " <<
                           timing_processStereoFrame_ << " " <<
                           timing_featureSelection_ << " " <<
                           timing_vio_ << " " <<
                           vio->debug_info_.linearizeTime_ << " " <<
                           vio->debug_info_.linearSolveTime_ << " " <<
                           vio->debug_info_.retractTime_ << " " <<
                           vio->debug_info_.linearizeMarginalizeTime_ << " " <<
                           vio->debug_info_.marginalizeTime_ << " " <<
                           vio->debug_info_.imuPreintegrateTime_ << std::endl;

  outputFile_timingTracker_ << vio->cur_kf_id_ << " " <<
                               stereoTracker.tracker_.debugInfo_.featureDetectionTime_ << " " <<
                               stereoTracker.tracker_.debugInfo_.featureTrackingTime_ << " " <<
                               stereoTracker.tracker_.debugInfo_.monoRansacTime_ << " " <<
                               stereoTracker.tracker_.debugInfo_.stereoRansacTime_ << " " <<
                               stereoTracker.tracker_.debugInfo_.monoRansacIters_ << " " <<
                               stereoTracker.tracker_.debugInfo_.stereoRansacIters_ << " " <<
                               stereoTracker.tracker_.debugInfo_.featureSelectionTime_ << " " << std::endl;

  // log performance of tracker (currently we only log info at keyframes!!)
  outputFile_statsTracker_ << vio->cur_kf_id_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrDetectedFeatures_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrTrackerFeatures_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrMonoInliers_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrMonoPutatives_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrStereoInliers_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrStereoPutatives_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrValidRKP_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrNoLeftRectRKP_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrNoRightRectRKP_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrNoDepthRKP_ << " " <<
                              stereoTracker.tracker_.debugInfo_.nrFailedArunRKP_ << " " <<
                              stereoTracker.tracker_.debugInfo_.need_n_corners_  << " " <<
                              stereoTracker.tracker_.debugInfo_.extracted_corners_  << std::endl;

  // statistics about factors added to the graph
  outputFile_statsFactors_ << vio->cur_kf_id_ << " " <<
                              vio->debug_info_.numAddedSmartF_ << " " <<
                              vio->debug_info_.numAddedImuF_ << " " <<
                              vio->debug_info_.numAddedNoMotionF_ << " " <<
                              vio->debug_info_.numAddedConstantVelF_ << " " <<
                              vio->debug_info_.numAddedBetweenStereoF_ << " " <<
                              vio->state_.size() << " " << // current number of states
                              3 * std::min( double(vio->cur_kf_id_ + 1),
                                            vio->vio_params_.horizon_  / (stereoTracker.tracker_.trackerParams_.intra_keyframe_time_)  + 1) << std::endl; // expected nr of states

  std::cout << "data written to file" << std::endl;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::displayInitialStateVioInfo(
    const ETHDatasetParser& dataset,
    const std::shared_ptr<VIO::VioBackEnd>& vio,
    gtNavState initialStateGT,
    const ImuAccGyr& imu_accgyr,
    const Timestamp timestamp_k) const {
  initialStateGT.print("initialStateGT\n");
  gtsam::Vector3 rpy_gt = initialStateGT.pose.rotation().rpy(); // such that R = Rot3::Ypr(y,p,r)
  std::cout << "yaw= " << rpy_gt(2) << " pitch= " << rpy_gt(1) << " roll= "<< rpy_gt(0) << std::endl;
  Vector3 localGravity = initialStateGT.pose.rotation().inverse().matrix() * vio->vio_params_.n_gravity_;
  std::cout << "gravity in global frame: \n" << vio->vio_params_.n_gravity_ << std::endl;
  std::cout << "gravity in local frame: \n" << localGravity << std::endl;
  std::cout << "expected initial acc measurement (no bias correction): \n" << -localGravity  << std::endl;
  std::cout << "expected initial acc measurement: \n" << -localGravity + initialStateGT.imuBias.accelerometer()  << std::endl;
  std::cout << "actual initial acc measurement: \n" << imu_accgyr.block<3,1>(0,0) << std::endl;
  std::cout << "expected initial gyro measurement: \n" << initialStateGT.imuBias.gyroscope()  << std::endl;
  std::cout << "actual initial gyro measurement: \n" << imu_accgyr.block<3,1>(3,0) << std::endl;

  vio->print();

  double vioRotError,vioTranError;
  std::tie(vioRotError,vioTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(initialStateGT.pose, vio->W_Pose_Blkf_);
  if(vioRotError > 1e-4 || vioTranError > 1e-4)
    throw std::runtime_error("stereoVIOExample: wrong initialization (we currently initialize to ground truth)");

  // for comparison: gt bias:
  std::cout << " dataset.getGroundTruthState(timestamp_k): " << std::endl;
  dataset.getGroundTruthState(timestamp_k).print();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void LoggerMatlab::displayOverallTiming() const {
  std::cout << "----------- timing stats: -----------" << std::endl;
  std::cout << "timing_loadStereoFrame_: " << timing_loadStereoFrame_ << std::endl;
  std::cout << "timing_processStereoFrame_: " << timing_processStereoFrame_ << std::endl;
  std::cout << "timing_featureSelection_: " << timing_featureSelection_ << std::endl;
  std::cout << "timing_vio_: " << timing_vio_ << std::endl;
  std::cout << "timing_loggerFrontend_: " << timing_loggerFrontend_ << std::endl;
  std::cout << "timing_loggerBackend_: " << timing_loggerBackend_ << std::endl;
}

} // namespace VIO

