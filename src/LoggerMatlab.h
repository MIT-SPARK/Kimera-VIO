/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoggerMatlab.h
 * @brief  Logging information for matlab stats and visualizations
 * @author Luca Carlone
 */

#ifndef LoggerMatlab_H_
#define LoggerMatlab_H_

#include <memory>
#include <unordered_map>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <VioBackEnd.h>
#include "UtilsOpenCV.h"
#include "ETH_parser.h"
#include "StereoVisionFrontEnd.h"

namespace VIO {

///////////////////////////////////////////////////////////////////////////////////////
class LoggerMatlab
{
public:
  LoggerMatlab() {}

  //  class variables
  std::ofstream outputFile_;
  std::ofstream outputFile_posesVIO_;
  std::ofstream outputFile_posesGT_;
  std::ofstream outputFile_smartFactors_;
  std::ofstream outputFile_timingVIO_;
  std::ofstream outputFile_timingTracker_;
  std::ofstream outputFile_statsTracker_;
  std::ofstream outputFile_statsFactors_;

  gtsam::Pose3 W_Pose_Bprevkf_vio_;

  double timing_loadStereoFrame_,timing_processStereoFrame_,
  timing_featureSelection_,timing_vio_ ,timing_loggerBackend_ ,timing_loggerFrontend_;

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void openLogFiles(){
    // store output data and debug info:
    UtilsOpenCV::OpenFile("./output.txt",outputFile_);
    UtilsOpenCV::OpenFile("./output_posesVIO.txt",outputFile_posesVIO_);
    UtilsOpenCV::OpenFile("./output_posesGT.txt",outputFile_posesGT_);
    UtilsOpenCV::OpenFile("./output_smartFactors.txt",outputFile_smartFactors_);
    UtilsOpenCV::OpenFile("./output_timingVIO.txt",outputFile_timingVIO_);
    UtilsOpenCV::OpenFile("./output_timingTracker.txt",outputFile_timingTracker_);
    UtilsOpenCV::OpenFile("./output_statsTracker.txt",outputFile_statsTracker_);
    UtilsOpenCV::OpenFile("./output_statsFactors.txt",outputFile_statsFactors_);
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void closeLogFiles(){
    outputFile_.close();
    outputFile_posesVIO_.close();
    outputFile_posesGT_.close();
    outputFile_smartFactors_.close();
    outputFile_timingVIO_.close();
    outputFile_timingTracker_.close();
    outputFile_statsTracker_.close();
    outputFile_statsFactors_.close();
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logFrontendResults(const ETHDatasetParser& dataset,
      const StereoVisionFrontEnd& stereoTracker, const Timestamp timestamp_lkf, const Timestamp timestamp_k){
    // if it's a keyframe, check pose estimate
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
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logBackendResults(const ETHDatasetParser& dataset,
      const StereoVisionFrontEnd& stereoTracker, const boost::shared_ptr<VioBackEnd>& vio,
      const Timestamp timestamp_lkf, const Timestamp timestamp_k, const size_t k){

    double vioRotError,vioTranError;
    gtsam::Pose3 W_Pose_Bkf_gt = (dataset.getGroundTruthState(timestamp_k)).pose;
    std::tie(vioRotError,vioTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(W_Pose_Bkf_gt, vio->W_Pose_Blkf_);
    std::cout << "vioRotError " << vioRotError << ", vioTranError " << vioTranError << std::endl;
    
    // Absolute vio errors
    outputFile_ << vio->cur_id_ << " " <<  vioRotError << " " << vioTranError << " " << vio->landmark_count_ << " ";

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
    gtsam::Pose3 Bprevkf_Pose_Bkf_imuPreint( vio->debugInfo_.imuR_lkf_kf, gtsam::Point3() );// rotation from imu preintegration, no translation
    boost::tie(relativeRotError_imu_wrt_gt,relativeTranError) =
        dataset.computePoseErrors(Bprevkf_Pose_Bkf_imuPreint, true, timestamp_lkf, timestamp_k); // always VALID = TRUE
    std::tie(relativeRotError_imu_wrt_5point,relativeTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(
        Bprevkf_Pose_Bkf_imuPreint, stereoTracker.getRelativePoseBodyMono());
    outputFile_ << relativeRotError_imu_wrt_gt << " " << relativeRotError_imu_wrt_5point << " ";

    // relative imu prediction errors
    gtsam::Pose3 Bprevkf_Pose_Bkf_imuPredict = W_Pose_Bprevkf_vio_.between(vio->debugInfo_.navstate_k_.pose());
    boost::tie(relativeRotError,relativeTranError) =
        dataset.computePoseErrors(Bprevkf_Pose_Bkf_imuPredict, true, timestamp_lkf, timestamp_k); // always VALID = TRUE
    outputFile_ << relativeRotError << " " << relativeTranError << " ";

    // check consistency of stereo translation estimate
    gtsam::Vector3 Tstereo = stereoTracker.getRelativePoseBodyStereo().translation().vector();
    gtsam::Vector3 Tgt = dataset.getGroundTruthRelativePose(timestamp_lkf, timestamp_k).translation().vector();
    gtsam::Matrix3 infoMat = stereoTracker.trackerStatusSummary_.infoMatStereoTranslation_;
    outputFile_ << (Tstereo - Tgt).norm() << " " << (Tstereo - Tgt).transpose() * infoMat * (Tstereo - Tgt) << " " << std::endl;

    // debug smart factors:
    outputFile_smartFactors_ << vio->cur_id_ << " " << k << " " << UtilsOpenCV::NsecToSec(timestamp_k) // keyframe id, frame id, timestamp
    << " " << vio->debugInfo_.numSF_ << " " << vio->debugInfo_.numValid_
    << " " << vio->debugInfo_.numDegenerate_ << " " << vio->debugInfo_.numFarPoints_
    << " " << vio->debugInfo_.numOutliers_ << " " << vio->debugInfo_.numCheirality_
    << " " << vio->debugInfo_.meanPixelError_ << " " << vio->debugInfo_.maxPixelError_
    << " " << vio->debugInfo_.meanTrackLength_ << " " << vio->debugInfo_.maxTrackLength_
    << " " << vio->debugInfo_.nrElementsInMatrix_ << " " << vio->debugInfo_.nrZeroElementsInMatrix_ <<  std::endl;

    // we log the camera since we will display camera poses in matlab
    gtsam::Pose3 W_Pose_camlkf_vio = vio->W_Pose_Blkf_.compose(vio->B_Pose_leftCam_);
    outputFile_posesVIO_ << vio->cur_id_ << " " << W_Pose_camlkf_vio.translation().transpose() << " " <<
        W_Pose_camlkf_vio.rotation().matrix().row(0) << " " <<
        W_Pose_camlkf_vio.rotation().matrix().row(1) << " " <<
        W_Pose_camlkf_vio.rotation().matrix().row(2) << " " <<
        vio->W_Vel_Blkf_.transpose()               << " " <<
        vio->imu_bias_lkf_.accelerometer().transpose() << " " <<
        vio->imu_bias_lkf_.gyroscope().transpose() << std::endl;

    // we log the camera since we will display camera poses in matlab
    gtsam::Pose3 W_Pose_camlkf_gt = W_Pose_Bkf_gt.compose(vio->B_Pose_leftCam_);
    Vector3 W_Vel_camlkf_gt = (dataset.getGroundTruthState(timestamp_k)).velocity;
    gtsam::imuBias::ConstantBias imu_bias_lkf_gt = (dataset.getGroundTruthState(timestamp_k)).imuBias;
    outputFile_posesGT_ << vio->cur_id_ << " " << W_Pose_camlkf_gt.translation().transpose() << " " <<
        W_Pose_camlkf_gt.rotation().matrix().row(0) << " " <<
        W_Pose_camlkf_gt.rotation().matrix().row(1) << " " <<
        W_Pose_camlkf_gt.rotation().matrix().row(2) << " " <<
        W_Vel_camlkf_gt.transpose()               << " " <<
        imu_bias_lkf_gt.accelerometer().transpose() << " " <<
        imu_bias_lkf_gt.gyroscope().transpose() << std::endl;

    // log timing for benchmarking and performance profiling
    outputFile_timingVIO_ << vio->cur_id_ << " " <<
        vio->debugInfo_.factorsAndSlotsTime_ << " " <<
        vio->debugInfo_.preUpdateTime_ << " " <<
        vio->debugInfo_.updateTime_ << " " <<
        vio->debugInfo_.updateSlotTime_ << " " <<
        vio->debugInfo_.extraIterationsTime_ << " " <<
        vio->debugInfo_.printTime_ << " " <<
        timing_loadStereoFrame_ << " " <<
        timing_processStereoFrame_ << " " <<
        timing_featureSelection_ << " " <<
        timing_vio_ << " " <<
        vio->debugInfo_.linearizeTime_ << " " <<
        vio->debugInfo_.linearSolveTime_ << " " <<
        vio->debugInfo_.retractTime_ << " " <<
        vio->debugInfo_.linearizeMarginalizeTime_ << " " <<
        vio->debugInfo_.marginalizeTime_ << " " <<
		vio->debugInfo_.imuPreintegrateTime_ << std::endl;

    outputFile_timingTracker_ << vio->cur_id_ << " " <<
        stereoTracker.tracker_.debugInfo_.featureDetectionTime_ << " " <<
        stereoTracker.tracker_.debugInfo_.featureTrackingTime_ << " " <<
        stereoTracker.tracker_.debugInfo_.monoRansacTime_ << " " <<
        stereoTracker.tracker_.debugInfo_.stereoRansacTime_ << " " <<
        stereoTracker.tracker_.debugInfo_.monoRansacIters_ << " " <<
        stereoTracker.tracker_.debugInfo_.stereoRansacIters_ << " " <<
        stereoTracker.tracker_.debugInfo_.featureSelectionTime_ << " " << std::endl;

    // log performance of tracker (currently we only log info at keyframes!!)
    outputFile_statsTracker_ << vio->cur_id_ << " " <<
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
    outputFile_statsFactors_ << vio->cur_id_ << " " <<
        vio->debugInfo_.numAddedSmartF_ << " " <<
        vio->debugInfo_.numAddedImuF_ << " " <<
        vio->debugInfo_.numAddedNoMotionF_ << " " <<
        vio->debugInfo_.numAddedConstantVelF_ << " " <<
        vio->debugInfo_.numAddedBetweenStereoF_ << " " <<
        vio->state_.size() << " " << // current number of states
        3 * std::min( double(vio->cur_id_ + 1),
            vio->vioParams_.horizon_  / (stereoTracker.tracker_.trackerParams_.intra_keyframe_time_)  + 1) << std::endl; // expected nr of states

    std::cout << "data written to file" << std::endl;
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
   void displayInitialStateVioInfo(const ETHDatasetParser& dataset, const boost::shared_ptr<VIO::VioBackEnd>& vio,
       gtNavState initialStateGT, const ImuAccGyr& imu_accgyr, const Timestamp timestamp_k) const {
     initialStateGT.print("initialStateGT\n");
     gtsam::Vector3 rpy_gt = initialStateGT.pose.rotation().rpy(); // such that R = Rot3::Ypr(y,p,r)
     std::cout << "yaw= " << rpy_gt(2) << " pitch= " << rpy_gt(1) << " roll= "<< rpy_gt(0) << std::endl;
     Vector3 localGravity = initialStateGT.pose.rotation().inverse().matrix() * vio->vioParams_.n_gravity_;
     std::cout << "gravity in global frame: \n" << vio->vioParams_.n_gravity_ << std::endl;
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
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void displayOverallTiming() const {
    std::cout << "----------- timing stats: -----------" << std::endl;
    std::cout << "timing_loadStereoFrame_: " << timing_loadStereoFrame_ << std::endl;
    std::cout << "timing_processStereoFrame_: " << timing_processStereoFrame_ << std::endl;
    std::cout << "timing_featureSelection_: " << timing_featureSelection_ << std::endl;
    std::cout << "timing_vio_: " << timing_vio_ << std::endl;
    std::cout << "timing_loggerFrontend_: " << timing_loggerFrontend_ << std::endl;
    std::cout << "timing_loggerBackend_: " << timing_loggerBackend_ << std::endl;
  }
};

} // namespace VIO
#endif /* LoggerMatlab_H_ */

