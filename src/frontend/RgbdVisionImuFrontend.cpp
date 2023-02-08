/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdVisionImuFrontend.cpp
 * @brief  Class describing a Rgbd tracking Frontend
 * @author Marcus Abate
 */

#include <memory>

#include "kimera-vio/frontend/RgbdVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend.h"

DEFINE_bool(log_rgbd_matching_images,
            false,
            "Display/Save rgbd tracking rectified and unrectified images.");

namespace VIO {

RgbdVisionImuFrontend::RgbdVisionImuFrontend(
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const FrontendParams& frontend_params,
    const StereoCamera::ConstPtr& stereo_camera,
    DisplayQueue* display_queue,
    bool log_output)
    : VisionImuFrontend(imu_params, imu_initial_bias, display_queue, log_output),
    rgbd_frame_k_(nullptr),
    rgbd_frame_km1_(nullptr),
    rgbd_frame_lkf_(nullptr),
    keyframe_R_ref_frame_(gtsam::Rot3::Identity()),
    feature_detector_(nullptr),
    stereo_camera_(stereo_camera),
    frontend_params_(frontend_params) {
  CHECK(stereo_camera_);
  tracker_ = VIO::make_unique<Tracker>(frontend_params_, stereo_camera_->getOriginalLeftCamera(), display_queue);

  if (!frontend_params_.use_on_device_tracking_){

    feature_detector_ = VIO::make_unique<FeatureDetector>(
      frontend_params_.feature_detector_params_);
  }
  if (VLOG_IS_ON(1)) tracker_->tracker_params_.print();
  VLOG(8) <<"Body pose of the left Camera: " << stereo_camera_->getOriginalLeftCamera()->getBodyPoseCam() 
          << "  Body pose of the left Rectified Camera: " << stereo_camera_->getBodyPoseLeftCamRect();
}

RgbdVisionImuFrontend::~RgbdVisionImuFrontend() {
  LOG(INFO) << "RgbdVisionImuFrontend destructor called.";
}

RgbdFrontendOutput::UniquePtr RgbdVisionImuFrontend::bootstrapSpinRgbd(
    RgbdFrontendInputPayload::UniquePtr&& input) {
  CHECK(input);

  // Initialize members of the Frontend
  processFirstFrame(input->getRgbdFrame());

  // Initialization done, set state to nominal
  frontend_state_ = FrontendState::Nominal;

  // Create mostly invalid output
  CHECK(rgbd_frame_lkf_);
  CHECK(stereo_camera_);
  return VIO::make_unique<RgbdFrontendOutput>(rgbd_frame_lkf_->isKeyframe_,
                                              nullptr,
                                              TrackingStatus::DISABLED,
                                              getRelativePoseBodyRgbd(),
                                              stereo_camera_->getOriginalLeftCamera()->getBodyPoseCam(),
                                              *rgbd_frame_lkf_,
                                              nullptr,
                                              input->getImuAccGyrs(),
                                              cv::Mat(),
                                              getTrackerInfo());
}

RgbdFrontendOutput::UniquePtr RgbdVisionImuFrontend::nominalSpinRgbd(
    RgbdFrontendInputPayload::UniquePtr&& input) {
  // For timing
  utils::StatsCollector timing_stats_frame_rate(
      "VioFrontend RgbdFrame Rate [ms]");
  utils::StatsCollector timing_stats_keyframe_rate(
      "VioFrontend Keyframe Rate [ms]");
  auto start_time = utils::Timer::tic();

  const RgbdFrame& rgbd_frame_k = input->getRgbdFrame();
  const auto& k = rgbd_frame_k.id_;
  VLOG(1) << "------------------- Processing frame k = " << k
          << "--------------------";
  VLOG(8) <<"Body pose of the left Camera: " << stereo_camera_->getOriginalLeftCamera()->getBodyPoseCam() 
          << "  Body pose of the left Rectified Camera: " << stereo_camera_->getBodyPoseLeftCamRect();

  if (VLOG_IS_ON(10)) input->print();

  // For k > 1
  // The preintegration btw frames is needed for RANSAC.
  // But note that we are using interpolated "fake" values when doing the preint
  // egration!! Should we remove them??
  // Actually, currently does not integrate fake interpolated meas as it does
  // not take the last measurement into account (although it takes its stamp
  // into account!!!). 
  auto tic_full_preint = utils::Timer::tic();
  const ImuFrontend::PimPtr& pim = imu_frontend_->preintegrateImuMeasurements(
      input->getImuStamps(), input->getImuAccGyrs());
  CHECK(pim);
  auto full_preint_duration =
      utils::Timer::toc<std::chrono::microseconds>(tic_full_preint).count();

  VLOG_IF(1, full_preint_duration != 0.0)
      << "Current IMU Preintegration frequency: " << 10e6 / full_preint_duration
      << " Hz. (" << full_preint_duration << " us).";

  // On the left camera rectified!! pose w.r.t body
  const gtsam::Rot3 body_R_cam = 
      camera_->getBodyPoseCam().rotation();
  const gtsam::Rot3 cam_R_body = body_R_cam.inverse();
  gtsam::Rot3 imuLKF_R_K_imu = pim->deltaRij();

  VLOG(5) <<" Relative rotation of Curr IMU frame w.r.t IMU at LKF -> " << imuLKF_R_K_imu << "it's rpy rotation is -> "<< imuLKF_R_K_imu.rpy();
  gtsam::Rot3 camLrectLkf_R_camLrectK_imu = 
      cam_R_body * imuLKF_R_K_imu * body_R_cam;

  if (VLOG_IS_ON(10)) {
    body_R_cam.print("body_R_cam");
    camLrectLkf_R_camLrectK_imu.print("camLrectLkf_R_camLrectK_imu");
  }

  /////////////////////////////// TRACKING /////////////////////////////////////
  VLOG(10) << "Starting processFrame...";
  cv::Mat feature_tracks;
  StatusRgbdMeasurementsPtr status_rgbd_measurements = processFrame(
      rgbd_frame_k, camLrectLkf_R_camLrectK_imu, &feature_tracks);
  CHECK(!rgbd_frame_k_);  // We want a nullptr at the end of the processing.
  VLOG(10) << "Finished processRgbdFrame.";
  //////////////////////////////////////////////////////////////////////////////

  if (VLOG_IS_ON(5))
    RgbdVisionImuFrontend::printStatusRgbdMeasurements(*status_rgbd_measurements);

  if (rgbd_frame_km1_->isKeyframe()) {
    CHECK_EQ(rgbd_frame_lkf_->timestamp_, rgbd_frame_km1_->timestamp_);
    CHECK_EQ(rgbd_frame_lkf_->id_, rgbd_frame_km1_->id_);
    CHECK(!rgbd_frame_k_);
    CHECK(rgbd_frame_lkf_->isKeyframe());
    VLOG(1) << "Keyframe " << k
            << " with: " << status_rgbd_measurements->second.size()
            << " smart measurements";

    ////////////////// DEBUG INFO FOR FRONT-END ////////////////////////////////
    if (logger_) {
      logger_->logFrontendStats(rgbd_frame_lkf_->timestamp_,
                                getTrackerInfo(),
                                tracker_status_summary_,
                                rgbd_frame_km1_->intensity_img_->getNrValidKeypoints());
      // TODO(marcus): Last arg is usually stereo, need to refactor logger
      // to not require that.
      logger_->logFrontendRansac(rgbd_frame_lkf_->timestamp_,
                                 tracker_status_summary_.lkf_T_k_mono_,
                                 tracker_status_summary_.lkf_T_k_stereo_);
    }
    //////////////////////////////////////////////////////////////////////////////

    // Reset integration; the later the better.
    VLOG(10) << "Reset IMU preintegration with latest IMU bias.";
    imu_frontend_->resetIntegrationWithCachedBias();

    // Record keyframe rate timing
    timing_stats_keyframe_rate.AddSample(utils::Timer::toc(start_time).count());

    // Return the output of the Frontend for the others.
    VLOG(2) << "Frontend output is a keyframe: pushing to output callbacks.";
    return VIO::make_unique<RgbdFrontendOutput>(
        true,
        status_rgbd_measurements,
        tracker_status_summary_.kfTrackingStatus_stereo_,  // This is a stereo status only
        tracker_->tracker_params_.useStereoTracking_
            ? getRelativePoseBodyRgbd()
            : gtsam::Pose3::Identity(),
        stereo_camera_->getOriginalLeftCamera()->getBodyPoseCam(),
        *rgbd_frame_lkf_,  //! This is really the current keyframe in this if
        pim,
        input->getImuAccGyrs(),
        feature_tracks,
        getTrackerInfo());
  } else {
    // Record frame rate timing
    timing_stats_frame_rate.AddSample(utils::Timer::toc(start_time).count());

    VLOG(2) << "Frontend output is not a keyframe. Skipping output queue push.";
    return VIO::make_unique<RgbdFrontendOutput>(
        false,
        status_rgbd_measurements,
        TrackingStatus::INVALID,
        tracker_->tracker_params_.useStereoTracking_
            ? getRelativePoseBodyRgbd()
            : gtsam::Pose3::Identity(),
        stereo_camera_->getOriginalLeftCamera()->getBodyPoseCam(),
        *rgbd_frame_lkf_,  //! This is really the current keyframe in this if
        pim,
        input->getImuAccGyrs(),
        feature_tracks,
        getTrackerInfo());
  }
}

void RgbdVisionImuFrontend::processFirstFrame(const RgbdFrame& first_frame) {
  VLOG(2) << "Processing first rgbd frame \n";
  rgbd_frame_k_ = std::make_shared<RgbdFrame>(first_frame);
  rgbd_frame_k_->setIsKeyframe(true);
  last_keyframe_timestamp_ = rgbd_frame_k_->timestamp_;

  // CHECK_EQ(rgbd_frame_k_->intensity_img_->keypoints_.size(), 0)
  //     << "Keypoints already present in first frame: please do not extract"
  //        " keypoints manually";

  if(!frontend_params_.use_on_device_tracking_){
    CHECK(feature_detector_);
    feature_detector_->featureDetection(rgbd_frame_k_.get());
  }

  // Undistort keypoints:
  stereo_camera_->undistortRectifyLeftKeypoints(rgbd_frame_k_->intensity_img_->keypoints_,
                                   &rgbd_frame_k_->intensity_img_->keypoints_undistorted_);

  // camera_->undistortKeypoints(rgbd_frame_k_->intensity_img_->keypoints_,
  //                                  &rgbd_frame_k_->intensity_img_->keypoints_undistorted_);

  // TODO(marcus): get 3d points if possible?
  rgbd_frame_km1_ = rgbd_frame_k_;
  rgbd_frame_lkf_ = rgbd_frame_k_;
  rgbd_frame_k_.reset();
  ++frame_count_;

  imu_frontend_->resetIntegrationWithCachedBias();
}

StatusRgbdMeasurementsPtr RgbdVisionImuFrontend::processFrame(
    const RgbdFrame& cur_frame,
    const gtsam::Rot3& keyframe_R_cur_frame,
    cv::Mat* feature_tracks) {
  CHECK(tracker_);
  VLOG(2) << "===================================================\n"
          << "Frame number: " << frame_count_ << " at time "
          << cur_frame.timestamp_ << " empirical framerate (sec): "
          << UtilsNumerical::NsecToSec(cur_frame.timestamp_ -
                                       rgbdFrame_km1_->timestamp_)
          << " (timestamp diff: "
          << cur_frame.timestamp_ - rgbdFrame_km1_->timestamp_ << ")";
  auto start_time = utils::Timer::tic();

  // TODO this copies the rgbd frame!!
  rgbdFrame_k_ = std::make_shared<RgbdFrame>(cur_frame);
  Frame* left_frame_k = &rgbdFrame_k_->intensity_img_;
 VLOG(5) << "Features in the previous frame -> " << rgbdFrame_km1_->intensity_img_.keypoints_.size();

  /////////////////////// MONO TRACKING ////////////////////////////////////////
  VLOG(2) << "Starting feature tracking...";
  // We need to use the frame to frame rotation.
  gtsam::Rot3 ref_frame_R_cur_frame =
      keyframe_R_ref_frame_.inverse().compose(keyframe_R_cur_frame);

  //TODO(saching): Currently when on-device feature tracking is used, 
  // we are skipping the marking of old features to be not used at the time. 
  if (!frontend_params_.use_on_device_tracking_) {
    tracker_->featureTracking(&rgbdFrame_km1_->intensity_img_,
                            left_frame_k,
                            ref_frame_R_cur_frame);
  }

  if (feature_tracks) {
    // TODO(Toni): these feature tracks are not outlier rejected...
    // TODO(Toni): this image should already be computed and inside the
    // display_queue
    // if it is sent to the tracker.
    *feature_tracks = tracker_->getTrackerImage(rgbdFrame_lkf_->intensity_img_,
                                               rgbdFrame_k_->intensity_img_);
    VLOG(5) << "getting feature_tracks image w.r.t Key frame ............... " ;
  }

  VLOG(2) << "Finished feature tracking.";
  //////////////////////////////////////////////////////////////////////////////

  // Not tracking at all in this phase.
  tracker_status_summary_.kfTrackingStatus_mono_ = TrackingStatus::INVALID;
  tracker_status_summary_.kfTrackingStatus_stereo_ = TrackingStatus::INVALID;

  // This will be the info we actually care about
  RgbdMeasurements smart_rgbd_measurements;

  const bool max_time_elapsed =
      rgbdFrame_k_->timestamp_ - last_keyframe_timestamp_ >=
      tracker_->tracker_params_.intra_keyframe_time_ns_;
  const size_t& nr_valid_features = left_frame_k->getNrValidKeypoints();
  const bool nr_features_low =
      nr_valid_features <= tracker_->tracker_params_.min_number_features_;

  // Also if the user requires the keyframe to be enforced
  LOG_IF(WARNING, rgbdFrame_k_->isKeyframe()) << "User enforced keyframe!";
  // If max time elaspsed and not able to track feature -> create new keyframe
  if (max_time_elapsed || nr_features_low || rgbdFrame_k_->isKeyframe()) {
    ++keyframe_count_;  // mainly for debugging

    VLOG(2) << "Keyframe after [s]: "
            << UtilsNumerical::NsecToSec(rgbdFrame_k_->timestamp_ -
                                         last_keyframe_timestamp_);

    VLOG_IF(2, max_time_elapsed) << "Keyframe reason: max time elapsed.";
    VLOG_IF(2, nr_features_low)
        << "Keyframe reason: low nr of features (" << nr_valid_features << " < "
        << tracker_->tracker_params_.min_number_features_ << ").";

    double sparse_rgbd_time = 0;
    if (tracker_->tracker_params_.useRANSAC_) {
      // MONO geometric outlier rejection
      TrackingStatusPose status_pose_mono;
      Frame* left_frame_lkf = &rgbdFrame_lkf_->intensity_img_;
      outlierRejectionMono(keyframe_R_cur_frame,
                           left_frame_lkf,
                           left_frame_k,
                           &status_pose_mono);
      tracker_status_summary_.kfTrackingStatus_mono_ = status_pose_mono.first;
      if (status_pose_mono.first == TrackingStatus::VALID) {
        tracker_status_summary_.lkf_T_k_mono_ = status_pose_mono.second;
      }

      // RGBD geometric outlier rejection
      // get 3D points via rgbd
      start_time = utils::Timer::tic();
      // Undistort keypoints:
      // Needed to calculate 3D points since it adds KeypointStatus
      stereo_camera_->undistortRectifyLeftKeypoints(rgbd_frame_k_->intensity_img_->keypoints_,
                                      &rgbd_frame_k_->intensity_img_->keypoints_undistorted_);

      // camera_->undistortKeypoints(rgbdFrame_k_->intensity_img_->keypoints_,
      //                                &rgbdFrame_k_->intensity_img_->keypoints_undistorted_);

      rgbdFrame_k_->calculate3dKeypoints();
      sparse_rgbd_time = utils::Timer::toc(start_time).count();

      TrackingStatusPose status_pose_rgbd;
      if (tracker_->tracker_params_.useStereoTracking_) {
        outlierRejectionStereo(keyframe_R_cur_frame,
                               rgbdFrame_lkf_,
                               rgbdFrame_k_,
                               &status_pose_stereo);
        tracker_status_summary_.kfTrackingStatus_stereo_ =
            status_pose_rgbd.first;

        if (status_pose_rgbd.first == TrackingStatus::VALID) {
          tracker_status_summary_.lkf_T_k_stereo_ = status_pose_rgbd.second;
        }
      } else {
        status_pose_rgbd.first = TrackingStatus::INVALID;
        status_pose_rgbd.second = gtsam::Pose3::Identity();
        tracker_status_summary_.kfTrackingStatus_stereo_ =
            TrackingStatus::INVALID;
      }
    } else {
      tracker_status_summary_.kfTrackingStatus_mono_ =
          TrackingStatus::DISABLED;
      tracker_status_summary_.kfTrackingStatus_stereo_ =
          TrackingStatus::DISABLED;
    }

    if (VLOG_IS_ON(2)) {
      printTrackingStatus(tracker_status_summary_.kfTrackingStatus_mono_,
                          "mono");
      printTrackingStatus(tracker_status_summary_.kfTrackingStatus_stereo_,
                          "rgbd");
    }

    // If its been long enough, make it a keyframe
    last_keyframe_timestamp_ = rgbdFrame_k_->timestamp_;
    rgbdFrame_k_->setIsKeyframe(true);

    // Perform feature detection (note: this must be after RANSAC,
    // since if we discard more features, we need to extract more)
    if (!frontend_params_.use_on_device_tracking_){
      CHECK(feature_detector_);
      feature_detector_->featureDetection(left_frame_k);
    }

    // Log images if needed.
    if (logger_ &&
        (FLAGS_visualize_frontend_images || FLAGS_save_frontend_images)) {
      if (FLAGS_log_feature_tracks) sendFeatureTracksToLogger();
      if (FLAGS_log_mono_tracking_images) sendStereoMatchesToLogger();
      if (FLAGS_log_stereo_matching_images) sendMonoTrackingToLogger();
    }
    if (display_queue_ && FLAGS_visualize_feature_tracks) {
      displayImage(rgbdFrame_k_->timestamp_,
                   "feature_tracks",
                   tracker_->getTrackerImage(rgbdFrame_lkf_->left_frame_,
                                            rgbdFrame_k_->left_frame_),
                   display_queue_);
    }

    // Populate statistics.
    // TODO(saching): Add statistics for the current frame.
    // rgbdFrame_k_->checkStatusRightKeypoints(&tracker_->debug_info_);

    // Move on.
    rgbdFrame_lkf_ = rgbdFrame_k_;

    // Get relevant info for keyframe.
    start_time = utils::Timer::tic();
    getSmartRgbdMeasurements(rgbdFrame_k_, &smart_rgbd_measurements);
    double get_smart_rgbd_meas_time = utils::Timer::toc(start_time).count();

    VLOG(2) << "timeSparseRgbd: " << sparse_rgbd_time << '\n'
            << "timeGetMeasurements: " << get_smart_rgbd_meas_time;
  } else {
    CHECK_EQ(smart_rgbd_measurements.size(), 0u);
    rgbdFrame_k_->setIsKeyframe(false);
  }

  // Update keyframe to reference frame for next iteration.
  if (rgbdFrame_k_->isKeyframe()) {
    // Reset relative rotation if we have a keyframe.
    keyframe_R_ref_frame_ = gtsam::Rot3::Identity();
  } else {
    // Update rotation from keyframe to next iteration reference frame (aka
    // cur_frame in current iteration).
    keyframe_R_ref_frame_ = keyframe_R_cur_frame;
  }
  VLOG(4) <<" Relative pose of Curr Rgbd frame w.r.t body frame in Rgbd process -> " << getRelativePoseBodyRgbd();

  // Reset frames.
  rgbdFrame_km1_ = rgbdFrame_k_;
  rgbdFrame_k_.reset();
  ++frame_count_;
  return std::make_shared<StatusRgbdMeasurements>(
      std::make_pair(tracker_status_summary_,
                     // TODO(Toni): please, fix this, don't use std::pair...
                     // copies, manyyyy copies: actually thousands of copies...
                     smart_rgbd_measurements));
}

// TODO(marcus): for convenience mono measurements are gtsam::StereoPoint2
// hence this is near identical to getSmartStereoMeasurements
// but want to switch to gtsam::Point2
// TODO(saching): Check if gtsam::StereoPoint2 makes sense for us. 
void RgbdVisionImuFrontend::getSmartRgbdMeasurements(
    const RgbdFrame::Ptr& rgbdFrame_kf, RgbdMeasurements* smart_rgbd_measurements) {
  // TODO(marcus): convert to point2 when ready!
  CHECK_NOTNULL(smart_rgbd_measurements);
  rgbdFrame_kf->checkRgbdFrame();

  // depth = fx * baseline / disparity (should be fx = focal * sensorsize)
  double fx_b =
      stereo_camera_->getStereoCalib()->fx() * stereo_camera_->getBaseline()
  VLOG(8) << "Fx from stereo Camera: " << stereo_camera_->getStereoCalib()->fx() 
          << "Fx from Left Camera: " << stereo_camera_->getOriginalLeftCamera()->fx()
          << "Baseline is -> " stereo_camera_->getBaseline();

  const LandmarkIds& landmarkId_kf = rgbdFrame_kf->intensity_img_->landmarks_;
  const StatusKeypointsCV& keypoints_undistorted =
      rgbdFrame_kf->intensity_img_->keypoints_undistorted_;
  const std::vector<gtsam::Vector3>& keypoints_3d = rgbdFrame_kf->keypoints_3d_;
  // Pack information in landmark structure.
  smart_rgbd_measurements->clear();
  smart_rgbd_measurements->reserve(landmarkId_kf.size());
  for (size_t i = 0; i < landmarkId_kf.size(); ++i) {
    if (landmarkId_kf.at(i) == -1) {
      continue;  // skip invalid points
    }

    // TODO implicit conversion float to double increases floating-point
    // precision! Using auto instead of double for possible future proofing.
    const auto& uL = keypoints_undistorted.at(i).second.x;
    const auto& v = keypoints_undistorted.at(i).second.y;
    
    // Initialize to missing pixel information.
    double uR = std::numeric_limits<double>::quiet_NaN();

    if (!tracker_->tracker_params_.useStereoTracking_) {
      LOG_EVERY_N(WARNING, 10) << "Dropping stereo information! (set "
                                  "useStereoTracking_ = true to use it)";
    }

    if (tracker_->tracker_params_.useStereoTracking_ &&
        keypoints_undistorted.at(i).first == KeypointStatus::VALID) {
      // TODO implicit conversion float to double increases floating-point
      // precision!
      uR = uL - fx_b / keypoints_3d[i](2); // Taking only the z axis value since thats what depth map represents
    }
    smart_rgbd_measurements->push_back(
        std::make_pair(landmarkId_kf[i], gtsam::StereoPoint2(uL, uR, v)));
  }
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - MONO
// RANSAC
gtsam::Pose3 RgbdVisionImuFrontend::getRelativePoseBodyMono() const {
  // lkfBody_T_kBody = lkfBody_T_lkfCamera *  lkfCamera_T_kCamera_ *
  // kCamera_T_kBody = body_Pose_cam_ * lkf_T_k_mono_ * body_Pose_cam_^-1
  gtsam::Pose3 body_Pose_cam_ = stereo_camera_->getOriginalLeftCamera()->getBodyPoseCam();// of the left camera!!
  return body_Pose_cam_ * tracker_status_summary_.lkf_T_k_mono_ *
         body_Pose_cam_.inverse();
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - STEREO
// RANSAC
gtsam::Pose3 RgbdVisionImuFrontend::getRelativePoseBodyRgbd() const {
  gtsam::Pose3 body_Pose_cam_ = stereo_camera_->getOriginalLeftCamera()->getBodyPoseCam();  // of the left camera!!
  return body_Pose_cam_ * tracker_status_summary_.lkf_T_k_stereo_ *
         body_Pose_cam_.inverse();
}

void RgbdVisionImuFrontend::printStatusRgbdMeasurements(
    const StatusRgbdMeasurements& status_rgbd_measurements) {
  LOG(INFO) << "SmartRgbdMeasurements with status: ";
  printTrackingStatus(status_rgbd_measurements.first.kfTrackingStatus_mono_,
                      "mono");
  printTrackingStatus(status_rgbd_measurements.first.kfTrackingStatus_stereo_,
                      "stereo");
  LOG(INFO) << " stereo points:";
  const RgbdMeasurements& rgbd_measurements = status_rgbd_measurements.second;
  for (const auto& meas : rgbd_measurements) {
    std::cout << " " << meas.second << " ";
  }
  std::cout << std::endl;
} 

}  // namespace VIO
