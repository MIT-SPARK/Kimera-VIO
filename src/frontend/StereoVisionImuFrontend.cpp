/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionImuFrontend.cpp
 * @brief  Class describing a stereo tracker
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/frontend/StereoVisionImuFrontend.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Rot3.h>

#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

DECLARE_bool(do_fine_imu_camera_temporal_sync);

namespace VIO {

StereoVisionImuFrontend::StereoVisionImuFrontend(
    const FrontendParams& frontend_params,
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const StereoCamera::ConstPtr& stereo_camera,
    DisplayQueue* display_queue,
    bool log_output,
    boost::optional<OdometryParams> odom_params)
    : VisionImuFrontend(frontend_params,
                        imu_params,
                        imu_initial_bias,
                        display_queue,
                        log_output,
                        odom_params),
      stereoFrame_k_(nullptr),
      stereoFrame_km1_(nullptr),
      stereoFrame_lkf_(nullptr),
      keyframe_R_ref_frame_(gtsam::Rot3()),
      feature_detector_(nullptr),
      stereo_camera_(stereo_camera),
      stereo_matcher_(stereo_camera, frontend_params.stereo_matching_params_),
      output_images_path_("./outputImages/") {
  CHECK(stereo_camera_);

  feature_detector_ = VIO::make_unique<FeatureDetector>(
      frontend_params.feature_detector_params_);

  tracker_ = VIO::make_unique<Tracker>(frontend_params_.tracker_params_,
                                       stereo_camera_->getOriginalLeftCamera(),
                                       display_queue);

  if (VLOG_IS_ON(1)) tracker_->tracker_params_.print();
}

StereoVisionImuFrontend::~StereoVisionImuFrontend() {
  LOG(INFO) << "StereoVisionImuFrontend destructor called.";
}

StereoFrontendOutput::UniquePtr StereoVisionImuFrontend::bootstrapSpinStereo(
    StereoFrontendInputPayload::UniquePtr&& input) {
  // Initialize members of the Frontend
  processFirstStereoFrame(input->getStereoFrame());

  // Initialization done, set state to nominal
  frontend_state_ = FLAGS_do_fine_imu_camera_temporal_sync
                        ? FrontendState::InitialTimeAlignment
                        : FrontendState::Nominal;

  if (!FLAGS_do_fine_imu_camera_temporal_sync && odom_params_) {
    // we assume that the first frame is hardcoded to be a keyframe.
    // it's okay if world_NavState_odom_ is boost::none (it gets cached later)
    cacheExternalOdometry(input.get());
  }

  // Create mostly invalid output, to send the imu_acc_gyrs to the Backend.
  CHECK(stereoFrame_lkf_);

  if (FLAGS_do_fine_imu_camera_temporal_sync) {
    return nullptr;  // skip adding a frame to all downstream modules
  }

  return VIO::make_unique<StereoFrontendOutput>(
      stereoFrame_lkf_->isKeyframe(),
      nullptr,
      stereo_camera_->getBodyPoseLeftCamRect(),
      stereo_camera_->getBodyPoseRightCamRect(),
      *stereoFrame_lkf_,
      nullptr,
      input->getImuAccGyrs(),
      cv::Mat(),
      getTrackerInfo());
}

StereoFrontendOutput::UniquePtr StereoVisionImuFrontend::nominalSpinStereo(
    StereoFrontendInputPayload::UniquePtr&& input) {
  // For timing
  utils::StatsCollector timing_stats_frame_rate("VioFrontend Frame Rate [ms]");
  utils::StatsCollector timing_stats_keyframe_rate(
      "VioFrontend Keyframe Rate [ms]");
  auto start_time = utils::Timer::tic();

  // Get stereo info
  const StereoFrame& stereoFrame_k = input->getStereoFrame();
  const auto& k = stereoFrame_k.id_;
  VLOG(1) << "------------------- Processing frame k = " << k
          << "--------------------";

  ////////////////////////////// PROCESS IMU DATA //////////////////////////////

  // Print IMU data.
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
  // Don't add them because they are confusing
  // utils::StatsCollector stats_full_preint("IMU Preintegration Timing [us]");
  // stats_full_preint.AddSample(full_preint_duration);
  VLOG_IF(1, full_preint_duration != 0.0)
      << "Current IMU Preintegration frequency: " << 10e6 / full_preint_duration
      << " Hz. (" << full_preint_duration << " us).";

  // On the left camera rectified!!
  const gtsam::Rot3 body_Rot_cam =
      stereo_camera_->getBodyPoseLeftCamRect().rotation();
  const gtsam::Rot3 cam_Rot_body = body_Rot_cam.inverse();

  // Relative rotation of the left cam rectified from the last keyframe to the
  // curr frame. pim.deltaRij() corresponds to bodyLkf_R_bodyK_imu
  gtsam::Rot3 camLrectLkf_R_camLrectK_imu =
      cam_Rot_body * pim->deltaRij() * body_Rot_cam;

  if (VLOG_IS_ON(10)) {
    body_Rot_cam.print("Body_Rot_cam");
    camLrectLkf_R_camLrectK_imu.print("calLrectLkf_R_camLrectK_imu");
  }
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////////////// TRACKING /////////////////////////////////////
  // Main function for tracking.
  // Rotation used in 1 and 2 point ransac.
  VLOG(10) << "Starting processStereoFrame...";
  cv::Mat feature_tracks;
  StatusStereoMeasurementsPtr status_stereo_measurements = processStereoFrame(
      stereoFrame_k, camLrectLkf_R_camLrectK_imu, &feature_tracks);

  CHECK(!stereoFrame_k_);  // processStereoFrame is setting this to nullptr!!!
  VLOG(10) << "Finished processStereoFrame.";
  //////////////////////////////////////////////////////////////////////////////

  if (VLOG_IS_ON(5))
    StereoVisionImuFrontend::printStatusStereoMeasurements(
        *status_stereo_measurements);

  if (stereoFrame_km1_->isKeyframe()) {
    // We got a keyframe!
    CHECK_EQ(stereoFrame_lkf_->timestamp_, stereoFrame_km1_->timestamp_);
    CHECK_EQ(stereoFrame_lkf_->id_, stereoFrame_km1_->id_);
    CHECK(!stereoFrame_k_);
    CHECK(stereoFrame_lkf_->isKeyframe());
    VLOG(1) << "Keyframe " << k
            << " with: " << status_stereo_measurements->second.size()
            << " smart measurements";

    ////////////////// DEBUG INFO FOR FRONT-END ////////////////////////////////
    if (logger_) {
      logger_->logFrontendStats(
          stereoFrame_lkf_->timestamp_,
          getTrackerInfo(),
          tracker_status_summary_,
          stereoFrame_km1_->left_frame_.getNrValidKeypoints());
      // Logger needs information in camera frame for evaluation
      logger_->logFrontendRansac(stereoFrame_lkf_->timestamp_,
                                 tracker_status_summary_.lkf_T_k_mono_,
                                 tracker_status_summary_.lkf_T_k_stereo_);
    }
    ////////////////////////////////////////////////////////////////////////////

    // Reset integration the later the better so that we give to the Backend
    // the most time possible to update the IMU bias.
    VLOG(10) << "Reset IMU preintegration with latest IMU bias.";
    imu_frontend_->resetIntegrationWithCachedBias();

    // Record keyframe rate timing
    timing_stats_keyframe_rate.AddSample(utils::Timer::toc(start_time).count());

    // Return the output of the Frontend for the others.
    // We have a keyframe, so We fill stereo_frame_lkf_ with the newest keyframe
    VLOG(2) << "Frontend output is a keyframe: pushing to output callbacks.";
    return VIO::make_unique<StereoFrontendOutput>(
        frontend_state_ == FrontendState::Nominal,
        status_stereo_measurements,
        stereo_camera_->getBodyPoseLeftCamRect(),
        stereo_camera_->getBodyPoseRightCamRect(),
        *stereoFrame_lkf_,
        pim,
        input->getImuAccGyrs(),
        feature_tracks,
        getTrackerInfo(),
        getExternalOdometryRelativeBodyPose(input.get()),
        getExternalOdometryWorldVelocity(input.get()));
  } else {
    // Record frame rate timing
    timing_stats_frame_rate.AddSample(utils::Timer::toc(start_time).count());

    // TODO(nathan) unify returning output packets
    // We don't have a keyframe, so instead we forward the newest frame in this
    // packet for use in the temporal calibration (if enabled)
    VLOG(2) << "Frontend output is not a keyframe. Skipping output queue push.";
    return VIO::make_unique<StereoFrontendOutput>(
        false,
        status_stereo_measurements,
        stereo_camera_->getBodyPoseLeftCamRect(),
        stereo_camera_->getBodyPoseRightCamRect(),
        *stereoFrame_km1_,
        pim,
        input->getImuAccGyrs(),
        feature_tracks,
        getTrackerInfo());
  }
}

/* -------------------------------------------------------------------------- */
// TODO this can be greatly improved, but we need to get rid of global variables
// stereoFrame_km1_, stereoFrame_lkf_, stereoFrame_k_, etc...
void StereoVisionImuFrontend::processFirstStereoFrame(
    const StereoFrame& firstFrame) {
  VLOG(2) << "Processing first stereo frame \n";
  stereoFrame_k_ =
      std::make_shared<StereoFrame>(firstFrame);  // TODO this can be optimized!
  stereoFrame_k_->setIsKeyframe(true);
  last_keyframe_timestamp_ = stereoFrame_k_->timestamp_;

  // Tracking is based on left frame.
  Frame* left_frame = &stereoFrame_k_->left_frame_;
  CHECK_EQ(left_frame->keypoints_.size(), 0)
      << "Keypoints already present in first frame: please do not extract"
         " keypoints manually";

  // Perform feature detection.
  CHECK(feature_detector_);
  feature_detector_->featureDetection(left_frame, stereo_camera_->getR1());

  // Get 3D points via stereo.
  VLOG(2) << "calling sparseStereoReconstruction \n";
  stereo_matcher_.sparseStereoReconstruction(stereoFrame_k_.get());

  // Prepare for next iteration.
  stereoFrame_km1_ = stereoFrame_k_;
  stereoFrame_lkf_ = stereoFrame_k_;
  stereoFrame_k_.reset();
  ++frame_count_;

  ///////////////////////////// IMU Frontend ///////////////////////////////////
  // Initialize IMU Frontend.
  imu_frontend_->resetIntegrationWithCachedBias();
}

/* -------------------------------------------------------------------------- */
// Frontend WORKHORSE
// THIS FUNCTION CAN BE GREATLY OPTIMIZED
// TODO(marcus): const ref cur_frame mutable members are modified! label is
// misleading
StatusStereoMeasurementsPtr StereoVisionImuFrontend::processStereoFrame(
    const StereoFrame& cur_frame,
    const gtsam::Rot3& keyframe_R_cur_frame,
    cv::Mat* feature_tracks) {
  CHECK(tracker_);
  VLOG(2) << "===================================================\n"
          << "Frame number: " << frame_count_ << " at time "
          << cur_frame.timestamp_ << " empirical framerate (sec): "
          << UtilsNumerical::NsecToSec(cur_frame.timestamp_ -
                                       stereoFrame_km1_->timestamp_)
          << " (timestamp diff: "
          << cur_frame.timestamp_ - stereoFrame_km1_->timestamp_ << ")";
  auto start_time = utils::Timer::tic();

  // TODO this copies the stereo frame!!
  stereoFrame_k_ = std::make_shared<StereoFrame>(cur_frame);
  Frame* left_frame_k = &stereoFrame_k_->left_frame_;

  /////////////////////// MONO TRACKING ////////////////////////////////////////
  VLOG(2) << "Starting feature tracking...";
  // We need to use the frame to frame rotation.
  gtsam::Rot3 ref_frame_R_cur_frame =
      keyframe_R_ref_frame_.inverse().compose(keyframe_R_cur_frame);
  tracker_->featureTracking(&stereoFrame_km1_->left_frame_,
                            left_frame_k,
                            ref_frame_R_cur_frame,
                            frontend_params_.feature_detector_params_,
                            stereo_camera_->getR1());

  // feature tracking failed for all points, move on to the next frame
  if (left_frame_k->keypoints_.size() == 0) {
    VLOG(2)
        << "feature tracking failed for all points, moving to next frame \n";
    feature_detector_->featureDetection(left_frame_k, stereo_camera_->getR1());
    stereoFrame_km1_ = stereoFrame_k_;
    stereoFrame_k_.reset();
    ++frame_count_;
    StereoMeasurements smart_stereo_measurements;
    return std::make_shared<StatusStereoMeasurements>(
        std::make_pair(tracker_status_summary_, smart_stereo_measurements));
  }

  if (feature_tracks) {
    // TODO(Toni): these feature tracks are not outlier rejected...
    // TODO(Toni): this image should already be computed and inside the
    // display_queue
    // if it is sent to the tracker.
    *feature_tracks = tracker_->getTrackerImage(stereoFrame_lkf_->left_frame_,
                                                stereoFrame_k_->left_frame_);
  }

  VLOG(2) << "Finished feature tracking.";
  //////////////////////////////////////////////////////////////////////////////

  StereoMeasurements smart_stereo_measurements;

  // determine if frame should be a keyframe
  const bool new_keyframe = shouldBeKeyframe(stereoFrame_k_->left_frame_,
                                             stereoFrame_lkf_->left_frame_);
  if (new_keyframe) {
    ++keyframe_count_;  // mainly for debugging

    tracker_status_summary_.kfTrackingStatus_mono_ = TrackingStatus::INVALID;
    tracker_status_summary_.kfTrackingStatus_stereo_ = TrackingStatus::INVALID;

    double sparse_stereo_time = 0;
    if (frontend_params_.useRANSAC_) {
      // MONO geometric outlier rejection
      TrackingStatusPose status_pose_mono;
      Frame* left_frame_lkf = &stereoFrame_lkf_->left_frame_;
      outlierRejectionMono(keyframe_R_cur_frame,
                           left_frame_lkf,
                           left_frame_k,
                           &status_pose_mono);
      tracker_status_summary_.kfTrackingStatus_mono_ = status_pose_mono.first;
      if (status_pose_mono.first == TrackingStatus::VALID) {
        tracker_status_summary_.lkf_T_k_mono_ = status_pose_mono.second;
      }
      // STEREO geometric outlier rejection
      // get 3D points via stereo
      start_time = utils::Timer::tic();
      stereo_matcher_.sparseStereoReconstruction(stereoFrame_k_.get());
      sparse_stereo_time = utils::Timer::toc(start_time).count();

      TrackingStatusPose status_pose_stereo;
      if (frontend_params_.use_stereo_tracking_) {
        outlierRejectionStereo(
            stereo_camera_->getGtsamStereoCam(),
            keyframe_R_cur_frame,
            stereoFrame_lkf_.get(),
            stereoFrame_k_.get(),
            &status_pose_stereo,
            &tracker_status_summary_.infoMatStereoTranslation_);
        tracker_status_summary_.kfTrackingStatus_stereo_ =
            status_pose_stereo.first;

        if (status_pose_stereo.first == TrackingStatus::VALID) {
          tracker_status_summary_.lkf_T_k_stereo_ = status_pose_stereo.second;
        }
      } else {
        status_pose_stereo.first = TrackingStatus::INVALID;
        status_pose_stereo.second = gtsam::Pose3();
        tracker_status_summary_.kfTrackingStatus_stereo_ =
            TrackingStatus::INVALID;
      }

      if (frontend_params_.use_pnp_tracking_) {
        TrackingStatusPose status_pose_pnp;
        outlierRejectionPnP(*stereoFrame_k_, &status_pose_pnp);

        tracker_status_summary_.kfTracking_status_pnp_ = status_pose_pnp.first;
        tracker_status_summary_.W_T_k_pnp_ = status_pose_pnp.second;
      } else {
        tracker_status_summary_.kfTracking_status_pnp_ =
            TrackingStatus::INVALID;
        tracker_status_summary_.W_T_k_pnp_ = gtsam::Pose3();
      }

    } else {
      tracker_status_summary_.kfTrackingStatus_mono_ = TrackingStatus::DISABLED;
      tracker_status_summary_.kfTrackingStatus_stereo_ =
          TrackingStatus::DISABLED;
    }

    if (VLOG_IS_ON(2)) {
      printTrackingStatus(tracker_status_summary_.kfTrackingStatus_mono_,
                          "mono");
      printTrackingStatus(tracker_status_summary_.kfTrackingStatus_stereo_,
                          "stereo");
    }

    // If its been long enough, make it a keyframe
    last_keyframe_timestamp_ = stereoFrame_k_->timestamp_;
    stereoFrame_k_->setIsKeyframe(true);

    // Perform feature detection (note: this must be after RANSAC,
    // since if we discard more features, we need to extract more)
    CHECK(feature_detector_);
    feature_detector_->featureDetection(left_frame_k, stereo_camera_->getR1());

    // Get 3D points via stereo, including newly extracted
    // (this might be only for the visualization).
    start_time = utils::Timer::tic();
    stereo_matcher_.sparseStereoReconstruction(stereoFrame_k_.get());
    sparse_stereo_time += utils::Timer::toc(start_time).count();

    // Log images if needed.
    if (logger_ &&
        (FLAGS_visualize_frontend_images || FLAGS_save_frontend_images)) {
      if (FLAGS_log_feature_tracks) sendFeatureTracksToLogger();
      if (FLAGS_log_mono_tracking_images) sendStereoMatchesToLogger();
      if (FLAGS_log_stereo_matching_images) sendMonoTrackingToLogger();
    }
    if (display_queue_ && FLAGS_visualize_feature_tracks) {
      displayImage(stereoFrame_k_->timestamp_,
                   "feature_tracks",
                   tracker_->getTrackerImage(stereoFrame_lkf_->left_frame_,
                                             stereoFrame_k_->left_frame_),
                   display_queue_);
    }

    // Populate statistics.
    stereoFrame_k_->checkStatusRightKeypoints(&tracker_->debug_info_);

    // Move on.
    stereoFrame_lkf_ = stereoFrame_k_;

    // Get relevant info for keyframe.
    start_time = utils::Timer::tic();
    getSmartStereoMeasurements(stereoFrame_k_, &smart_stereo_measurements);
    double get_smart_stereo_meas_time = utils::Timer::toc(start_time).count();

    VLOG(2) << "timeSparseStereo: " << sparse_stereo_time << '\n'
            << "timeGetMeasurements: " << get_smart_stereo_meas_time;
  } else {
    CHECK_EQ(smart_stereo_measurements.size(), 0u);
    stereoFrame_k_->setIsKeyframe(false);
  }

  // Update keyframe to reference frame for next iteration.
  if (stereoFrame_k_->isKeyframe()) {
    // Reset relative rotation if we have a keyframe.
    keyframe_R_ref_frame_ = gtsam::Rot3();
  } else {
    // Update rotation from keyframe to next iteration reference frame (aka
    // cur_frame in current iteration).
    keyframe_R_ref_frame_ = keyframe_R_cur_frame;
  }

  // Reset frames.
  stereoFrame_km1_ = stereoFrame_k_;
  stereoFrame_k_.reset();
  ++frame_count_;
  return std::make_shared<StatusStereoMeasurements>(
      std::make_pair(tracker_status_summary_,
                     // TODO(Toni): please, fix this, don't use std::pair...
                     // copies, manyyyy copies: actually thousands of copies...
                     smart_stereo_measurements));
}

/* -------------------------------------------------------------------------- */
// TODO(Toni): THIS FUNCTION CAN BE GREATLY OPTIMIZED...
void StereoVisionImuFrontend::getSmartStereoMeasurements(
    const StereoFrame::Ptr& stereoFrame_kf,
    StereoMeasurements* smart_stereo_measurements) const {
  // Sanity check first.
  CHECK(tracker_);
  CHECK_NOTNULL(smart_stereo_measurements);
  CHECK(stereoFrame_kf->isRectified())
      << "getSmartStereoMeasurements: stereo pair is not rectified";
  // Checks dimensionality of the feature vectors.
  stereoFrame_kf->checkStereoFrame();

  // Extract relevant info from the stereo frame:
  // essentially the landmark if and the left/right pixel measurements.
  const LandmarkIds& landmarkId_kf = stereoFrame_kf->left_frame_.landmarks_;
  const StatusKeypointsCV& leftKeypoints =
      stereoFrame_kf->left_keypoints_rectified_;
  const StatusKeypointsCV& rightKeypoints =
      stereoFrame_kf->right_keypoints_rectified_;

  // Pack information in landmark structure.
  smart_stereo_measurements->clear();
  smart_stereo_measurements->reserve(landmarkId_kf.size());
  for (size_t i = 0; i < landmarkId_kf.size(); ++i) {
    if (landmarkId_kf.at(i) == -1) {
      continue;  // skip invalid points
    }

    // TODO implicit conversion float to double increases floating-point
    // precision!
    const double& uL = leftKeypoints.at(i).second.x;
    const double& v = leftKeypoints.at(i).second.y;
    // Initialize to missing pixel information.
    double uR = std::numeric_limits<double>::quiet_NaN();
    if (!frontend_params_.use_stereo_tracking_) {
      LOG_EVERY_N(WARNING, 10) << "Dropping stereo information! (set "
                                  "use_stereo_tracking_ = true to use it)";
    }
    if (frontend_params_.use_stereo_tracking_ &&
        rightKeypoints.at(i).first == KeypointStatus::VALID) {
      // TODO implicit conversion float to double increases floating-point
      // precision!
      uR = rightKeypoints.at(i).second.x;
    }
    smart_stereo_measurements->push_back(
        std::make_pair(landmarkId_kf[i], gtsam::StereoPoint2(uL, uR, v)));
  }
}

/* -------------------------------------------------------------------------- */
void StereoVisionImuFrontend::sendFeatureTracksToLogger() const {
  CHECK(tracker_);
  const Frame& left_frame_k(stereoFrame_k_->left_frame_);
  cv::Mat img_left =
      tracker_->getTrackerImage(stereoFrame_lkf_->left_frame_, left_frame_k);

  logger_->logFrontendImg(left_frame_k.id_,
                          img_left,
                          "monoFeatureTracksLeft",
                          "/monoFeatureTracksLeftImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
}

/* -------------------------------------------------------------------------- */
void StereoVisionImuFrontend::sendStereoMatchesToLogger() const {
  CHECK(tracker_);
  // Draw the matchings: assumes that keypoints in the left and right keyframe
  // are ordered in the same way
  const Frame& left_frame_k(stereoFrame_k_->left_frame_);
  const Frame& right_frame_k(stereoFrame_k_->right_frame_);

  cv::Mat img_left =
      tracker_->getTrackerImage(stereoFrame_lkf_->left_frame_, left_frame_k);

  if ((left_frame_k.img_.cols != right_frame_k.img_.cols) ||
      (left_frame_k.img_.rows != right_frame_k.img_.rows)) {
    LOG(FATAL) << "sendStereoMatchesToLogger: image dimension mismatch!";
  }

  cv::Mat img_right;
  right_frame_k.img_.copyTo(img_right);
  // stereoFrame_k_->keypoints_depth_

  DMatchVec matches;
  const StatusKeypointsCV& right_status_keypoints =
      stereoFrame_k_->right_keypoints_rectified_;
  if (left_frame_k.keypoints_.size() == right_frame_k.keypoints_.size()) {
    for (size_t i = 0; i < left_frame_k.keypoints_.size(); i++) {
      if (left_frame_k.landmarks_[i] != -1 &&
          right_status_keypoints[i].first == KeypointStatus::VALID) {
        matches.push_back(cv::DMatch(i, i, 0));
      }
    }
  } else {
    VLOG(10) << "No matches to show, since we did not compute keypoints "
                "in right frame.";
  }

  //############################################################################
  // Plot matches.
  cv::Mat img_left_right =
      UtilsOpenCV::DrawCornersMatches(img_left,
                                      left_frame_k.keypoints_,
                                      img_right,
                                      right_frame_k.keypoints_,
                                      matches,
                                      false);  // true: random color
  cv::putText(img_left_right,
              "S:" + std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(left_frame_k.id_,
                          img_left_right,
                          "stereoMatchingUnrectified",
                          "/stereoMatchingUnrectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
  //############################################################################

  // Display rectified, plot matches.
  static constexpr bool kUseRandomColor = false;
  cv::Mat img_left_right_rectified = UtilsOpenCV::DrawCornersMatches(
      stereoFrame_k_->getLeftImgRectified(),
      stereoFrame_k_->left_keypoints_rectified_,
      stereoFrame_k_->getRightImgRectified(),
      stereoFrame_k_->right_keypoints_rectified_,
      matches,
      kUseRandomColor);
  cv::putText(img_left_right_rectified,
              "S(Rect):" + std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(left_frame_k.id_,
                          img_left_right_rectified,
                          "stereoMatchingRectified",
                          "/stereoMatchingRectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
}

/* -------------------------------------------------------------------------- */
void StereoVisionImuFrontend::sendMonoTrackingToLogger() const {
  const Frame& cur_left_frame = stereoFrame_k_->left_frame_;
  const Frame& ref_left_frame = stereoFrame_lkf_->left_frame_;

  // Find keypoint matches.
  DMatchVec matches;
  for (size_t i = 0; i < cur_left_frame.keypoints_.size(); ++i) {
    if (cur_left_frame.landmarks_.at(i) != -1) {  // if landmark is valid
      auto it = find(ref_left_frame.landmarks_.begin(),
                     ref_left_frame.landmarks_.end(),
                     cur_left_frame.landmarks_.at(i));
      if (it != ref_left_frame.landmarks_.end()) {  // if landmark was found
        int nPos = std::distance(ref_left_frame.landmarks_.begin(), it);
        matches.push_back(cv::DMatch(nPos, i, 0));
      }
    }
  }
  //############################################################################

  // Plot matches.
  cv::Mat img_left_lkf_kf =
      UtilsOpenCV::DrawCornersMatches(ref_left_frame.img_,
                                      ref_left_frame.keypoints_,
                                      cur_left_frame.img_,
                                      cur_left_frame.keypoints_,
                                      matches,
                                      false);  // true: random color
  cv::putText(img_left_lkf_kf,
              "M:" + std::to_string(keyframe_count_ - 1) + "-" +
                  std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(cur_left_frame.id_,
                          img_left_lkf_kf,
                          "monoTrackingUnrectified",
                          "/monoTrackingUnrectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
  //############################################################################

  // Display rectified, plot matches.
  static constexpr bool kUseRandomColor = false;
  cv::Mat img_left_lkf_kf_rectified = StereoFrame::drawCornersMatches(
      *stereoFrame_lkf_, *stereoFrame_k_, matches, kUseRandomColor);
  cv::putText(img_left_lkf_kf_rectified,
              "M(Rect):" + std::to_string(keyframe_count_ - 1) + "-" +
                  std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(cur_left_frame.id_,
                          img_left_lkf_kf_rectified,
                          "monoTrackingRectified",
                          "/monoTrackingRectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
  // TODO Visualization must be done in the main thread.
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - MONO
// RANSAC
gtsam::Pose3 StereoVisionImuFrontend::getRelativePoseBodyMono() const {
  // lkfBody_T_kBody = lkfBody_T_lkfCamera *  lkfCamera_T_kCamera_ *
  // kCamera_T_kBody = body_Pose_cam_ * lkf_T_k_mono_ * body_Pose_cam_^-1
  gtsam::Pose3 body_Pose_cam_ =
      stereo_camera_->getBodyPoseLeftCamRect();  // of the left camera!!
  return body_Pose_cam_ * tracker_status_summary_.lkf_T_k_mono_ *
         body_Pose_cam_.inverse();
}

/* -------------------------------------------------------------------------- */
// return relative pose between last (lkf) and current keyframe (k) - STEREO
// RANSAC
gtsam::Pose3 StereoVisionImuFrontend::getRelativePoseBodyStereo() const {
  gtsam::Pose3 body_Pose_cam_ =
      stereo_camera_->getBodyPoseLeftCamRect();  // of the left camera!!
  return body_Pose_cam_ * tracker_status_summary_.lkf_T_k_stereo_ *
         body_Pose_cam_.inverse();
}

/* ------------------------------------------------------------------------ */
// Static function to display output of stereo tracker
void StereoVisionImuFrontend::printStatusStereoMeasurements(
    const StatusStereoMeasurements& statusStereoMeasurements) {
  LOG(INFO) << " SmartStereoMeasurements with status:";
  printTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_mono_,
                      "mono");
  printTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_stereo_,
                      "stereo");
  LOG(INFO) << " stereo points:";
  const StereoMeasurements& smartStereoMeas = statusStereoMeasurements.second;
  for (const auto& smart_stereo_meas : smartStereoMeas) {
    LOG(INFO) << " " << smart_stereo_meas.second << " ";
  }
  LOG(INFO) << std::endl;
}

}  // namespace VIO
