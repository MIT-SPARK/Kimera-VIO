/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoVisionImuFrontend.cpp
 * @brief  Class describing a monocular tracking Frontend
 * @author Marcus Abate
 */

#include "kimera-vio/frontend/MonoVisionImuFrontend.h"

#include <memory>

#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/utils/UtilsNumerical.h"

DEFINE_bool(log_mono_matching_images,
            false,
            "Display/Save mono tracking rectified and unrectified images.");
DECLARE_bool(do_fine_imu_camera_temporal_sync);

namespace VIO {

MonoVisionImuFrontend::MonoVisionImuFrontend(
    const FrontendParams& frontend_params,
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const Camera::ConstPtr& camera,
    DisplayQueue* display_queue,
    bool log_output,
    std::optional<OdometryParams> odom_params)
    : VisionImuFrontend(frontend_params,
                        imu_params,
                        imu_initial_bias,
                        display_queue,
                        log_output,
                        odom_params),
      mono_frame_k_(nullptr),
      mono_frame_km1_(nullptr),
      mono_frame_lkf_(nullptr),
      keyframe_R_ref_frame_(gtsam::Rot3()),
      feature_detector_(nullptr),
      mono_camera_(camera) {
  CHECK(mono_camera_);

  tracker_ = std::make_unique<Tracker>(
      frontend_params_.tracker_params_, mono_camera_, display_queue);

  feature_detector_ = std::make_unique<FeatureDetector>(
      frontend_params_.feature_detector_params_);

  if (VLOG_IS_ON(1)) tracker_->tracker_params_.print();
}

MonoVisionImuFrontend::~MonoVisionImuFrontend() {
  LOG(INFO) << "MonoVisionImuFrontend destructor called.";
}

MonoFrontendOutput::UniquePtr MonoVisionImuFrontend::bootstrapSpinMono(
    MonoFrontendInputPayload::UniquePtr&& input) {
  CHECK(input);

  // Initialize members of the Frontend
  processFirstFrame(input->getFrame());

  // Initialization done, set state to nominal
  frontend_state_ = FLAGS_do_fine_imu_camera_temporal_sync
                        ? FrontendState::InitialTimeAlignment
                        : FrontendState::Nominal;

  if (!FLAGS_do_fine_imu_camera_temporal_sync && odom_params_) {
    // we assume that the first frame is hardcoded to be a keyframe.
    // it's okay if world_NavState_odom_ is none (it gets cached later)
    cacheExternalOdometry(input.get());
  }

  // Create mostly invalid output
  CHECK(mono_frame_lkf_);
  CHECK(mono_camera_);

  if (FLAGS_do_fine_imu_camera_temporal_sync) {
    return nullptr;  // skip adding a frame to all downstream modules
  }

  // Create mostly invalid output
  return std::make_unique<MonoFrontendOutput>(mono_frame_lkf_->isKeyframe_,
                                              nullptr,
                                              mono_camera_->getBodyPoseCam(),
                                              *mono_frame_lkf_,
                                              nullptr,
                                              input->getImuAccGyrs(),
                                              cv::Mat(),
                                              getTrackerInfo());
}

MonoFrontendOutput::UniquePtr MonoVisionImuFrontend::nominalSpinMono(
    MonoFrontendInputPayload::UniquePtr&& input) {
  // For timing
  utils::StatsCollector timing_stats_frame_rate("VioFrontend Frame Rate [ms]");
  utils::StatsCollector timing_stats_keyframe_rate(
      "VioFrontend Keyframe Rate [ms]");
  auto start_time = utils::Timer::tic();

  const Frame& mono_frame_k = input->getFrame();
  const auto& k = mono_frame_k.id_;
  VLOG(1) << "------------------- Processing frame k = " << k
          << "--------------------";

  if (VLOG_IS_ON(10)) input->print();

  // auto tic_full_preint = utils::Timer::tic();
  const ImuFrontend::PimPtr& pim = imu_frontend_->preintegrateImuMeasurements(
      input->getImuStamps(), input->getImuAccGyrs());
  CHECK(pim);
  const gtsam::Rot3 body_R_cam = mono_camera_->getBodyPoseCam().rotation();
  const gtsam::Rot3 cam_R_body = body_R_cam.inverse();
  gtsam::Rot3 camLrectLkf_R_camLrectK_imu =
      cam_R_body * pim->deltaRij() * body_R_cam;

  if (VLOG_IS_ON(10)) {
    body_R_cam.print("body_R_cam");
    camLrectLkf_R_camLrectK_imu.print("camLrectLkf_R_camLrectK_imu");
  }

  /////////////////////////////// TRACKING /////////////////////////////////////
  VLOG(10) << "Starting processFrame...";
  cv::Mat feature_tracks;
  StatusMonoMeasurementsPtr status_mono_measurements =
      processFrame(mono_frame_k, camLrectLkf_R_camLrectK_imu, &feature_tracks);
  CHECK(!mono_frame_k_);  // We want a nullptr at the end of the processing.
  VLOG(10) << "Finished processStereoFrame.";
  //////////////////////////////////////////////////////////////////////////////

  if (VLOG_IS_ON(5))
    MonoVisionImuFrontend::printStatusMonoMeasurements(
        *status_mono_measurements);

  if (mono_frame_km1_->isKeyframe_) {
    CHECK_EQ(mono_frame_lkf_->timestamp_, mono_frame_km1_->timestamp_);
    CHECK_EQ(mono_frame_lkf_->id_, mono_frame_km1_->id_);
    CHECK(!mono_frame_k_);
    CHECK(mono_frame_lkf_->isKeyframe_);
    VLOG(1) << "Keyframe " << k
            << " with: " << status_mono_measurements->second.size()
            << " smart measurements";

    ////////////////// DEBUG INFO FOR FRONT-END ////////////////////////////////
    if (logger_) {
      logger_->logFrontendStats(mono_frame_lkf_->timestamp_,
                                getTrackerInfo(),
                                tracker_status_summary_,
                                mono_frame_km1_->getNrValidKeypoints());
      logger_->logFrontendRansac(mono_frame_lkf_->timestamp_,
                                 tracker_status_summary_.lkf_T_k_mono_,
                                 gtsam::Pose3());
    }
    //////////////////////////////////////////////////////////////////////////////

    // Reset integration; the later the better.
    VLOG(10) << "Reset IMU preintegration with latest IMU bias.";
    imu_frontend_->resetIntegrationWithCachedBias();

    // Record keyframe rate timing
    timing_stats_keyframe_rate.AddSample(utils::Timer::toc(start_time).count());

    // Return the output of the Frontend for the others.
    // We have a keyframe, so We fill frame_lkf_ with the newest keyframe
    VLOG(2) << "Frontend output is a keyframe: pushing to output callbacks.";
    return std::make_unique<MonoFrontendOutput>(
        frontend_state_ == FrontendState::Nominal,
        status_mono_measurements,
        mono_camera_->getBodyPoseCam(),
        *mono_frame_lkf_,
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
    return std::make_unique<MonoFrontendOutput>(false,
                                                status_mono_measurements,
                                                mono_camera_->getBodyPoseCam(),
                                                *mono_frame_km1_,
                                                pim,
                                                input->getImuAccGyrs(),
                                                feature_tracks,
                                                getTrackerInfo());
  }
}

void MonoVisionImuFrontend::processFirstFrame(const Frame& first_frame) {
  VLOG(2) << "Processing first mono frame \n";
  mono_frame_k_ = std::make_shared<Frame>(first_frame);
  mono_frame_k_->isKeyframe_ = true;
  last_keyframe_timestamp_ = mono_frame_k_->timestamp_;

  CHECK_EQ(mono_frame_k_->keypoints_.size(), 0)
      << "Keypoints already present in first frame: please do not extract"
         " keypoints manually";

  CHECK(feature_detector_);
  feature_detector_->featureDetection(mono_frame_k_.get());

  // Undistort keypoints:
  mono_camera_->undistortKeypoints(mono_frame_k_->keypoints_,
                                   &mono_frame_k_->keypoints_undistorted_);

  // TODO(marcus): get 3d points if possible?
  mono_frame_km1_ = mono_frame_k_;
  mono_frame_lkf_ = mono_frame_k_;
  mono_frame_k_.reset();
  ++frame_count_;

  imu_frontend_->resetIntegrationWithCachedBias();
}

StatusMonoMeasurementsPtr MonoVisionImuFrontend::processFrame(
    const Frame& cur_frame,
    const gtsam::Rot3& keyframe_R_cur_frame,
    cv::Mat* feature_tracks) {
  VLOG(1) << "===================================================\n"
          << "processing frame: " << cur_frame.id_ << " at time "
          << cur_frame.timestamp_ << " empirical framerate (sec): "
          << UtilsNumerical::NsecToSec(cur_frame.timestamp_ -
                                       mono_frame_km1_->timestamp_)
          << " (timestamp diff: "
          << cur_frame.timestamp_ - mono_frame_km1_->timestamp_ << ")";
  auto start_time = utils::Timer::tic();

  mono_frame_k_ = std::make_shared<Frame>(cur_frame);

  VLOG(2) << "Starting feature tracking...";
  gtsam::Rot3 ref_frame_R_cur_frame =
      keyframe_R_ref_frame_.inverse().compose(keyframe_R_cur_frame);
  tracker_->featureTracking(mono_frame_km1_.get(),
                            mono_frame_k_.get(),
                            ref_frame_R_cur_frame,
                            frontend_params_.feature_detector_params_);
  if (feature_tracks) {
    *feature_tracks =
        tracker_->getTrackerImage(*mono_frame_lkf_, *mono_frame_k_);
  }
  VLOG(2) << "Finished feature tracking.";

  // TODO(marcus): need another structure for monocular slam
  tracker_status_summary_.kfTrackingStatus_mono_ = TrackingStatus::INVALID;
  tracker_status_summary_.kfTrackingStatus_stereo_ = TrackingStatus::DISABLED;

  MonoMeasurements smart_mono_measurements;

  // determine if frame should be a keyframe
  const bool new_keyframe = shouldBeKeyframe(*mono_frame_k_, *mono_frame_lkf_);
  if (new_keyframe) {
    ++keyframe_count_;

    if (frontend_params_.useRANSAC_) {
      TrackingStatusPose status_pose_mono;
      outlierRejectionMono(keyframe_R_cur_frame,
                           mono_frame_lkf_.get(),
                           mono_frame_k_.get(),
                           &status_pose_mono);
      tracker_status_summary_.kfTrackingStatus_mono_ = status_pose_mono.first;

      if (status_pose_mono.first == TrackingStatus::VALID) {
        tracker_status_summary_.lkf_T_k_mono_ = status_pose_mono.second;
      }
    } else {
      tracker_status_summary_.kfTrackingStatus_mono_ = TrackingStatus::DISABLED;
    }

    if (VLOG_IS_ON(2)) {
      printTrackingStatus(tracker_status_summary_.kfTrackingStatus_mono_,
                          "mono");
    }

    last_keyframe_timestamp_ = mono_frame_k_->timestamp_;
    mono_frame_k_->isKeyframe_ = true;

    CHECK(feature_detector_);
    feature_detector_->featureDetection(mono_frame_k_.get());

    // Undistort keypoints:
    mono_camera_->undistortKeypoints(mono_frame_k_->keypoints_,
                                     &mono_frame_k_->keypoints_undistorted_);
    // Log images if needed.
    if (logger_ &&
        (FLAGS_visualize_frontend_images || FLAGS_save_frontend_images)) {
      if (FLAGS_log_feature_tracks) sendFeatureTracksToLogger();
      if (FLAGS_log_mono_matching_images) sendMonoTrackingToLogger();
    }
    if (display_queue_ && FLAGS_visualize_feature_tracks) {
      displayImage(mono_frame_k_->timestamp_,
                   "feature_tracks",
                   tracker_->getTrackerImage(*mono_frame_lkf_, *mono_frame_k_),
                   display_queue_);
    }

    mono_frame_lkf_ = mono_frame_k_;

    start_time = utils::Timer::tic();
    getSmartMonoMeasurements(mono_frame_k_, &smart_mono_measurements);
    double get_smart_mono_meas_time = utils::Timer::toc(start_time).count();

    VLOG(2) << "timeGetMeasurements: " << get_smart_mono_meas_time;
  } else {
    CHECK_EQ(smart_mono_measurements.size(), 0u);
    mono_frame_k_->isKeyframe_ = false;
  }

  if (mono_frame_k_->isKeyframe_) {
    keyframe_R_ref_frame_ = gtsam::Rot3();
  } else {
    keyframe_R_ref_frame_ = keyframe_R_cur_frame;
  }

  mono_frame_km1_ = mono_frame_k_;
  mono_frame_k_.reset();
  ++frame_count_;

  return std::make_shared<StatusMonoMeasurements>(
      std::make_pair(tracker_status_summary_, smart_mono_measurements));
}

// TODO(marcus): for convenience mono measurements are gtsam::StereoPoint2
// hence this is near identical to getSmartStereoMeasurements
// but want to switch to gtsam::Point2
void MonoVisionImuFrontend::getSmartMonoMeasurements(
    const Frame::Ptr& frame,
    MonoMeasurements* smart_mono_measurements) {
  // TODO(marcus): convert to point2 when ready!
  CHECK_NOTNULL(smart_mono_measurements);
  frame->checkFrame();

  const LandmarkIds& landmarkId_kf = frame->landmarks_;
  const StatusKeypointsCV& keypoints_undistorted =
      frame->keypoints_undistorted_;

  // Pack information in landmark structure.
  smart_mono_measurements->clear();
  smart_mono_measurements->reserve(landmarkId_kf.size());
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
    smart_mono_measurements->push_back(
        std::make_pair(landmarkId_kf[i], gtsam::StereoPoint2(uL, uR, v)));
  }
}

void MonoVisionImuFrontend::sendFeatureTracksToLogger() const {
  CHECK(tracker_);
  cv::Mat img_left =
      tracker_->getTrackerImage(*mono_frame_lkf_, *mono_frame_k_);
  logger_->logFrontendImg(mono_frame_k_->id_,
                          img_left,
                          "monoFeatureTracksLeft",
                          "/monoFeatureTracksLeftImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
}

void MonoVisionImuFrontend::sendMonoTrackingToLogger() const {
  const Frame& cur_left_frame = *mono_frame_k_;
  const Frame& ref_left_frame = *mono_frame_lkf_;

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
}

void MonoVisionImuFrontend::printStatusMonoMeasurements(
    const StatusMonoMeasurements& status_mono_measurements) {
  LOG(INFO) << "SmartMonoMeasurements with status: ";
  printTrackingStatus(status_mono_measurements.first.kfTrackingStatus_mono_,
                      "mono");
  LOG(INFO) << " stereo points:";
  const MonoMeasurements& mono_measurements = status_mono_measurements.second;
  for (const auto& meas : mono_measurements) {
    LOG(INFO) << " " << meas.second << " ";
  }
  LOG(INFO) << std::endl;
}

}  // namespace VIO
