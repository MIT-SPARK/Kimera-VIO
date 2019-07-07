/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.cpp
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial
 * Navigation. IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying
 * Perspective based on Smart Factors. In IEEE Intl. Conf. on Robotics and
 * Automation (ICRA), 2014.
 *
 * @author Marcus Abate
 */

#include "LoopClosureDetector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace VIO {

LoopClosureDetector::LoopClosureDetector(
    const LoopClosureDetectorParams& lcd_params,
    const bool log_output)
    : lcd_params_(lcd_params),
      log_output_(log_output),
      set_intrinsics_(false) {
  orb_feature_detector_ = cv::ORB::create(lcd_params_.nfeatures_,
                                          lcd_params_.scaleFactor_,
                                          lcd_params_.nlevels_,
                                          lcd_params_.edgeThreshold_,
                                          lcd_params_.firstLevel_,
                                          lcd_params_.WTA_K_,
                                          lcd_params_.scoreType_,
                                          lcd_params_.patchSize_,
                                          lcd_params_.fastThreshold_);

  std::cout << "Loading vocabulary from " << lcd_params_.vocabulary_path_
            << std::endl;
  vocab_.loadFromTextFile(lcd_params_.vocabulary_path_); // TODO: support .gz files
  std::cout << "Loaded vocabulary with " << vocab_.size() << " visual words."
            << std::endl;
}

bool LoopClosureDetector::spin(
          ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
          ThreadsafeQueue<LoopClosureDetectorOutputPayload>& output_queue,
          bool parallel_run) {
  LOG(INFO) << "Spinning LoopClosureDetector.";
  // TODO: Timing stuff??
  // utils::StatsCollector stat_pipeline_timing("Pipeline Overall Timing [ms]");
  // utils::StatsCollector stat_lcd_timing("LoopClosureDetector Timing [ms]");
  while (!shutdown_) {
    is_thread_working_ = false;
    std::shared_ptr<LoopClosureDetectorInputPayload> input = input_queue.popBlocking();
    is_thread_working_ = true;
    if (input) {
      auto tic = utils::Timer::tic();
      LoopClosureDetectorOutputPayload output_payload = spinOnce(input);
      // Only push output if it's a detected loop.
      if (output_payload.is_loop_) {
        output_queue.push(output_payload);
      }
      auto spin_duration = utils::Timer::toc(tic).count();
      LOG(WARNING) << "Current LoopClosureDetector frequency: "
                   << 1000.0 / spin_duration << " Hz. ("
                   << spin_duration << " ms).";
      // stat_lcd_timing.AddSample(spin_duration);
    } else {
      LOG(WARNING) << "No LoopClosureDetector Input Payload received.";
    }

    if (!parallel_run) return true;
  }
  LOG(INFO) << "LoopClosureDetector successfully shutdown.";
  return true;
}

LoopClosureDetectorOutputPayload LoopClosureDetector::spinOnce(
    const std::shared_ptr<LoopClosureDetectorInputPayload>& input) {
  CHECK(input) << "No LoopClosureDetector Input Payload received.";

  // One time initialization from camera parameters:
  if (!set_intrinsics_) {
    lcd_params_.focal_length_ =
      input->stereo_frame_.getLeftFrame().cam_param_.intrinsics_[0];
    lcd_params_.principle_point_ = cv::Point2d(
        input->stereo_frame_.getLeftFrame().cam_param_.intrinsics_[2],
        input->stereo_frame_.getLeftFrame().cam_param_.intrinsics_[3]);

    initLoopDetector();
    set_intrinsics_ = true;
  }

  StereoFrame working_frame = input->stereo_frame_;

  LoopClosureDetectorOutputPayload output_payload = checkLoopClosure(
    working_frame);

  // TODO: add LoggerMatlab options

  return output_payload;
}

void LoopClosureDetector::initLoopDetector() {
  OrbLoopDetector::Parameters params;
  params.image_rows = lcd_params_.image_height_;
  params.image_cols = lcd_params_.image_width_;
  params.use_nss = lcd_params_.use_nss_;
  params.alpha = lcd_params_.alpha_;
  params.k = lcd_params_.min_temporal_matches_;
  params.geom_check = lcd_params_.geom_check_;
  params.di_levels = lcd_params_.di_levels_;
  params.dislocal = lcd_params_.dist_local_;
  params.max_db_results = lcd_params_.max_db_results_;
  params.min_nss_factor = lcd_params_.min_nss_factor_;
  params.min_matches_per_group = lcd_params_.min_matches_per_group_;
  params.max_intragroup_gap = lcd_params_.max_intragroup_gap_;
  params.max_distance_between_groups = lcd_params_.max_distance_between_groups_;
  params.max_distance_between_queries =
    lcd_params_.max_distance_between_queries_;
  params.min_Fpoints = lcd_params_.min_Fpoints_;
  params.max_ransac_iterations = lcd_params_.max_ransac_iterations_;
  params.ransac_probability = lcd_params_.ransac_probability_;
  params.max_reprojection_error = lcd_params_.max_reprojection_error_;
  params.max_neighbor_ratio = lcd_params_.max_neighbor_ratio_;
  params.focal_length = lcd_params_.focal_length_;
  params.principle_point = lcd_params_.principle_point_;

  // loop_detector_ = OrbLoopDetector(vocab_, params); // TODO: Why doesn't this work?
  loop_detector_ = OrbLoopDetector(params);
  loop_detector_.setVocabulary(vocab_);
}

LoopClosureDetectorOutputPayload LoopClosureDetector::checkLoopClosure(
    StereoFrame& stereo_frame) {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Mat> descriptors;

  extractOrb(stereo_frame.getLeftFrame().img_, keypoints, descriptors);

  DLoopDetector::DetectionResult loop_result;
  loop_detector_.detectLoop(keypoints, descriptors, loop_result);

  if (loop_result.detection()) {
    LOG(ERROR) << "LoopClosureDetector: LOOP CLOSURE detected from image "
               << loop_result.match << " to image " << loop_result.query
               << std::endl;

    // Populate frame keypoints with ORB features instead of the normal
    // VIO features that came with the StereoFrame.

    // TODO: is this thread safe? Should I copy the stereo_frame object first?
    // Frame* left_frame_mutable = stereo_frame.getLeftFrameMutable();
    // Frame* right_frame_mutable = stereo_frame.getRightFrameMutable();
    //
    // // Clear all relevant fields.
    // left_frame_mutable->keypoints_.clear();
    // left_frame_mutable->versors_.clear();
    // right_frame_mutable->keypoints_.clear();
    // right_frame_mutable->versors_.clear();
    // stereo_frame.keypoints_3d_.clear();
    // stereo_frame.left_keypoints_rectified_.clear();
    // stereo_frame.right_keypoints_rectified_.clear();
    //
    // // Add ORB keypoints.
    // left_frame_mutable->keypoints_ = KeypointsCV(keypoints.size());
    // for (unsigned int i; i<keypoints.size(); i++) {
    //   // TODO: additional requirements for converting cv::KeyPoint to KeypointCV?
    //   left_frame_mutable->keypoints_[i] = keypoints[i].pt;
    // }
    //
    // // Automatically match keypoints in right image with those in left.
    // stereo_frame.sparseStereoMatching();

    // Compute the scaling factor for the translation between matched frames.
    // TODO: does this work as expected to convert cv::Mat into Matrix3?
    // TODO: may need a transposition on the rotation matrix.
    // TODO: do you even need to clear the old keypoints? Why not use the old ones?
    // TODO: Almost certainly you have the pose backwards. It seems like it's pose from ref to curr,
    // but you don't have access to ref so you need to invert (?)
    // TODO: opencv seems to calculate movement of points, not of camera. In this case you might have to invert?
    std::vector<gtsam::Point3> cols(3);
    for (unsigned int i=0; i<3; i++) {
      gtsam::Point3 col = gtsam::Point3(loop_result.rotation.at<double>(0,i),
                                        loop_result.rotation.at<double>(1,i),
                                        loop_result.rotation.at<double>(2,i));
      cols[i] = col;
    }

    gtsam::Rot3 rot = gtsam::Rot3(cols[0], cols[1], cols[2]);
    gtsam::Vector3 unit_T = gtsam::Vector3(loop_result.translation.at<double>(0),
      loop_result.translation.at<double>(1),
      loop_result.translation.at<double>(2));
    gtsam::Vector3 rotated_unit_T = rot * unit_T;
    float scaling_factor = 0.0;

    for (unsigned int i; i<stereo_frame.keypoints_3d_.size(); i++) {
      gtsam::Vector3 curr_keypoint = stereo_frame.keypoints_3d_[i];
      gtsam::Vector3 rotated_keypoint = rot * curr_keypoint;
      scaling_factor += rotated_keypoint.dot(rotated_unit_T);
    }
    scaling_factor /= stereo_frame.keypoints_3d_.size();

    gtsam::Point3 scaled_T = gtsam::Point3(unit_T[0] * scaling_factor,
        unit_T[1] * scaling_factor,
        unit_T[2] * scaling_factor);
    gtsam::Pose3 match_pose_curr = gtsam::Pose3(rot, scaled_T);

    LoopClosureDetectorOutputPayload output_payload(true,
        stereo_frame.getTimestamp(), loop_result.query, loop_result.match,
        match_pose_curr);

    LOG(INFO) << "\nLoopClosureDetector: stats:"
              << "\n\nloop_result translation:\n" << loop_result.translation
              << "\n\nloop_result rotation:\n" << loop_result.rotation
              << "\n\nunit translation:\n" << unit_T
              << "\n\nrotated unit translation:\n" << rotated_unit_T
              << "\n\nscaling factor: " << scaling_factor
              << "\n\nscaled translation:\n" << scaled_T
              << "\n\nrotation:\n" << rot
              << "\n";

    return output_payload;
  }
  else {
    // TODO: print the status here for when loops are not detected
    return LoopClosureDetectorOutputPayload(false,-1,0,0,gtsam::Pose3());
  }
}

void LoopClosureDetector::extractOrb(const cv::Mat& img,
    std::vector<cv::KeyPoint>& keypoints,
    std::vector<cv::Mat>& descriptors) {
  cv::Mat plain;

  orb_feature_detector_->detectAndCompute(img, cv::Mat(), keypoints, plain);

  int L = orb_feature_detector_->descriptorSize();
  descriptors.resize(plain.size().height);

  for (unsigned int i = 0; i < descriptors.size(); i++) {
    descriptors[i] = cv::Mat(1, L, plain.type()); // one row only
    plain.row(i).copyTo(descriptors[i].row(0));
  }
}

} // namespace VIO
