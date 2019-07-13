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

  orb_feature_matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

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
  DLoopDetector::DetectionResult loop_result = processAndAddFrame(stereo_frame);

  if (loop_result.detection()) {
    LOG(ERROR) << "LoopClosureDetector: LOOP CLOSURE detected from image "
               << loop_result.match << " to image " << loop_result.query
               << std::endl;

    gtsam::Pose3 pose_2d = LoopClosureDetector::mat2pose(loop_result.rotation,
        loop_result.translation);
    gtsam::Pose3 pose_3d = computePoseStereoGiven2D(loop_result.query,
        loop_result.match, pose_2d);
    // gtsam::Pose3 pose_3d = computePoseStereoRansac(loop_result.query,
        // loop_result.match);
    gtsam::Pose3 match_pose_cur = gtsam::Pose3(pose_2d.rotation(),
        pose_3d.translation());

    LoopClosureDetectorOutputPayload output_payload(true,
      stereo_frame.getTimestamp(), loop_result.query, loop_result.match,
      match_pose_cur);

    return output_payload;
  }
  else {
    // std::cout << "Failure reason: ";
    // switch (loop_result.status) {
    //   case 0: std::cout << "Loop detected. You shouldn't be in this case.";
    //           break;
    //   case 1: std::cout << "All the matches are very recent.";
    //           break;
    //   case 2: std::cout << "No matches against the database.";
    //           break;
    //   case 3: std::cout << "Score of current image against previous too low.";
    //           break;
    //   case 4: std::cout << "Scores were below the alpha threshold.";
    //           break;
    //   case 5: std::cout << "Not enough matches to create groups.";
    //           break;
    //   case 6: std::cout << "Not enough temporal consistency between matches.";
    //           break;
    //   case 7: std::cout << "The geometric consistency check failed.";
    //           break;
    // }
    // std::cout << "\n" << std::endl;
    return LoopClosureDetectorOutputPayload(false,-1,0,0,gtsam::Pose3());
  }
}

DLoopDetector::DetectionResult LoopClosureDetector::processAndAddFrame(
    StereoFrame& stereo_frame) {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors_mat;
  std::vector<cv::Mat> descriptors_vec;

  // Extract ORB features and construct descriptors_vec
  orb_feature_detector_->detectAndCompute(stereo_frame.getLeftFrame().img_,
      cv::Mat(), keypoints, descriptors_mat);

  int L = orb_feature_detector_->descriptorSize();
  descriptors_vec.resize(descriptors_mat.size().height);

  for (unsigned int i=0; i<descriptors_vec.size(); i++) {
    descriptors_vec[i] = cv::Mat(1, L, descriptors_mat.type()); // one row only
    descriptors_mat.row(i).copyTo(descriptors_vec[i].row(0));
  }

  // Fill StereoFrame with ORB keypoints and perform stereo matching
  rewriteStereoFrameFeatures(stereo_frame, keypoints);

  // Save keypoints and descriptors for later detections
  // TODO: if I'm doing this anyway, is BOW even worth it? Seems like I'm
  // saving a lot of data...
  // TODO: ask Hank about ICP for matching random point clouds with a good
  // initial guess. That initial guess could be the rotation and unit
  // translation obtained from DBoW2...
  db_keypoints_.push_back(keypoints);
  db_descriptors_.push_back(descriptors_mat);
  db_keypoints_3d_.push_back(stereo_frame.keypoints_3d_);

  DLoopDetector::DetectionResult loop_result;
  loop_detector_.detectLoop(keypoints, descriptors_vec, loop_result);

  return loop_result;
}

void LoopClosureDetector::rewriteStereoFrameFeatures(
    StereoFrame& stereo_frame, const std::vector<cv::KeyPoint>& keypoints) {
  // Populate frame keypoints with ORB features instead of the normal
  // VIO features that came with the StereoFrame.

  // TODO: is this thread safe? Should I copy the stereo_frame object first?
  Frame* left_frame_mutable = stereo_frame.getLeftFrameMutable();
  Frame* right_frame_mutable = stereo_frame.getRightFrameMutable();

  // Clear all relevant fields.
  left_frame_mutable->keypoints_.clear();
  left_frame_mutable->versors_.clear();
  left_frame_mutable->scores_.clear();
  right_frame_mutable->keypoints_.clear();
  right_frame_mutable->versors_.clear();
  right_frame_mutable->scores_.clear();
  stereo_frame.keypoints_3d_.clear();
  stereo_frame.keypoints_depth_.clear();
  stereo_frame.left_keypoints_rectified_.clear();
  stereo_frame.right_keypoints_rectified_.clear();

  // Reserve space in all relevant fields
  left_frame_mutable->keypoints_.reserve(keypoints.size());
  left_frame_mutable->versors_.reserve(keypoints.size());
  left_frame_mutable->scores_.reserve(keypoints.size());
  right_frame_mutable->keypoints_.reserve(keypoints.size());
  right_frame_mutable->versors_.reserve(keypoints.size());
  right_frame_mutable->scores_.reserve(keypoints.size());
  stereo_frame.keypoints_3d_.reserve(keypoints.size());
  stereo_frame.keypoints_depth_.reserve(keypoints.size());
  stereo_frame.left_keypoints_rectified_.reserve(keypoints.size());
  stereo_frame.right_keypoints_rectified_.reserve(keypoints.size());

  stereo_frame.setIsRectified(false);

  // Add ORB keypoints.
  for (unsigned int i=0; i<keypoints.size(); i++) {
    // TODO: additional requirements for converting cv::KeyPoint to KeypointCV?
    left_frame_mutable->keypoints_.push_back(keypoints[i].pt);
    left_frame_mutable->versors_.push_back(Frame::CalibratePixel(keypoints[i].pt,
        left_frame_mutable->cam_param_));
    left_frame_mutable->scores_.push_back(1.0); // TODO: is this a max score? Is that the right thing to do?
  }

  // Automatically match keypoints in right image with those in left.
  stereo_frame.sparseStereoMatching();
}

gtsam::Pose3 LoopClosureDetector::computePoseStereoRansac(size_t query,
    size_t match) {
  // Vector of 3D vectors
  // TODO: is there anyway to reserve the correct space for these?
  Points3d f_ref; // TODO: find out what this type really is
  Points3d f_cur;

  // Get two best matches between frame keypoints
  std::vector<std::vector<cv::DMatch>> matches;
  orb_feature_matcher_->knnMatch(db_descriptors_[query], db_descriptors_[match],
      matches, 2);

  // Keep only the best matches using Lowe's ratio test and fill point clouds
  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance < lcd_params_.lowe_ratio_ *
      matches[i][1].distance) {
        f_ref.push_back(db_keypoints_3d_[match].at(matches[i][0].trainIdx));
        f_cur.push_back(db_keypoints_3d_[query].at(matches[i][0].queryIdx));
    }
  }

  // TODO: make a working version of the old method for using 2d rotation and dotting vectors
  // TODO: make methods for each of these and also for the MonoGivenRot option
  // Setup problem (3D-3D adapter) -
  // http://laurentkneip.github.io/opengv/page_how_to_use.html
  AdapterStereo adapter(f_ref, f_cur);
  std::shared_ptr<ProblemStereo> problem = std::make_shared<ProblemStereo>(
      adapter, lcd_params_.ransac_randomize_stereo_);
  opengv::sac::Ransac<ProblemStereo> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = lcd_params_.ransac_threshold_stereo_;
  ransac.max_iterations_ = lcd_params_.max_ransac_iterations_stereo_;
  ransac.probability_ = lcd_params_.ransac_probability_stereo_;

  if (!ransac.computeModel(0)) {
    LOG(ERROR) << "LoopClosureDetector Failure: RANSAC could not solve.";
    return gtsam::Pose3();
  }

  // TODO: check for >50% inliers
  // std::cout << "ransac inliers: " << ransac.inliers_.size() << std::endl;
  // TODO: check quality of result like in Tracker::geometricOutlierRejectionStereo

  opengv::transformation_t best_transformation = ransac.model_coefficients_;
  return UtilsOpenCV::Gvtrans2pose(best_transformation);
}

// TODO: lots of copied pasted code here. Make it neater

gtsam::Pose3 LoopClosureDetector::computePoseStereoNonlinearOpt(size_t query,
    size_t match) {
  Points3d f_ref;
  Points3d f_cur;

  // Get two best matches between frame keypoints
  std::vector<std::vector<cv::DMatch>> matches;
  orb_feature_matcher_->knnMatch(db_descriptors_[query], db_descriptors_[match],
      matches, 2);

  // Keep only the best matches using Lowe's ratio test and fill point clouds
  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance < lcd_params_.lowe_ratio_ *
      matches[i][1].distance) {
        f_ref.push_back(db_keypoints_3d_[match].at(matches[i][0].trainIdx));
        f_cur.push_back(db_keypoints_3d_[query].at(matches[i][0].queryIdx));
    }
  }

  AdapterStereo adapter(f_ref, f_cur );
  opengv::transformation_t nonlinear_transformation =
      opengv::point_cloud::optimize_nonlinear(adapter);

  return UtilsOpenCV::Gvtrans2pose(nonlinear_transformation);
}

// TODO: this is required for mono SLAM, find a better way.
gtsam::Pose3 LoopClosureDetector::computePoseMonoRansac(size_t query,
    size_t match) {
  // Points3d f_ref;
  // Points3d f_cur;
  //
  // // Get two best matches between frame keypoints
  // std::vector<std::vector<cv::DMatch>> matches;
  // orb_feature_matcher_->knnMatch(db_descriptors_[query], db_descriptors_[match],
  //     matches, 2);
  //
  // // Keep only the best matches using Lowe's ratio test and fill point clouds
  // for (size_t i=0; i<matches.size(); i++) {
  //   if (matches[i][0].distance < lcd_params_.lowe_ratio_ *
  //     matches[i][1].distance) {
  //       f_ref.push_back(db_keypoints_3d_[match].at(matches[i][0].trainIdx));
  //       f_cur.push_back(db_keypoints_3d_[query].at(matches[i][0].queryIdx));
  //   }
  // }
  //
  // // TODO: this requires bearingVectors, not point clouds.
  // AdapterMono adapter(f_ref, f_cur);
  // std::shared_ptr<ProblemMono> problem = std::make_shared<ProblemMono>(
  //     adapter, ProblemMono::NISTER);
  // opengv::sac::Ransac<ProblemMono> ransac;
  // ransac.sac_model_ = problem;
  // ransac.threshold_ = lcd_params_.ransac_threshold_stereo_;
  // ransac.max_iterations_ = lcd_params_.max_ransac_iterations_stereo_;
  // ransac.probability_ = lcd_params_.ransac_probability_stereo_;
  //
  // if (!ransac.computeModel(0)) {
  //   LOG(ERROR) << "LoopClosureDetector Failure: RANSAC could not solve.";
  //   return gtsam::Pose3();
  // }
  //
  // opengv::transformation_t best_transformation = ransac.model_coefficients_;
  // return UtilsOpenCV::Gvtrans2pose(best_transformation);
}

gtsam::Pose3 LoopClosureDetector::computePoseStereoGiven2D(size_t query,
    size_t match, gtsam::Pose3& pose_2d) {
  // Both of these are in the 'ref' frame, which is the match frame
  gtsam::Rot3 R = pose_2d.rotation();
  gtsam::Point3 unit_t = pose_2d.translation();

  // Get matching point clouds between the two frames
  Points3d f_ref;
  Points3d f_cur;

  // Get two best matches between frame keypoints
  std::vector<std::vector<cv::DMatch>> matches;
  orb_feature_matcher_->knnMatch(db_descriptors_[query], db_descriptors_[match],
      matches, 2);

  // Keep only the best matches using Lowe's ratio test and fill point clouds
  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance < lcd_params_.lowe_ratio_ *
      matches[i][1].distance) {
        f_ref.push_back(db_keypoints_3d_[match].at(matches[i][0].trainIdx));
        f_cur.push_back(db_keypoints_3d_[query].at(matches[i][0].queryIdx));
    }
  }

  // Get sacling factor for translation by averaging across point cloud
  double scaling_factor = 0.0;
  for (size_t i=0; i<f_ref.size(); i++) {
    gtsam::Vector3 keypoint_ref = f_ref[i];
    gtsam::Vector3 keypoint_cur = f_cur[i];

    gtsam::Vector3 rotated_keypoint_diff = (R*keypoint_ref) - keypoint_cur;
    scaling_factor += rotated_keypoint_diff.dot(unit_t);
  }
  scaling_factor /= f_ref.size();

  gtsam::Point3 scaled_t = gtsam::Point3(unit_t[0] * scaling_factor,
      unit_t[1] * scaling_factor, unit_t[2] * scaling_factor);

  return gtsam::Pose3(R, scaled_t);
}

gtsam::Pose3 LoopClosureDetector::mat2pose(const cv::Mat& R,
    const cv::Mat& t) {
  /* Return rotation between the two matches and translation unit vector */

  // TODO: Almost certainly you have the pose backwards. It seems like it's pose from ref to curr,
  // but you don't have access to ref so you need to invert (?)
  // TODO: opencv seems to calculate movement of points, not of camera. In this case you might have to invert?
  std::vector<gtsam::Point3> cols(3);
  for (unsigned int i=0; i<3; i++) {
    gtsam::Point3 col = gtsam::Point3(R.at<double>(0,i),
      R.at<double>(1,i),
      R.at<double>(2,i));
    cols[i] = col;
  }
  gtsam::Rot3 rot = gtsam::Rot3(cols[0], cols[1], cols[2]);
  gtsam::Rot3 rot_inv = rot.inverse();

  gtsam::Vector3 unit_T = gtsam::Vector3(t.at<double>(0), t.at<double>(1),
      t.at<double>(2));

  // std::cout << "pose: " << gtsam::Pose3(rot_inv, unit_T);
  // TODO: verify that it's rot_inv and not rot
  return gtsam::Pose3(rot_inv, unit_T);
}

void LoopClosureDetector::computeAndDrawMatchesBetweenFrames(
    const cv::Mat& query_img, const cv::Mat& match_img, const size_t& query_id,
    const size_t& match_id) {
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> good_matches;

  orb_feature_matcher_->knnMatch(db_descriptors_[query_id],
      db_descriptors_[match_id], matches, 2);

  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance <
        lcd_params_.lowe_ratio_ * matches[i][1].distance) {
      good_matches.push_back(matches[i][0]);
    }
  }

  // Draw matches
  cv::Mat img_matches;
  cv::drawMatches(query_img, db_keypoints_[query_id], match_img,
      db_keypoints_[match_id], good_matches, img_matches, cv::Scalar(255, 0, 0),
      cv::Scalar(255, 0, 0));
  cv::imshow("Good Matches", img_matches );
  cv::waitKey();
}

} // namespace VIO
