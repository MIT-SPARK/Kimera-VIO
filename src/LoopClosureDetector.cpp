/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.cpp
 * @brief  Pipeline for detection and reporting of Loop Closures between frames
 * @author Marcus Abate, Luca Carlone
 */

#include "LoopClosureDetector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

// TODO: add statistics reporting for verbosity_>=3
/** Verbosity settings: (cumulative with every increase in level)
      0: Runtime errors and warnings, spin start and frequency are reported.
      1: Loop closure detections are reported as warnings.
      2: Loop closure failures are reported as errors.
      3: Statistics are reported at relevant steps.
**/
static const int VERBOSITY = 0;

namespace VIO {

LoopClosureDetector::LoopClosureDetector(
    const LoopClosureDetectorParams& lcd_params,
    const bool log_output)
    : lcd_params_(lcd_params),
      log_output_(log_output),
      set_intrinsics_(false),
      latest_bowvec_(DBoW2::BowVector()),
      latest_matched_island_(MatchIsland()),
      latest_query_id_(FrameId(0)),
      temporal_entries_(0) {
  // Initialize the ORB feature detector object:
  orb_feature_detector_ = cv::ORB::create(lcd_params_.nfeatures_,
      lcd_params_.scale_factor_, lcd_params_.nlevels_,
      lcd_params_.edge_threshold_, lcd_params_.first_level_, lcd_params_.WTA_K_,
      lcd_params_.score_type_, lcd_params_.patch_sze_,
      lcd_params_.fast_threshold_);

  // Initialize our feature matching object:
  orb_feature_matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

  // Load ORB vocabulary:
  std::cout << "Loading vocabulary from "
            << lcd_params_.vocabulary_path_
            << std::endl;
  vocab_.loadFromTextFile(lcd_params_.vocabulary_path_); // TODO: support .gz files
  std::cout << "Loaded vocabulary with "
            << vocab_.size() << " visual words."
            << std::endl;

  // Initialize db_BoW_ and db_frames_:
  setVocabulary(vocab_);
  db_frames_.clear();
}

bool LoopClosureDetector::spin(
    ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
    ThreadsafeQueue<LoopClosureDetectorOutputPayload>& output_queue,
    bool parallel_run) {
  LOG(INFO) << "Spinning LoopClosureDetector.";
  utils::StatsCollector stat_lcd_timing("LoopClosureDetector Timing [ms]");

  while (!shutdown_) {
    // Get input keyframe.
    is_thread_working_ = false;
    std::shared_ptr<LoopClosureDetectorInputPayload> input =
        input_queue.popBlocking();
    is_thread_working_ = true;

    if (input) {
      auto tic = utils::Timer::tic();
      LoopClosureDetectorOutputPayload output_payload = spinOnce(input);

      // Only push output if it's a detected loop.
      if (output_payload.is_loop_) {
        output_queue.push(output_payload);
      }

      auto spin_duration = utils::Timer::toc(tic).count();
      stat_lcd_timing.AddSample(spin_duration);

      LOG(WARNING) << "Current LoopClosureDetector frequency: "
                   << 1000.0 / spin_duration << " Hz. ("
                   << spin_duration << " ms).";

    }
    else {
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

  // One time initialization from camera parameters.
  if (!set_intrinsics_) {
    setIntrinsics(input->stereo_frame_);
  }

  // Process the StereoFrame and check for a loop closure with previous ones.
  StereoFrame working_frame = input->stereo_frame_;
  LoopClosureDetectorOutputPayload output_payload = checkLoopClosure(
      working_frame);

  return output_payload;
}

LoopClosureDetectorOutputPayload LoopClosureDetector::checkLoopClosure(
    StereoFrame& stereo_frame) {
  FrameId frame_id = processAndAddFrame(stereo_frame);
  LoopResult loop_result = detectLoop(frame_id);

  if (loop_result.isLoop()) {
    if (VERBOSITY >= 1) {
      LOG(ERROR) << "LoopClosureDetector: LOOP CLOSURE detected from image "
                 << loop_result.match_id_ << " to image "
                 << loop_result.query_id_;
    }

    gtsam::Pose3 pose_mono = loop_result.relative_pose_mono_;
    gtsam::Pose3 pose_stereo = compute3DPose(loop_result.query_id_,
        loop_result.match_id_, pose_mono);
    gtsam::Point3 t = pose_stereo.translation();
    gtsam::Rot3 R(pose_stereo.rotation());

    if (lcd_params_.use_mono_rot_) {
      gtsam::Pose3 pose_mono_body;
      transformCameraPose2BodyPose(pose_mono, &pose_mono_body);
      R = gtsam::Rot3(pose_mono_body.rotation());
    }

    LoopClosureDetectorOutputPayload output_payload(true,
      stereo_frame.getTimestamp(),
      db_frames_[loop_result.match_id_].id_kf_,
      db_frames_[loop_result.query_id_].id_kf_,
      gtsam::Pose3(R, t));

    return output_payload;
  }
  else {
    if (VERBOSITY >= 2) {
      std::string erhdr = "LoopClosureDetector Failure Reason: ";
      switch (loop_result.status_) {
        case 0: LOG(ERROR) << erhdr+"Loop detected.";
                break;
        case 1: LOG(ERROR) << erhdr+"No matches against the database.";
                break;
        case 2: LOG(ERROR) << erhdr+"Current image score vs previous too low.";
                break;
        case 3: LOG(ERROR) << erhdr+"Scores were below the alpha threshold.";
                break;
        case 4: LOG(ERROR) << erhdr+"Not enough matches to create groups.";
                break;
        case 5: LOG(ERROR) << erhdr+"No temporal consistency between matches.";
                break;
        case 6: LOG(ERROR) << erhdr+"The geometric verification step failed.";
                break;
      }
    }

    return LoopClosureDetectorOutputPayload(false,-1,0,0,gtsam::Pose3());
  }
}

FrameId LoopClosureDetector::processAndAddFrame(
    StereoFrame& stereo_frame) {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Mat> descriptors_vec;
  cv::Mat descriptors_mat;

  // Extract ORB features and construct descriptors_vec.
  orb_feature_detector_->detectAndCompute(stereo_frame.getLeftFrame().img_,
      cv::Mat(), keypoints, descriptors_mat);

  int L = orb_feature_detector_->descriptorSize();
  descriptors_vec.resize(descriptors_mat.size().height);

  for (unsigned int i=0; i<descriptors_vec.size(); i++) {
    descriptors_vec[i] = cv::Mat(1, L, descriptors_mat.type()); // one row only
    descriptors_mat.row(i).copyTo(descriptors_vec[i].row(0));
  }

  // Fill StereoFrame with ORB keypoints and perform stereo matching.
  rewriteStereoFrameFeatures(stereo_frame, keypoints);

  // Build and store LCDFrame object.
  LCDFrame cur_frame = LCDFrame(
      stereo_frame.getTimestamp(), db_frames_.size(),
      stereo_frame.getFrameId(), keypoints, stereo_frame.keypoints_3d_,
      descriptors_vec, descriptors_mat, stereo_frame.getLeftFrame().versors_);

  db_frames_.push_back(cur_frame);

  return cur_frame.id_;
}

LoopResult LoopClosureDetector::detectLoop(size_t frame_id) {
  LoopResult result;
  result.query_id_ = frame_id;

  DBoW2::BowVector bow_vec;

  // Create BOW representation of descriptors.
  // TODO: implement direct indexing, if needed
  db_BoW_.getVocabulary()->transform(db_frames_[frame_id].descriptors_vec_,
      bow_vec);

  int max_possible_match_id = frame_id - lcd_params_.dist_local_;
  if (max_possible_match_id < 0) max_possible_match_id = 0;

  // Query for BoW vector matches in database.
  DBoW2::QueryResults query_result;
  db_BoW_.query(bow_vec, query_result, lcd_params_.max_db_results_,
      max_possible_match_id);

  // Add current BoW vector to database.
  db_BoW_.add(bow_vec);

  if(query_result.empty()) {
    result.status_ = LCDStatus::NO_MATCHES;
  }
  else {
    double nss_factor = 1.0;
    if (lcd_params_.use_nss_) {
      nss_factor = db_BoW_.getVocabulary()->score(bow_vec, latest_bowvec_);
    }

    if (lcd_params_.use_nss_ && nss_factor < lcd_params_.min_nss_factor_) {
      result.status_ = LCDStatus::LOW_NSS_FACTOR;
    }
    else {
      // Remove low scores from the QueryResults based on nss.
      DBoW2::QueryResults::iterator query_it = lower_bound(query_result.begin(),
          query_result.end(), DBoW2::Result(0,lcd_params_.alpha_*nss_factor),
          DBoW2::Result::geq);
      if (query_it != query_result.end()) {
        query_result.resize(query_it - query_result.begin());
      }

      // Begin grouping and checking matches.
      if (query_result.empty()) {
        result.status_ = LCDStatus::LOW_SCORE;
      }
      else {
        // Set best candidate to highest scorer.
        result.match_id_ = query_result[0].Id;

        // Compute islands in the matches.
        std::vector<MatchIsland> islands;
        computeIslands(query_result, islands);

        if (islands.empty()) {
          result.status_ = LCDStatus::NO_GROUPS;
        }
        else {
          // Find the best island grouping using MatchIsland sorting.
          const MatchIsland& best_island = *std::max_element(islands.begin(),
              islands.end());

          // Run temporal constraint check on this best island.
          bool passTemporalConstraint = checkTemporalConstraint(frame_id,
              best_island);

          latest_matched_island_ = best_island;
          latest_query_id_ = frame_id;

          if (!passTemporalConstraint) {
            result.status_ = LCDStatus::FAILED_TEMPORAL_CONSTRAINT;
          }

          else {
            // Perform geometric verification check.
            bool passGeometricVerification = geometricVerificationCheck(
                frame_id, best_island.best_id_, result);

            if (passGeometricVerification) {
              result.status_ = LCDStatus::LOOP_DETECTED;
            }
            else {
              result.status_ = LCDStatus::FAILED_GEOM_VERIFICATION;
            }
          }
        }
      }
    }
  }

  // Update latest bowvec for normalized similarity scoring (NSS).
  if ((int)(frame_id + 1) > lcd_params_.dist_local_) {
    latest_bowvec_ = bow_vec;
  }

  return result;
}

bool LoopClosureDetector::geometricVerificationCheck(const FrameId& query_id,
    const FrameId& match_id, LoopResult& result) {
  switch (lcd_params_.geom_check_) {
    case GeomVerifOption::TEMP_CV:
      return geometricVerification_cv(query_id, match_id, result);
    case GeomVerifOption::NISTER:
      return geometricVerification_gv(query_id, match_id, result);
    case GeomVerifOption::NONE:
      return true;
  }

  return false;
}

gtsam::Pose3 LoopClosureDetector::compute3DPose(const FrameId& query_id,
    const FrameId& match_id, gtsam::Pose3& pose_2d) {
  switch (lcd_params_.pose_recovery_option_) {
    case PoseRecoveryOption::RANSAC_3PT:
      return compute3DPose3pt(query_id, match_id);
      break;
    case PoseRecoveryOption::GIVEN_ROT:
      return compute3DPoseGiven2D(query_id, match_id, pose_2d);
      break;
  }

  return gtsam::Pose3();
}

// TODO: should this be parsed from the other VIO components' params?
void LoopClosureDetector::setIntrinsics(const StereoFrame& stereo_frame) {
  lcd_params_.focal_length_ =
      stereo_frame.getLeftFrame().cam_param_.intrinsics_[0];
  lcd_params_.principle_point_ = cv::Point2d(
      stereo_frame.getLeftFrame().cam_param_.intrinsics_[2],
      stereo_frame.getLeftFrame().cam_param_.intrinsics_[3]);

  B_Pose_camLrect_ = stereo_frame.getBPoseCamLRect();
  set_intrinsics_ = true;
}

void LoopClosureDetector::setDatabase(const OrbDatabase& db) {
  db_BoW_ = OrbDatabase(db);
  clear();
}

// TODO: can pass direct indexing bool and levels here.
void LoopClosureDetector::setVocabulary(const OrbVocabulary& vocab) {
  db_BoW_ = OrbDatabase(vocab);
}

void LoopClosureDetector::allocate(size_t n) {
  // Reserve frames
  if (n > db_frames_.size()) {
    db_frames_.resize(n);
  }

  // Reserve size for keypoints and descriptors of each expected frame
  for (size_t i=0; i<db_frames_.size(); i++) {
    db_frames_[i].keypoints_.reserve(lcd_params_.nfeatures_);
    db_frames_[i].keypoints_3d_.reserve(lcd_params_.nfeatures_);
    db_frames_[i].descriptors_vec_.reserve(lcd_params_.nfeatures_);
    db_frames_[i].versors_.reserve(lcd_params_.nfeatures_);
  }

  db_BoW_.allocate(n, lcd_params_.nfeatures_);
}

void LoopClosureDetector::clear() {
  db_BoW_.clear();
  db_frames_.clear();
  latest_bowvec_.clear();
  latest_matched_island_.clear();
  latest_query_id_ = 0;
  temporal_entries_ = 0;
}

void LoopClosureDetector::print() const {
  // TODO: implement
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

cv::Mat LoopClosureDetector::computeAndDrawMatchesBetweenFrames(
    const cv::Mat& query_img, const cv::Mat& match_img, const size_t& query_id,
    const size_t& match_id, bool cut_matches) {
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> good_matches;

  // Use the Lowe's Ratio Test only if asked.
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  // TODO: this can use computeMatchedIndices() as well...
  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
      db_frames_[match_id].descriptors_mat_, matches, 2);

  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance < lowe_ratio * matches[i][1].distance) {
      good_matches.push_back(matches[i][0]);
    }
  }

  // Draw matches.
  cv::Mat img_matches;
  cv::drawMatches(query_img, db_frames_[query_id].keypoints_, match_img,
      db_frames_[match_id].keypoints_, good_matches, img_matches,
      cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 0));

  return img_matches;
}

gtsam::Pose3 LoopClosureDetector::mat2pose(const cv::Mat& R,
    const cv::Mat& t) {
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

  // TODO: verify that it's rot_inv and not rot
  return gtsam::Pose3(rot, unit_T);
}

// TODO: rewrite to be less like DLoopDetector
bool LoopClosureDetector::checkTemporalConstraint(const FrameId& id,
    const MatchIsland& island) {
  if (temporal_entries_ == 0 ||
      int(id - latest_query_id_) > lcd_params_.max_distance_between_queries_) {
    temporal_entries_ = 1;
  }
  else {
    // Check whether current island encloses latest island or vice versa
    bool curEnclosesOld = island.start_id_<=latest_matched_island_.start_id_ &&
          latest_matched_island_.start_id_<=island.end_id_;
    bool oldEnclosesCur = latest_matched_island_.start_id_<=island.start_id_ &&
          island.start_id_<=latest_matched_island_.end_id_;

    bool passGroupDistConstraint = curEnclosesOld || oldEnclosesCur;
    if(!passGroupDistConstraint) {
      int d1 = (int)latest_matched_island_.start_id_ - (int)island.end_id_;
      int d2 = (int)island.start_id_ - (int)latest_matched_island_.end_id_;

      int gap;
      if (d1 > d2) {
        gap = d1;
      }
      else {
        gap = d2;
      }

      passGroupDistConstraint = gap <= lcd_params_.max_distance_between_groups_;
    }

    if(passGroupDistConstraint) {
      temporal_entries_++;
    }
    else {
      temporal_entries_ = 1;
    }
  }

  return temporal_entries_ > lcd_params_.min_temporal_matches_;
}

// TODO: rewrite to be less like DLoopDetector
void LoopClosureDetector::computeIslands(DBoW2::QueryResults& q,
    std::vector<MatchIsland>& islands) const {
  islands.clear();

  // The case of one island is easy to compute and is done separately
  if (q.size() == 1) {
    MatchIsland island(q[0].Id, q[0].Id, q[0].Score);
    island.best_id_ = q[0].Id;
    island.best_score_ = q[0].Score;
    islands.push_back(island);
  }

  else if (!q.empty()) {
    // sort query results in ascending order of ids
    std::sort(q.begin(), q.end(), DBoW2::Result::ltId);

    // create long enough islands
    DBoW2::QueryResults::const_iterator dit = q.begin();
    int first_island_entry = dit->Id;
    int last_island_entry = dit->Id;

    // these are indices of q
    unsigned int i_first = 0;
    unsigned int i_last = 0;

    double best_score = dit->Score;
    FrameId best_entry = dit->Id;

    ++dit;
    for(unsigned int idx = 1; dit != q.end(); ++dit, ++idx)
    {
      if((int)dit->Id - last_island_entry < lcd_params_.max_intragroup_gap_)
      {
        // go on until find the end of the island
        last_island_entry = dit->Id;
        i_last = idx;
        if(dit->Score > best_score)
        {
          best_score = dit->Score;
          best_entry = dit->Id;
        }
      }
      else
      {
        // end of island reached
        int length = last_island_entry - first_island_entry + 1;
        if(length >= lcd_params_.min_matches_per_group_)
        {
          MatchIsland island = MatchIsland(first_island_entry,
              last_island_entry, computeIslandScore(q, i_first, i_last));

          islands.push_back(island);
          islands.back().best_score_ = best_score;
          islands.back().best_id_ = best_entry;
        }

        // prepare next island
        first_island_entry = last_island_entry = dit->Id;
        i_first = i_last = idx;
        best_score = dit->Score;
        best_entry = dit->Id;
      }
    }
    // add last island
    if(last_island_entry - first_island_entry + 1 >=
      lcd_params_.min_matches_per_group_)
    {
      MatchIsland island = MatchIsland(first_island_entry,
          last_island_entry, computeIslandScore(q, i_first, i_last));

      islands.push_back(island);
      islands.back().best_score_ = best_score;
      islands.back().best_id_ = best_entry;
    }
  }
}

double LoopClosureDetector::computeIslandScore(const DBoW2::QueryResults& q,
    const FrameId& start_id, const FrameId& end_id) const {
  double score_sum = 0.0;
  for (FrameId id=start_id; id<=end_id; id++) {
    score_sum += q[id].Score;
  }

  return score_sum;
}

// TODO: make sure cv::DescriptorMatcher doesn't store descriptors and match against them later
void LoopClosureDetector::computeMatchedIndices(const FrameId& query_id,
    const FrameId& match_id, std::vector<unsigned int>& i_query,
    std::vector<unsigned int>& i_match, bool cut_matches) {
  // Get two best matches between frame descriptors.
  std::vector<std::vector<cv::DMatch>> matches;
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
      db_frames_[match_id].descriptors_mat_, matches, 2);

  // TODO: any way to reserve space ahead of time? even if it's over-reserved
  // Keep only the best matches using Lowe's ratio test and store indicies.
  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance < lowe_ratio * matches[i][1].distance) {
      i_query.push_back(matches[i][0].queryIdx);
      i_match.push_back(matches[i][0].trainIdx);
    }
  }
}

bool LoopClosureDetector::geometricVerification_cv(const FrameId& query_id,
    const FrameId& match_id, LoopResult& result) {
  // Find correspondences between keypoints.
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match, true);

  std::vector<cv::Point2f> query_points, match_points;

  // Build vectors of matched keypoints.
  for (size_t i=0; i<i_match.size(); i++) {
    query_points.push_back(db_frames_[query_id].keypoints_[i_query[i]].pt);
    match_points.push_back(db_frames_[match_id].keypoints_[i_match[i]].pt);
  }

  // Calculate essential matrix between 2D correspondences.
  if ((int)match_points.size() >= lcd_params_.min_correspondences_) {
    cv::Mat queryMat(query_points.size(), 2, CV_32F, &query_points[0]);
    cv::Mat matchMat(match_points.size(), 2, CV_32F, &match_points[0]);

    // TODO: figure out ordering: queryMat first or matchMat first?
    // TODO: decide on LMEDS vs RANSAC and tune parameters

    cv::Mat E = cv::findEssentialMat(queryMat, matchMat,
        lcd_params_.focal_length_, lcd_params_.principle_point_, cv::RANSAC,
        lcd_params_.ransac_probability_mono_,
        lcd_params_.ransac_threshold_mono_);

    std::cout << "found E:\n" << E << std::endl;

    if (!E.empty()) {
      // 2D pose recovery.
      cv::Mat R, T;
      cv::recoverPose(E, queryMat, matchMat, R, T, lcd_params_.focal_length_,
          lcd_params_.principle_point_);

      // result.relative_pose_2d_ = mat2pose(R, T);
      gtsam::Pose3 camRef_T_camCur = mat2pose(R, T);
      // camRef_T_camCur = camRef_T_camCur.inverse();
      // transformCameraPose2BodyPose(camRef_T_camCur, &result.relative_pose_2d_);
      result.relative_pose_mono_ = camRef_T_camCur;

      return true;
    }
  }

  return false;
}

// TODO: both geometrticVerification and compute3DPose run the matching alg
// this is wasteful. Store the matched indices as latest for use in the compute step
// TODO: This is the camera frame. Your transform has to happen later
bool LoopClosureDetector::geometricVerification_gv(const FrameId& query_id,
    const FrameId& match_id, LoopResult& result) {
  // Find correspondences between keypoints.
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match, true);

  BearingVectors query_versors, match_versors;
  std::vector<cv::Point2f> query_points, match_points;

  for (size_t i=0; i<i_match.size(); i++) {
    query_versors.push_back(db_frames_[query_id].versors_[i_query[i]]);
    match_versors.push_back(db_frames_[match_id].versors_[i_match[i]]);
  }

  // Recover relative pose between frames, with translation up to a scalar.
  if ((int)match_versors.size() >= lcd_params_.min_correspondences_) {
    AdapterMono adapter(match_versors, query_versors);

    // Use RANSAC to solve the central-relative-pose problem.
    opengv::sac::Ransac<SacProblemMono> ransac;
    std::shared_ptr<SacProblemMono> relposeproblem_ptr(
        new SacProblemMono(adapter, SacProblemMono::Algorithm::NISTER,
            lcd_params_.ransac_randomize_mono_));

    ransac.sac_model_ = relposeproblem_ptr;
    ransac.max_iterations_ = lcd_params_.max_ransac_iterations_mono_;
    ransac.probability_ = lcd_params_.ransac_probability_mono_;
    ransac.threshold_ = lcd_params_.ransac_threshold_mono_;

    // Compute transformation via RANSAC.
    if (!ransac.computeModel()) {
      LOG(ERROR)
          << "LoopClosureDetector Failure: RANSAC 5pt could not solve.";
    }
    else {
      opengv::transformation_t transformation = ransac.model_coefficients_;

      gtsam::Pose3 camRef_T_camCur = UtilsOpenCV::Gvtrans2pose(transformation);
      result.relative_pose_mono_ = camRef_T_camCur;

      // TODO: return statistics on the ransac.
      std::cout << "ransac inliers: " << ransac.inliers_.size()
                << "\niterations: " << ransac.iterations_
                << "\nsize of input: " << query_versors.size()
                << std::endl;

      return true;
    }
  }

  return false;
}

void LoopClosureDetector::transformCameraPose2BodyPose(
    gtsam::Pose3& camRef_T_camCur, gtsam::Pose3* bodyRef_T_bodyCur) const {
  // TODO: is this the right way to set an object without returning? @toni
  *bodyRef_T_bodyCur = B_Pose_camLrect_ * camRef_T_camCur *
    B_Pose_camLrect_.inverse();
}

void LoopClosureDetector::transformBodyPose2CameraPose(
    gtsam::Pose3& bodyRef_T_bodyCur, gtsam::Pose3*camRef_T_camCur) const {
  *camRef_T_camCur = B_Pose_camLrect_.inverse() * bodyRef_T_bodyCur *
    B_Pose_camLrect_;
}

gtsam::Pose3 LoopClosureDetector::compute3DPose3pt(const FrameId& query_id,
    const FrameId& match_id) {
  // Find correspondences between frames.
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match, false);

  // TODO: is there any way to reserve the correct space for these?
  Points3d f_ref;
  Points3d f_cur;

  // Fill point clouds with matched 3D keypoints.
  for (size_t i=0; i<i_match.size(); i++) {
    f_cur.push_back(db_frames_[query_id].keypoints_3d_.at(i_query[i]));
    f_ref.push_back(db_frames_[match_id].keypoints_3d_.at(i_match[i]));
  }

  AdapterStereo adapter(f_ref, f_cur);
  opengv::transformation_t transformation;

  // Compute transform using RANSAC 3-point method (Arun).
  std::shared_ptr<SacProblemStereo> ptcloudproblem_ptr(
      new SacProblemStereo(adapter, lcd_params_.ransac_randomize_stereo_));
  opengv::sac::Ransac<SacProblemStereo> ransac;
  ransac.sac_model_ = ptcloudproblem_ptr;
  ransac.max_iterations_ = lcd_params_.max_ransac_iterations_stereo_;
  ransac.probability_ = lcd_params_.ransac_probability_stereo_;
  ransac.threshold_ = lcd_params_.ransac_threshold_stereo_;

  if (!ransac.computeModel()) {
    LOG(ERROR) << "LoopClosureDetector Failure: RANSAC could not solve.";
  }
  else {
    // TODO: return statistics on the ransac.
    std::cout << "ransac inliers: " << ransac.inliers_.size()
              << "\niterations: " << ransac.iterations_
              << "\nsize of input: " << f_ref.size()
              << std::endl;

    // TODO: check for >50% inliers on all methods

    transformation = ransac.model_coefficients_;

    // Transform pose from camera frame to body frame.
    gtsam::Pose3 bodyRef_T_bodyCur;
    gtsam::Pose3 camRef_T_camCur = UtilsOpenCV::Gvtrans2pose(transformation);
    transformCameraPose2BodyPose(camRef_T_camCur, &bodyRef_T_bodyCur);

    return bodyRef_T_bodyCur;
  }

  return gtsam::Pose3();
}

// TODO: Add median check coordiante wise instead of the other check
// TODO: rename pose_2d to pose_cam or something for cam frame
gtsam::Pose3 LoopClosureDetector::compute3DPoseGiven2D(const FrameId& query_id,
    const FrameId& match_id, gtsam::Pose3& pose_2d) {
  gtsam::Rot3 R = pose_2d.rotation();
  // TODO: input should alwasy be with unit translation, no need to check
  gtsam::Point3 unit_t = pose_2d.translation() / pose_2d.translation().norm();

  // Find correspondences between frames.
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match, true);

  // TODO: is there anyway to reserve the correct space for these?
  Points3d f_ref;
  Points3d f_cur;

  // Fill point clouds with matched 3D keypoints.
  for (size_t i=0; i<i_match.size(); i++) {
    f_cur.push_back(db_frames_[query_id].keypoints_3d_.at(i_query[i]));
    f_ref.push_back(db_frames_[match_id].keypoints_3d_.at(i_match[i]));
  }

  // TODO: decide between median check and scaling factor
  std::vector<double> x_coord, y_coord, z_coord;
  for (size_t i=0; i<f_ref.size(); i++) {
    gtsam::Vector3 keypoint_ref = f_ref[i];
    gtsam::Vector3 keypoint_cur = f_cur[i];

    gtsam::Vector3 rotated_keypoint_diff = keypoint_ref - (R*keypoint_cur);
    x_coord.push_back(rotated_keypoint_diff[0]);
    y_coord.push_back(rotated_keypoint_diff[1]);
    z_coord.push_back(rotated_keypoint_diff[2]);
  }

  std::sort(x_coord.begin(), x_coord.end());
  std::sort(y_coord.begin(), y_coord.end());
  std::sort(z_coord.begin(), z_coord.end());

  gtsam::Point3 scaled_t(x_coord[int(x_coord.size()/2)],
      y_coord[int(y_coord.size()/2)],
      z_coord[int(z_coord.size()/2)]);

  // // Get sacling factor for translation by averaging across point cloud.
  // double scaling_factor = 0.0;
  // for (size_t i=0; i<f_ref.size(); i++) {
  //   gtsam::Vector3 keypoint_ref = f_ref[i];
  //   gtsam::Vector3 keypoint_cur = f_cur[i];
  //
  //   gtsam::Vector3 rotated_keypoint_diff =
  //       keypoint_ref - (R*keypoint_cur);
  //
  //   double cur_scaling_factor = rotated_keypoint_diff.dot(unit_t);
  //   if (cur_scaling_factor < 0) {
  //     cur_scaling_factor *= -1.0;
  //   }
  //   scaling_factor += cur_scaling_factor;
  // }
  // scaling_factor /= f_ref.size();
  //
  // if (scaling_factor < 0) {
  //   scaling_factor *= -1.0;
  // }
  //
  // gtsam::Point3 scaled_t(unit_t[0] * scaling_factor,
  //     unit_t[1] * scaling_factor, unit_t[2] * scaling_factor);

  // Transform pose from camera frame to body frame.
  gtsam::Pose3 camRef_T_camCur(R, scaled_t);
  gtsam::Pose3 bodyRef_T_bodyCur;
  transformCameraPose2BodyPose(camRef_T_camCur, &bodyRef_T_bodyCur);

  return bodyRef_T_bodyCur;
}

} // namespace VIO
