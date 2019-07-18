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

/** Verbosity settings: (cumulative with every increase in level)
      0: Runtime errors and warnings, spin start and frequency are reported.
      1: Loop closure detections are reported as warnings.
      2: Loop closure failures are reported as errors.
      3: Statistics are reported at relevant steps.
**/

// TODO: add statistics reporting for verbosity_>=3

namespace VIO {

LoopClosureDetector::LoopClosureDetector(
    const LoopClosureDetectorParams& lcd_params,
    const bool log_output,
    const int verbosity)
    : lcd_params_(lcd_params),
      log_output_(log_output),
      verbosity_(verbosity),
      set_intrinsics_(false),
      latest_bowvec_(DBoW2::BowVector()),
      latest_matched_island_(MatchIsland()),
      latest_query_id_(FrameId(0)),
      temporal_entries_(0) {
  // Initialize the ORB feature detector object:
  orb_feature_detector_ = cv::ORB::create(lcd_params_.nfeatures_,
      lcd_params_.scaleFactor_, lcd_params_.nlevels_,
      lcd_params_.edgeThreshold_, lcd_params_.firstLevel_, lcd_params_.WTA_K_,
      lcd_params_.scoreType_, lcd_params_.patchSize_,
      lcd_params_.fastThreshold_);

  // Initialize our feature matching object:
  orb_feature_matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

  // Load ORB vocabulary:
  std::cout << "Loading vocabulary from " << lcd_params_.vocabulary_path_
            << std::endl;
  vocab_.loadFromTextFile(lcd_params_.vocabulary_path_); // TODO: support .gz files
  std::cout << "Loaded vocabulary with " << vocab_.size() << " visual words."
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

  // One time initialization from camera parameters:
  if (!set_intrinsics_) {
    lcd_params_.focal_length_ =
      input->stereo_frame_.getLeftFrame().cam_param_.intrinsics_[0];
    lcd_params_.principle_point_ = cv::Point2d(
        input->stereo_frame_.getLeftFrame().cam_param_.intrinsics_[2],
        input->stereo_frame_.getLeftFrame().cam_param_.intrinsics_[3]);

    set_intrinsics_ = true;
  }

  StereoFrame working_frame = input->stereo_frame_;
  LoopClosureDetectorOutputPayload output_payload = checkLoopClosure(
      working_frame);

  // TODO: add LoggerMatlab options
  return output_payload;
}

FrameId LoopClosureDetector::processAndAddFrame(
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

  // TODO: ask Hank about ICP for matching random point clouds with a good
  // initial guess. That initial guess could be the rotation and unit
  // translation obtained from DBoW2...

  LCDFrame cur_frame = LCDFrame(
      stereo_frame.getTimestamp(), db_frames_.size(),
      stereo_frame.getFrameId(), keypoints, stereo_frame.keypoints_3d_,
      descriptors_vec, descriptors_mat, stereo_frame.getLeftFrame().versors_);

  db_frames_.push_back(cur_frame);

  return cur_frame.id_;
}

LoopClosureDetectorOutputPayload LoopClosureDetector::checkLoopClosure(
    StereoFrame& stereo_frame) {
  FrameId frame_id = processAndAddFrame(stereo_frame);
  LoopResult loop_result = detectLoop(frame_id);

  if (loop_result.isLoop()) {
    if (verbosity_ >= 1) {
      LOG(ERROR) << "LoopClosureDetector: LOOP CLOSURE detected from image "
                 << loop_result.match_id_ << " to image "
                 << loop_result.query_id_;
    }

    gtsam::Pose3 pose_2d = loop_result.relative_pose_2d_;
    gtsam::Pose3 pose_3d = compute3DPose(loop_result.query_id_,
        loop_result.match_id_, pose_2d);
    gtsam::Pose3 match_pose_cur(pose_3d);

    if (lcd_params_.use_2d_rot_) {
      match_pose_cur = gtsam::Pose3(pose_2d.rotation(), pose_3d.translation());
    }

    LoopClosureDetectorOutputPayload output_payload(true,
      stereo_frame.getTimestamp(), db_frames_[loop_result.match_id_].id_kf_,
      db_frames_[loop_result.query_id_].id_kf_, match_pose_cur);

    return output_payload;
  }
  else {
    if (verbosity_ >= 2) {
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

LoopResult LoopClosureDetector::detectLoop(size_t frame_id) {
  LoopResult result;
  result.query_id_ = frame_id;

  DBoW2::BowVector bow_vec;
  DBoW2::FeatureVector feat_vec;

  // Create BOW representation of descriptors
  if (lcd_params_.geom_check_ == GeomVerifOption::DIRECT_INDEX) {
    db_BoW_.getVocabulary()->transform(db_frames_[frame_id].descriptors_vec_, bow_vec,
        feat_vec, lcd_params_.di_levels_);
    db_frames_[frame_id].feat_vec_ = feat_vec; // Add to frame for geometrical verification
  }
  else {
    db_BoW_.getVocabulary()->transform(db_frames_[frame_id].descriptors_vec_, bow_vec);
  }

  // TODO: Subtract one because we've already added the frames to the databases?
  int max_possible_match_id = frame_id - lcd_params_.dist_local_;
  if (max_possible_match_id < 0) max_possible_match_id = 0;

  // Query for BoW vector matches in database
  DBoW2::QueryResults query_result;
  db_BoW_.query(bow_vec, query_result, lcd_params_.max_db_results_,
      max_possible_match_id);

  // Add current BoW vector to database
  db_BoW_.add(bow_vec, feat_vec);

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
      // Remove low scores from the QueryResults based on nss
      DBoW2::QueryResults::iterator query_it = lower_bound(query_result.begin(),
          query_result.end(), DBoW2::Result(0,lcd_params_.alpha_*nss_factor),
          DBoW2::Result::geq);
      if (query_it != query_result.end()) {
        query_result.resize(query_it - query_result.begin());
      }

      // Begin grouping and checking matches
      if (query_result.empty()) {
        result.status_ = LCDStatus::LOW_SCORE;
      }
      else {
        // Set best candidate to highest scorer
        result.match_id_ = query_result[0].Id;

        // Compute islands in the matches
        std::vector<MatchIsland> islands;
        computeIslands(query_result, islands);

        if (islands.empty()) {
          result.status_ = LCDStatus::NO_GROUPS;
        }
        else {
          // Find the best island grouping using MatchIsland sorting
          const MatchIsland& best_island = *std::max_element(islands.begin(),
              islands.end());

          // Run temporal constraint check on this best island
          bool passTemporalConstraint = checkTemporalConstraint(frame_id,
              best_island);

          latest_matched_island_ = best_island;
          latest_query_id_ = frame_id;

          if (!passTemporalConstraint) {
            result.status_ = LCDStatus::FAILED_TEMPORAL_CONSTRAINT;
          }

          else {
            // Perform geometric verification check
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

  // Update latest bowvec for normalized similarity scoring (NSS)
  if ((int)(frame_id + 1) > lcd_params_.dist_local_) {
    latest_bowvec_ = bow_vec;
  }

  return result;
}

bool LoopClosureDetector::geometricVerificationCheck(const FrameId& query_id,
    const FrameId& match_id, LoopResult& result) {

  switch (lcd_params_.geom_check_) {
    case TEMP_CV:
      return geometricVerification_NISTER_cv(query_id, match_id, result);
      break;
    // // TODO: make this whole or get rid of them:
    // case DIRECT_INDEX:
    //   return geometricVerification_DI(query_id, match_id, result);
    //   break;
    // // TODO: make this whole or get rid of them:
    // case EXHAUSTIVE:
    //   return geometricVerification_Exhaustive(query_id, match_id, result);
    //   break;
    case NONE:
      return true;
    // Use for Nister, Kneip, Seven-point, Eight-point with opengv
    default:
      return geometricVerification_gv(query_id, match_id, result);
      break;
  }
}

gtsam::Pose3 LoopClosureDetector::compute3DPose(const FrameId& query_id,
    const FrameId& match_id, gtsam::Pose3& pose_2d) {
  switch (lcd_params_.pose_recovery_option_) {
    // For RANSAC_3PT, ARUN_3PT and NONLINEAR_3PT:
    default:
      return compute3DPose_gv(query_id, match_id);
      break;
    case GIVEN_ROT:
      return compute3DPoseGiven2D(query_id, match_id, pose_2d);
      break;
  }

  return gtsam::Pose3();
}

void LoopClosureDetector::setDatabase(const OrbDatabase& db) {
  db_BoW_ = OrbDatabase(db);
  clear();
}

void LoopClosureDetector::setVocabulary(const OrbVocabulary& vocab) {
  db_BoW_ = OrbDatabase(vocab,
      (bool)(lcd_params_.geom_check_ == GeomVerifOption::DIRECT_INDEX),
      lcd_params_.di_levels_);
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
    const size_t& match_id) {
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> good_matches;

  // TODO: this can use computeMatchedIndices() as well...
  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
      db_frames_[match_id].descriptors_mat_, matches, 2);

  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance <
        lcd_params_.lowe_ratio_ * matches[i][1].distance) {
      good_matches.push_back(matches[i][0]);
    }
  }

  // Draw matches
  cv::Mat img_matches;
  cv::drawMatches(query_img, db_frames_[query_id].keypoints_, match_img,
      db_frames_[match_id].keypoints_, good_matches, img_matches,
      cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 0));

  return img_matches;
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

// TODO: Copied over from DLoopDetector almost line for line
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
    std::vector<unsigned int>& i_match) {
  // Get two best matches between frame descriptors
  std::vector<std::vector<cv::DMatch>> matches;

  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
      db_frames_[match_id].descriptors_mat_, matches, 2);

  // TODO: any way to reserve space ahead of time? even if it's over-reserved
  // Keep only the best matches using Lowe's ratio test and store indicies.
  for (size_t i=0; i<matches.size(); i++) {
    if (matches[i][0].distance < lcd_params_.lowe_ratio_ *
        matches[i][1].distance) {
      i_query.push_back(matches[i][0].queryIdx);
      i_match.push_back(matches[i][0].trainIdx);
    }
  }
}

// TODO: rename or get rid of completely
bool LoopClosureDetector::geometricVerification_NISTER_cv(const FrameId& query_id,
    const FrameId& match_id, LoopResult& result) {
  // Find correspondences between keypoints
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match);

  std::vector<cv::Point2f> query_points, match_points;

  // Build vectors of matched keypoints
  for (size_t i=0; i<i_match.size(); i++) {
    query_points.push_back(db_frames_[query_id].keypoints_[i_query[i]].pt);
    match_points.push_back(db_frames_[match_id].keypoints_[i_match[i]].pt);
  }

  // Calculate essential matrix between 2D correspondences
  if ((int)match_points.size() >= lcd_params_.min_Fpoints_) {
    cv::Mat queryMat(query_points.size(), 2, CV_32F, &query_points[0]);
    cv::Mat matchMat(match_points.size(), 2, CV_32F, &match_points[0]);

    // TODO: figure out ordering: queryMat first or matchMat first?
    // TODO: decide on LMEDS vs RANSAC and tune parameters

    cv::Mat E = cv::findEssentialMat(queryMat, matchMat,
        lcd_params_.focal_length_, lcd_params_.principle_point_, cv::LMEDS);
        // lcd_params_.ransac_probability_, lcd_params_.ransac_threshold_2d_);

    std::cout << "found E:\n" << E << std::endl;

    if (!E.empty()) {
      // 2D pose recovery:
      cv::Mat R, T;
      cv::recoverPose(E, queryMat, matchMat, R, T, lcd_params_.focal_length_,
          lcd_params_.principle_point_);

      result.relative_pose_2d_ = mat2pose(R, T);

      return true;
    }
  }

  return false;
}

// TODO: both geometrticVerification and compute3DPose run the matching alg
// this is wasteful. Store the matched indices as latest for use in the compute step

bool LoopClosureDetector::geometricVerification_gv(const FrameId& query_id,
    const FrameId& match_id, LoopResult& result) {
  // Find correspondences between keypoints
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match);

  BearingVectors query_versors, match_versors;
  std::vector<cv::Point2f> query_points, match_points;

  for (size_t i=0; i<i_match.size(); i++) {
    query_versors.push_back(db_frames_[query_id].versors_[i_query[i]]);
    match_versors.push_back(db_frames_[match_id].versors_[i_match[i]]);

    if (!lcd_params_.ransac_verification_) {
      query_points.push_back(db_frames_[query_id].keypoints_[i_query[i]].pt);
      match_points.push_back(db_frames_[match_id].keypoints_[i_match[i]].pt);
    }
  }

  // Recover relative pose between frames, with translation up to a scalar.
  if ((int)match_versors.size() >= lcd_params_.min_Fpoints_) {
    Adapter2D adapter(query_versors, match_versors);

    // Use RANSAC to solve the central-relative-pose problem.
    if (lcd_params_.ransac_verification_) {
      // Select algorithm.
      SacProblem2D::Algorithm alg;
      switch (lcd_params_.geom_check_) {
        default: {
          LOG(ERROR) << "Cannot use this alg with RANSAC.";
          alg = SacProblem2D::Algorithm::NISTER;
          break;
        }
        case NISTER: alg = SacProblem2D::Algorithm::NISTER;
          break;
        case STEWENIUS: alg = SacProblem2D::Algorithm::STEWENIUS;
          break;
        case SEVENPT: alg = SacProblem2D::Algorithm::SEVENPT;
          break;
        case EIGHTPT: alg = SacProblem2D::Algorithm::EIGHTPT;
          break;
      }

      opengv::sac::Ransac<SacProblem2D> ransac;

      // TODO: add support for choosing method from params.
      std::shared_ptr<SacProblem2D> relposeproblem_ptr(
          new SacProblem2D(adapter, alg));

      ransac.sac_model_ = relposeproblem_ptr;
      ransac.threshold_ = 3; // TODO: what should I use for this
      ransac.max_iterations_ = lcd_params_.max_ransac_iterations_;
      ransac.computeModel();

      opengv::transformation_t transformation = ransac.model_coefficients_;

      result.relative_pose_2d_ = UtilsOpenCV::Gvtrans2pose(transformation);

      return true;
    }

    // Without RANSAC we must calculate an essential matrix and decompose.
    else {
      cv::Mat queryMat(query_points.size(), 2, CV_32F, &query_points[0]);
      cv::Mat matchMat(match_points.size(), 2, CV_32F, &match_points[0]);

      opengv::essential_t E_gv;
      // TODO: need intelligent way to pick the best of E matrices instead of just the first one
      // TODO: it is possible that you need to do a cheriality check to choose the best one

      switch (lcd_params_.geom_check_) {
        default: {
          LOG(ERROR) << "Cannot use this alg without RANSAC. Defaulting to NISTER.";
          opengv::essentials_t E_mats = opengv::relative_pose::fivept_nister(
              adapter);
          E_gv = E_mats[0];
          break;
        }
        case NISTER: {
          opengv::essentials_t E_mats = opengv::relative_pose::fivept_nister(
              adapter);
          E_gv = E_mats[0];
          break;
        }
        case SEVENPT: {
          opengv::essentials_t E_mats = opengv::relative_pose::sevenpt(
              adapter);
          E_gv = E_mats[0];
          break;
        }
        case EIGHTPT: {
          E_gv = opengv::relative_pose::eightpt(adapter);
          break;
        }
      }

      // TODO: converting to cv::Mat renders using opengv pointless anwyay...
      cv::Mat E;
      cv::eigen2cv(E_gv, E);

      std::cout << "E: " << E_gv << std::endl;

      cv::Mat R, T;
      cv::recoverPose(E, matchMat, queryMat, R, T, lcd_params_.focal_length_,
          lcd_params_.principle_point_);
      result.relative_pose_2d_ = mat2pose(R, T);
    }

    return true;
  }

  return false;
}

// Copied almost line for line from DLoopDetector
// bool LoopClosureDetector::geometricVerification_DI(const FrameId& query_id,
//     const FrameId& match_id, const LoopResult& result) {
//   const DBoW2::FeatureVector& query_feat_vec = db_frames_[query_id].feat_vec_;
//   const DBoW2::FeatureVector& match_feat_vec = db_frames_[match_id].feat_vec_;
//
//   // Get closest descriptor for each word in common
//   vector<unsignd int> i_query, i_match;
//
//   DBoW2::FeatureVector::const_iterator query_it, match_it;
//   const DBoW2::FeatureVector::const_iterator query_end = query_feat_vec.end();
//   const DBoW2::FeatureVector::const_iterator match_end = match_feat_vec.end();
//
//   query_it = query_feat_vec.begin();
//   match_it = match_feat_vec.begin();
//
//   while (match_it != match_end && query_it != query_end) {
//     if (match_it->first == query_it->first) {
//       vector<unsigned int> i_query_now, i_match_now;
//
//       getMatches_neighratio(
//         db_frames_[match_id].descriptors_vec_, match_it->second,
//         db_frames_[query_id].descriptors_vec_, query_it->second,
//         i_match_now, i_query_now);
//
//       i_query.insert(i_query.end(), i_query_now.begin(), i_query_now.end());
//       i_match.insert(i_match.end(), i_match_now.begin(), i_match_now.end());
//
//       ++query_it;
//       ++match_it;
//     }
//
//     else if (match_it->first < query_it->first) {
//       match_it = match_feat_vec.lower_bound(query_it->first);
//     }
//
//     else {
//       query_it = query_feat_vec.lower_bound(match_it->first);
//     }
//   }
//
//   // Calculate essential matrix between 2D correspondences
//   if (i_match.size() >= lcd_params_.min_Fpoints_) {
//     vector<cv::Point2f> query_points, match_points;
//
//     // Add correspondences to vectors
//     vector<unsigned int>::const_iterator q_it, m_it;
//     q_it = i_query.begin();
//     m_it = i_match.begin();
//
//     for (; m_it != i_match.end(); ++m_it, ++q_it) {
//       const cv::KeyPoint& query_k = db_frames_[query_id].keypoints_[*q_it];
//       const cv::KeyPoint& match_k = db_frames_[match_id].keypoints_[*m_it];
//
//       query_points.push_back(query_k.pt);
//       match_points.push_back(match_k.pt);
//     }
//
//     cv::Mat queryMat(query_points.size(), 2, CV_32F, &query_points[0]);
//     cv::Mat matchMat(match_points.size(), 2, CV_32F, &match_points[0]);
//
//     cv::Mat E = cv::findEssentialMat(matchMat, queryMat,
//         lcd_params_.focal_length_, lcd_params_.principle_point_, cv::RANSAC,
//         lcd_params_.ransac_probability_);
//
//     if (!E.empty()) {
//       // 2D pose recovery:
//       cv::Mat R, T;
//       cv::recoverPose(E, matchMat, queryMat, R, T, lcd_params_.focal_length_,
//           lcd_params_.principle_point_);
//
//       result.relative_pose_2d_ = mat2pose(R, T);
//
//       return true;
//     }
//   }
//
//   return false;
// }

// bool LoopClosureDetector::geometricVerification_Exhaustive(
//     const FrameId& query_id, const FrameId& match_id,
//     const LoopResult& result) {
//
// }

gtsam::Pose3 LoopClosureDetector::compute3DPose_gv(const FrameId& query_id,
    const FrameId& match_id) {
  // Find correspondences between frames
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match);

  // TODO: is there any way to reserve the correct space for these?
  Points3d f_ref;
  Points3d f_cur;

  // Fill point clouds with matched 3D keypoints
  for (size_t i=0; i<i_match.size(); i++) {
    f_cur.push_back(db_frames_[query_id].keypoints_3d_.at(i_query[i]));
    f_ref.push_back(db_frames_[match_id].keypoints_3d_.at(i_match[i]));
  }

  Adapter3D adapter(f_ref, f_cur);
  opengv::transformation_t transformation;

  // Compute transform based on selected method
  switch (lcd_params_.pose_recovery_option_) {
    case Pose3DRecoveryOption::RANSAC_3PT: {
      std::shared_ptr<SacProblem3D> ptcloudproblem_ptr(
          new SacProblem3D(adapter, lcd_params_.ransac_randomize_3d_));
      opengv::sac::Ransac<SacProblem3D> ransac;
      ransac.sac_model_ = ptcloudproblem_ptr;
      ransac.threshold_ = lcd_params_.ransac_threshold_3d_;
      ransac.max_iterations_ = lcd_params_.max_ransac_iterations_3d_;
      ransac.probability_ = lcd_params_.ransac_probability_3d_;

      if (!ransac.computeModel()) {
        LOG(ERROR) << "LoopClosureDetector Failure: RANSAC could not solve.";
        return gtsam::Pose3();
      }

      // TODO: check for >50% inliers on all methods
      // std::cout << "ransac inliers: " << ransac.inliers_.size() << std::endl;
      // TODO: check quality of result like in Tracker::geometricOutlierRejectionStereo

      transformation = ransac.model_coefficients_;
      break;
    }

    // TODO: add checks on the emptiness of the transformation to all three
    case Pose3DRecoveryOption::ARUN_3PT: {
      transformation = opengv::point_cloud::threept_arun(adapter);
      break;
    }

    case Pose3DRecoveryOption::NONLINEAR_3PT: {
      transformation = opengv::point_cloud::optimize_nonlinear(adapter);
      break;
    }
    default: {
      LOG(ERROR) << "LoopClosureDetector: Something went wrong.";
      break;
    }
  }

  return UtilsOpenCV::Gvtrans2pose(transformation);
}

gtsam::Pose3 LoopClosureDetector::compute3DPoseGiven2D(const FrameId& query_id,
    const FrameId& match_id, gtsam::Pose3& pose_2d) {
  // Both of these are in the 'ref' frame, which is the match frame
  gtsam::Rot3 R = pose_2d.rotation();
  gtsam::Point3 unit_t = pose_2d.translation();

  // Find correspondences between frames
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match);

  // TODO: is there anyway to reserve the correct space for these?
  Points3d f_ref;
  Points3d f_cur;

  // Fill point clouds with matched 3D keypoints
  for (size_t i=0; i<i_match.size(); i++) {
    f_cur.push_back(db_frames_[query_id].keypoints_3d_.at(i_query[i]));
    f_ref.push_back(db_frames_[match_id].keypoints_3d_.at(i_match[i]));
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

} // namespace VIO
