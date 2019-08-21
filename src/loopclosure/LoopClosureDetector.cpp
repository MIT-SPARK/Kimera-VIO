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

#include <string>
#include <algorithm>

#include "loopclosure/LoopClosureDetector.h"

#include <RobustPGO/RobustSolver.h>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>

#include "utils/Timer.h"
#include "utils/Statistics.h"
#include "UtilsOpenCV.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

// TODO(marcus): check with crowd whether this relative default is okay
DEFINE_string(vocabulary_path, "../vocabulary/ORBvoc.yml",
              "Path to BoW vocabulary file for LoopClosureDetector module.");

// TODO(marcus): add statistics reporting for verbosity_>=3
/** Verbosity settings: (cumulative with every increase in level)
      0: Runtime errors and warnings, spin start and frequency are reported.
      1: Loop closure detections are reported as warnings.
      2: Loop closure failures are reported as errors.
      3: Statistics are reported at relevant steps.
**/
static const int VERBOSITY = 0;

using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
using SacProblemMono =
    opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using AdapterStereo = opengv::point_cloud::PointCloudAdapter;
using SacProblemStereo =
    opengv::sac_problems::point_cloud::PointCloudSacProblem;


namespace VIO {

LoopClosureDetector::LoopClosureDetector(
    const LoopClosureDetectorParams& lcd_params,
    const bool log_output)
    : lcd_params_(lcd_params),
      log_output_(log_output),
      set_intrinsics_(false),
      orb_feature_detector_(),
      orb_feature_matcher_(),
      db_BoW_(nullptr),
      db_frames_(),
      latest_bowvec_(),
      latest_matched_island_(),
      latest_query_id_(FrameId(0)),
      temporal_entries_(0),
      B_Pose_camLrect_(),
      pgo_(nullptr),
      W_Pose_Bkf_estimates_(),
      shared_noise_model_(gtsam::noiseModel::Isotropic::Variance(6, 0.1)) {
  // Initialize the ORB feature detector object:
  orb_feature_detector_ = cv::ORB::create(lcd_params_.nfeatures_,
      lcd_params_.scale_factor_, lcd_params_.nlevels_,
      lcd_params_.edge_threshold_, lcd_params_.first_level_, lcd_params_.WTA_K_,
      lcd_params_.score_type_, lcd_params_.patch_sze_,
      lcd_params_.fast_threshold_);

  // Initialize our feature matching object:
  orb_feature_matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

  // Load ORB vocabulary:
  OrbVocabulary vocab;
  LOG(INFO) << "Loading vocabulary from " << FLAGS_vocabulary_path;
  vocab.load(FLAGS_vocabulary_path);
  LOG(INFO) << "Loaded vocabulary with " << vocab.size() << " visual words.";

  // Initialize db_BoW_:
  // TODO(marcus): can pass direct indexing bool and levels here.
  db_BoW_ = VIO::make_unique<OrbDatabase>(vocab);

  // Initialize pgo_:
  // TODO(marcus): parametrize the verbosity of PGO params
  // shared_noise_model_ = gtsam::noiseModel::Isotropic::Variance(6, 0.1);
  RobustPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(lcd_params_.pgo_trans_threshold_,
      lcd_params_.pgo_rot_threshold_, RobustPGO::Verbosity::VERBOSE);

  pgo_ = VIO::make_unique<RobustPGO::RobustSolver>(pgo_params);
  // TODO(marcus): decide on initialization.
  // initializePGO();
}

LoopClosureDetector::~LoopClosureDetector() {
  LOG(INFO) << "LoopClosureDetector desctuctor called.";
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
      if (output_payload.is_loop_closure_) {
        output_queue.push(output_payload);
      }

      auto spin_duration = utils::Timer::toc(tic).count();
      stat_lcd_timing.AddSample(spin_duration);

      LOG(WARNING) << "Current LoopClosureDetector frequency: "
                   << 1000.0 / spin_duration << " Hz. ("
                   << spin_duration << " ms).";

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

  // One time initialization from camera parameters.
  if (!set_intrinsics_) {
    setIntrinsics(input->stereo_frame_);
  }

  // Update the PGO with the backend VIO estimate.
  // TODO(marcus): only add factor if it's a set distance away from previous
  W_Pose_Bkf_estimates_.push_back(input->W_Pose_Blkf_);
  VioFactor vio_factor(input->cur_kf_id_, input->W_Pose_Blkf_,
      shared_noise_model_);

  // Initialize PGO with first frame if needed.
  if (vio_factor.cur_key_ == 1) {
    initializePGO(vio_factor);
  }
  addVioFactorAndOptimize(vio_factor);

  // Process the StereoFrame and check for a loop closure with previous ones.
  StereoFrame working_frame = input->stereo_frame_;
  LoopResult loop_result;
  checkLoopClosure(working_frame, &loop_result);

  // Update the PGO with the loop closure result if available.
  if (loop_result.isLoop()) {
    LoopClosureFactor lc_factor(loop_result.match_id_, loop_result.query_id_,
        loop_result.relative_pose_, shared_noise_model_);
    addLoopClosureFactorAndOptimize(lc_factor);
  }

  // Construct output payload, fill only if loop closure.
  if (loop_result.isLoop()) {
    gtsam::Pose3 w_Pose_map = getWPoseMap();
    LoopClosureDetectorOutputPayload output_payload(loop_result.isLoop(),
        input->timestamp_kf_, loop_result.match_id_, loop_result.query_id_,
        loop_result.relative_pose_, w_Pose_map, pgo_->calculateEstimate());

    return output_payload;
  }

  return LoopClosureDetectorOutputPayload(false, 0, 0, 0, gtsam::Pose3(),
      gtsam::Pose3(), gtsam::Values());
}

void LoopClosureDetector::checkLoopClosure(
    const StereoFrame& stereo_frame, LoopResult* loop_result) {
  FrameId frame_id = processAndAddFrame(stereo_frame);
  detectLoop(frame_id, loop_result);

  if (loop_result->isLoop()) {
    if (VERBOSITY >= 1) {
      LOG(WARNING) << "LoopClosureDetector: LOOP CLOSURE detected from image "
                   << loop_result->match_id_ << " to image "
                   << loop_result->query_id_;
    }
  } else {
    if (VERBOSITY >= 2) {
      std::string erhdr = "LoopClosureDetector Failure Reason: ";
      switch (loop_result->status_) {
        default:
             LOG(ERROR) << erhdr + "INVALID LCDStatus.";
        case LCDStatus::LOOP_DETECTED:
             LOG(ERROR) << erhdr + "Loop detected.";
             break;
        case LCDStatus::NO_MATCHES:
             LOG(ERROR) << erhdr + "No matches against the database.";
             break;
        case LCDStatus::LOW_NSS_FACTOR:
             LOG(ERROR) << erhdr + "Current image score vs previous too low.";
             break;
        case LCDStatus::LOW_SCORE:
             LOG(ERROR) << erhdr + "Scores were below the alpha threshold.";
             break;
        case LCDStatus::NO_GROUPS:
             LOG(ERROR) << erhdr + "Not enough matches to create groups.";
             break;
        case LCDStatus::FAILED_TEMPORAL_CONSTRAINT:
             LOG(ERROR) << erhdr + "No temporal consistency between matches.";
             break;
        case LCDStatus::FAILED_GEOM_VERIFICATION:
             LOG(ERROR) << erhdr + "The geometric verification step failed.";
             break;
        case LCDStatus::FAILED_POSE_RECOVERY:
             LOG(ERROR) << erhdr + "The pose recovery step failed.";
             break;
      }
    }
  }
}

FrameId LoopClosureDetector::processAndAddFrame(
    const StereoFrame& stereo_frame) {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Mat> descriptors_vec;
  cv::Mat descriptors_mat;

  // Extract ORB features and construct descriptors_vec.
  orb_feature_detector_->detectAndCompute(stereo_frame.getLeftFrame().img_,
      cv::Mat(), keypoints, descriptors_mat);

  int L = orb_feature_detector_->descriptorSize();
  descriptors_vec.resize(descriptors_mat.size().height);

  for (unsigned int i = 0; i < descriptors_vec.size(); i++) {
    descriptors_vec[i] = cv::Mat(1, L, descriptors_mat.type());  // one row only
    descriptors_mat.row(i).copyTo(descriptors_vec[i].row(0));
  }

  // Fill StereoFrame with ORB keypoints and perform stereo matching.
  StereoFrame cp_stereo_frame = StereoFrame(stereo_frame);
  rewriteStereoFrameFeatures(keypoints, &cp_stereo_frame);

  // Build and store LCDFrame object.
  LCDFrame cur_frame =
      LCDFrame(cp_stereo_frame.getTimestamp(), db_frames_.size(),
               cp_stereo_frame.getFrameId(), keypoints,
               cp_stereo_frame.keypoints_3d_, descriptors_vec, descriptors_mat,
               cp_stereo_frame.getLeftFrame().versors_);

  db_frames_.push_back(cur_frame);

  return cur_frame.id_;
}

bool LoopClosureDetector::detectLoop(const FrameId& frame_id,
                                     LoopResult* result) {
  result->query_id_ = frame_id;

  DBoW2::BowVector bow_vec;

  // Create BOW representation of descriptors.
  // TODO(marcus): implement direct indexing, if needed
  db_BoW_->getVocabulary()->transform(db_frames_[frame_id].descriptors_vec_,
      bow_vec);

  int max_possible_match_id = frame_id - lcd_params_.dist_local_;
  if (max_possible_match_id < 0) max_possible_match_id = 0;

  // Query for BoW vector matches in database.
  DBoW2::QueryResults query_result;
  db_BoW_->query(bow_vec, query_result, lcd_params_.max_db_results_,
      max_possible_match_id);

  // Add current BoW vector to database.
  db_BoW_->add(bow_vec);

  if (query_result.empty()) {
    result->status_ = LCDStatus::NO_MATCHES;
  } else {
    double nss_factor = 1.0;
    if (lcd_params_.use_nss_) {
      nss_factor = db_BoW_->getVocabulary()->score(bow_vec, latest_bowvec_);
    }

    if (lcd_params_.use_nss_ && nss_factor < lcd_params_.min_nss_factor_) {
      result->status_ = LCDStatus::LOW_NSS_FACTOR;
    } else {
      // Remove low scores from the QueryResults based on nss.
      DBoW2::QueryResults::iterator query_it =
          lower_bound(query_result.begin(), query_result.end(),
                      DBoW2::Result(0, lcd_params_.alpha_ * nss_factor),
                      DBoW2::Result::geq);
      if (query_it != query_result.end()) {
        query_result.resize(query_it - query_result.begin());
      }

      // Begin grouping and checking matches.
      if (query_result.empty()) {
        result->status_ = LCDStatus::LOW_SCORE;
      } else {
        // Set best candidate to highest scorer.
        result->match_id_ = query_result[0].Id;

        // Compute islands in the matches.
        std::vector<MatchIsland> islands;
        computeIslands(query_result, islands);

        if (islands.empty()) {
          result->status_ = LCDStatus::NO_GROUPS;
        } else {
          // Find the best island grouping using MatchIsland sorting.
          const MatchIsland& best_island = *std::max_element(islands.begin(),
              islands.end());

          // Run temporal constraint check on this best island.
          bool passTemporalConstraint = checkTemporalConstraint(frame_id,
              best_island);

          latest_matched_island_ = best_island;
          latest_query_id_ = frame_id;

          if (!passTemporalConstraint) {
            result->status_ = LCDStatus::FAILED_TEMPORAL_CONSTRAINT;
          } else {
            // Perform geometric verification check.
            gtsam::Pose3 camCur_T_camRef_mono;
            bool passGeometricVerification = geometricVerificationCheck(
                frame_id, best_island.best_id_, &camCur_T_camRef_mono);

            if (!passGeometricVerification) {
              result->status_ = LCDStatus::FAILED_GEOM_VERIFICATION;
            } else {
              gtsam::Pose3 bodyCur_T_bodyRef_stereo;
              bool pass3DPoseCompute =
                  recoverPose(result->query_id_, result->match_id_,
                              camCur_T_camRef_mono, &bodyCur_T_bodyRef_stereo);

              if (!pass3DPoseCompute) {
                result->status_ = LCDStatus::FAILED_POSE_RECOVERY;
              } else {
                result->relative_pose_ = bodyCur_T_bodyRef_stereo;
                result->status_ = LCDStatus::LOOP_DETECTED;
              }
            }
          }
        }
      }
    }
  }

  // Update latest bowvec for normalized similarity scoring (NSS).
  if (static_cast<int>(frame_id + 1) > lcd_params_.dist_local_) {
    latest_bowvec_ = bow_vec;
  }

  return result->isLoop();
}

bool LoopClosureDetector::geometricVerificationCheck(
    const FrameId& query_id, const FrameId& match_id,
    gtsam::Pose3* camCur_T_camRef_mono) const {
  switch (lcd_params_.geom_check_) {
    default:
      LOG(ERROR) << "LoopClosureDetector: Incorrect geom_check_ option.";
      break;
    case GeomVerifOption::NISTER:
      return geometricVerificationNister(query_id, match_id,
                                          camCur_T_camRef_mono);
    case GeomVerifOption::NONE:
      return true;
  }

  return false;
}

bool LoopClosureDetector::recoverPose(
    const FrameId& query_id, const FrameId& match_id,
    const gtsam::Pose3& camCur_T_camRef_mono,
    gtsam::Pose3* bodyCur_T_bodyRef_stereo) const {
  bool compute_success = false;

  switch (lcd_params_.pose_recovery_option_) {
    default:
      LOG(ERROR) << "LoopClosureDetector: Incorrect pose recovery option.";
      break;
    case PoseRecoveryOption::RANSAC_ARUN:
      compute_success =
          recoverPoseArun(query_id, match_id, bodyCur_T_bodyRef_stereo);
      break;
    case PoseRecoveryOption::GIVEN_ROT:
      compute_success = recoverPoseGivenRot(
          query_id, match_id, camCur_T_camRef_mono, bodyCur_T_bodyRef_stereo);
      break;
  }

  if (lcd_params_.use_mono_rot_) {
    gtsam::Pose3 bodyCur_T_bodyRef_mono;
    transformCameraPose2BodyPose(camCur_T_camRef_mono, &bodyCur_T_bodyRef_mono);
    *bodyCur_T_bodyRef_stereo =
        gtsam::Pose3(bodyCur_T_bodyRef_mono.rotation(),
                     bodyCur_T_bodyRef_stereo->translation());
  }

  return compute_success;
}

// TODO(marcus): should this be parsed from the other VIO components' params?
void LoopClosureDetector::setIntrinsics(const StereoFrame& stereo_frame) {
  lcd_params_.image_width_ =
      stereo_frame.getLeftFrame().cam_param_.image_size_.width;
  lcd_params_.image_height_ =
      stereo_frame.getLeftFrame().cam_param_.image_size_.height;
  lcd_params_.focal_length_ =
      stereo_frame.getLeftFrame().cam_param_.intrinsics_[0];
  lcd_params_.principle_point_ = cv::Point2d(
      stereo_frame.getLeftFrame().cam_param_.intrinsics_[2],
      stereo_frame.getLeftFrame().cam_param_.intrinsics_[3]);

  B_Pose_camLrect_ = stereo_frame.getBPoseCamLRect();
  set_intrinsics_ = true;
}

void LoopClosureDetector::setDatabase(const OrbDatabase& db) {
  db_BoW_ = VIO::make_unique<OrbDatabase>(db);
  clear();
}

void LoopClosureDetector::allocate(size_t n) {
  // Reserve frames
  if (n > db_frames_.size()) {
    db_frames_.resize(n);
  }

  // Reserve size for keypoints and descriptors of each expected frame
  for (size_t i = 0; i < db_frames_.size(); i++) {
    db_frames_[i].keypoints_.reserve(lcd_params_.nfeatures_);
    db_frames_[i].keypoints_3d_.reserve(lcd_params_.nfeatures_);
    db_frames_[i].descriptors_vec_.reserve(lcd_params_.nfeatures_);
    db_frames_[i].versors_.reserve(lcd_params_.nfeatures_);
  }

  db_BoW_->allocate(n, lcd_params_.nfeatures_);
  W_Pose_Bkf_estimates_.reserve(n);
}

void LoopClosureDetector::clear() {
  db_BoW_->clear();
  db_frames_.clear();
  latest_bowvec_.clear();
  latest_matched_island_.clear();
  W_Pose_Bkf_estimates_.clear();
  latest_query_id_ = 0;
  temporal_entries_ = 0;
}

void LoopClosureDetector::print() const {
  // TODO(marcus): implement
}

void LoopClosureDetector::rewriteStereoFrameFeatures(
    const std::vector<cv::KeyPoint>& keypoints,
    StereoFrame* stereo_frame) const {
  // Populate frame keypoints with ORB features instead of the normal
  // VIO features that came with the StereoFrame.

  // TODO(marcus): is this thread safe? Should I copy the stereo_frame object
  // first?
  Frame* left_frame_mutable = stereo_frame->getLeftFrameMutable();
  Frame* right_frame_mutable = stereo_frame->getRightFrameMutable();

  // Clear all relevant fields.
  left_frame_mutable->keypoints_.clear();
  left_frame_mutable->versors_.clear();
  left_frame_mutable->scores_.clear();
  right_frame_mutable->keypoints_.clear();
  right_frame_mutable->versors_.clear();
  right_frame_mutable->scores_.clear();
  stereo_frame->keypoints_3d_.clear();
  stereo_frame->keypoints_depth_.clear();
  stereo_frame->left_keypoints_rectified_.clear();
  stereo_frame->right_keypoints_rectified_.clear();

  // Reserve space in all relevant fields
  left_frame_mutable->keypoints_.reserve(keypoints.size());
  left_frame_mutable->versors_.reserve(keypoints.size());
  left_frame_mutable->scores_.reserve(keypoints.size());
  right_frame_mutable->keypoints_.reserve(keypoints.size());
  right_frame_mutable->versors_.reserve(keypoints.size());
  right_frame_mutable->scores_.reserve(keypoints.size());
  stereo_frame->keypoints_3d_.reserve(keypoints.size());
  stereo_frame->keypoints_depth_.reserve(keypoints.size());
  stereo_frame->left_keypoints_rectified_.reserve(keypoints.size());
  stereo_frame->right_keypoints_rectified_.reserve(keypoints.size());

  stereo_frame->setIsRectified(false);

  // Add ORB keypoints.
  for (unsigned int i = 0; i < keypoints.size(); i++) {
    left_frame_mutable->keypoints_.push_back(keypoints[i].pt);
    left_frame_mutable->versors_.push_back(
        Frame::CalibratePixel(keypoints[i].pt, left_frame_mutable->cam_param_));
    left_frame_mutable->scores_.push_back(1.0);
    // TODO(marcus): ^ is this a max score? Is that the right thing to do?
  }

  // Automatically match keypoints in right image with those in left.
  stereo_frame->sparseStereoMatching();
}

cv::Mat LoopClosureDetector::computeAndDrawMatchesBetweenFrames(
    const cv::Mat& query_img, const cv::Mat& match_img, const FrameId& query_id,
    const FrameId& match_id, bool cut_matches) const {
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> good_matches;

  // Use the Lowe's Ratio Test only if asked.
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  // TODO(marcus): this can use computeMatchedIndices() as well...
  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
      db_frames_[match_id].descriptors_mat_, matches, 2);

  for (size_t i = 0; i < matches.size(); i++) {
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

// TODO(marcus): rewrite to be less like DLoopDetector
bool LoopClosureDetector::checkTemporalConstraint(const FrameId& id,
    const MatchIsland& island) {
  if (temporal_entries_ == 0 ||
      int(id - latest_query_id_) > lcd_params_.max_distance_between_queries_) {
    temporal_entries_ = 1;
  } else {
    // Check whether current island encloses latest island or vice versa
    bool curEnclosesOld =
        island.start_id_ <= latest_matched_island_.start_id_ &&
        latest_matched_island_.start_id_ <= island.end_id_;
    bool oldEnclosesCur =
        latest_matched_island_.start_id_ <= island.start_id_ &&
        island.start_id_ <= latest_matched_island_.end_id_;

    bool passGroupDistConstraint = curEnclosesOld || oldEnclosesCur;
    if (!passGroupDistConstraint) {
      int d1 = (int)latest_matched_island_.start_id_ - (int)island.end_id_;
      int d2 = (int)island.start_id_ - (int)latest_matched_island_.end_id_;

      int gap;
      if (d1 > d2) {
        gap = d1;
      } else {
        gap = d2;
      }

      passGroupDistConstraint = gap <= lcd_params_.max_distance_between_groups_;
    }

    if (passGroupDistConstraint) {
      temporal_entries_++;
    } else {
      temporal_entries_ = 1;
    }
  }

  return temporal_entries_ > lcd_params_.min_temporal_matches_;
}

// TODO(marcus): rewrite to be less like DLoopDetector
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
    for (unsigned int idx = 1; dit != q.end(); ++dit, ++idx) {
      if ((int)dit->Id - last_island_entry < lcd_params_.max_intragroup_gap_) {
        last_island_entry = dit->Id;
        i_last = idx;
        if (dit->Score > best_score) {
          best_score = dit->Score;
          best_entry = dit->Id;
        }
      } else {
        // end of island reached
        int length = last_island_entry - first_island_entry + 1;
        if (length >= lcd_params_.min_matches_per_group_) {
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
    if (last_island_entry - first_island_entry + 1 >=
        lcd_params_.min_matches_per_group_) {
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
  for (FrameId id = start_id; id <= end_id; id++) {
    score_sum += q[id].Score;
  }

  return score_sum;
}

// TODO(marcus): make sure cv::DescriptorMatcher doesn't store descriptors and
// match against them later
void LoopClosureDetector::computeMatchedIndices(
    const FrameId& query_id, const FrameId& match_id,
    std::vector<unsigned int>& i_query, std::vector<unsigned int>& i_match,
    bool cut_matches) const {
  // Get two best matches between frame descriptors.
  std::vector<std::vector<cv::DMatch>> matches;
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
      db_frames_[match_id].descriptors_mat_, matches, 2);

  // TODO(marcus): any way to reserve space ahead of time? even if it's
  // over-reserved Keep only the best matches using Lowe's ratio test and store
  // indicies.
  for (size_t i = 0; i < matches.size(); i++) {
    if (matches[i][0].distance < lowe_ratio * matches[i][1].distance) {
      i_query.push_back(matches[i][0].queryIdx);
      i_match.push_back(matches[i][0].trainIdx);
    }
  }
}

// TODO(marcus): both geometrticVerification and recoverPose run the matching
// alg this is wasteful. Store the matched indices as latest for use in the
// compute step
// TODO(marcus): This is the camera frame. Your transform has to happen later
bool LoopClosureDetector::geometricVerificationNister(
    const FrameId& query_id, const FrameId& match_id,
    gtsam::Pose3* camCur_T_camRef_mono) const {
  // Find correspondences between keypoints.
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match, true);

  BearingVectors query_versors, match_versors;
  std::vector<cv::Point2f> query_points, match_points;

  for (size_t i = 0; i < i_match.size(); i++) {
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
    } else {
      // TODO(marcus): return statistics on the ransac.
      // std::cout << "ransac inliers: " << ransac.inliers_.size()
      //           << "\niterations: " << ransac.iterations_
      //           << "\nsize of input: " << query_versors.size()
      //           << std::endl;

      double inliers = static_cast<double>(ransac.inliers_.size());
      double inlier_percentage = inliers / query_versors.size();

      if (inlier_percentage >= lcd_params_.ransac_inlier_threshold_mono_) {
        opengv::transformation_t transformation = ransac.model_coefficients_;
        *camCur_T_camRef_mono = UtilsOpenCV::Gvtrans2pose(transformation);

        return true;
      }
    }
  }

  return false;
}

void LoopClosureDetector::transformCameraPose2BodyPose(
    const gtsam::Pose3& camCur_T_camRef,
    gtsam::Pose3* bodyCur_T_bodyRef) const {
  // TODO(marcus): is this the right way to set an object without returning?
  // @toni
  *bodyCur_T_bodyRef = B_Pose_camLrect_ * camCur_T_camRef *
    B_Pose_camLrect_.inverse();
}

void LoopClosureDetector::transformBodyPose2CameraPose(
    const gtsam::Pose3& bodyCur_T_bodyRef,
    gtsam::Pose3* camCur_T_camRef) const {
  *camCur_T_camRef = B_Pose_camLrect_.inverse() * bodyCur_T_bodyRef *
    B_Pose_camLrect_;
}

inline const gtsam::Pose3 LoopClosureDetector::getWPoseMap() const {
  if (W_Pose_Bkf_estimates_.size() > 1) {
    gtsam::Pose3 w_Pose_Bkf_estim = W_Pose_Bkf_estimates_.back();
    gtsam::Pose3 w_Pose_Bkf_optimal =
        pgo_->calculateBestEstimate().at<gtsam::Pose3>(
            W_Pose_Bkf_estimates_.size() - 1);

    return w_Pose_Bkf_optimal.between(w_Pose_Bkf_estim);
  }

  return gtsam::Pose3();
}

bool LoopClosureDetector::recoverPoseArun(
    const FrameId& query_id, const FrameId& match_id,
    gtsam::Pose3* bodyCur_T_bodyRef) const {
  // Find correspondences between frames.
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match, false);

  // TODO(marcus): is there any way to reserve the correct space for these?
  Points3d f_ref;
  Points3d f_cur;

  // Fill point clouds with matched 3D keypoints.
  for (size_t i = 0; i < i_match.size(); i++) {
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
  } else {
    // TODO(marcus): return statistics on the ransac.
    // std::cout << "ransac inliers: " << ransac.inliers_.size()
    //           << "\niterations: " << ransac.iterations_
    //           << "\nsize of input: " << f_ref.size()
    //           << std::endl;

    double inliers = static_cast<double>(ransac.inliers_.size());
    double inlier_percentage = inliers / f_ref.size();

    if (inlier_percentage >= lcd_params_.ransac_inlier_threshold_stereo_) {
      transformation = ransac.model_coefficients_;

      // Transform pose from camera frame to body frame.
      gtsam::Pose3 camCur_T_camRef = UtilsOpenCV::Gvtrans2pose(transformation);
      transformCameraPose2BodyPose(camCur_T_camRef, bodyCur_T_bodyRef);

      return true;
    }
  }

  return false;
}

// TODO(marcus): Add median check coordiante wise instead of the other check
// TODO(marcus): rename pose_2d to pose_cam or something for cam frame
bool LoopClosureDetector::recoverPoseGivenRot(
    const FrameId& query_id, const FrameId& match_id,
    const gtsam::Pose3& camCur_T_camRef_mono,
    gtsam::Pose3* bodyCur_T_bodyRef) const {
  gtsam::Rot3 R = camCur_T_camRef_mono.rotation();

  // Find correspondences between frames.
  std::vector<unsigned int> i_query, i_match;
  computeMatchedIndices(query_id, match_id, i_query, i_match, true);

  // TODO(marcus): is there anyway to reserve the correct space for these?
  Points3d f_ref;
  Points3d f_cur;

  // Fill point clouds with matched 3D keypoints.
  for (size_t i = 0; i < i_match.size(); i++) {
    f_cur.push_back(db_frames_[query_id].keypoints_3d_.at(i_query[i]));
    f_ref.push_back(db_frames_[match_id].keypoints_3d_.at(i_match[i]));
  }

  // TODO(marcus): decide between median check and scaling factor
  std::vector<double> x_coord, y_coord, z_coord;
  for (size_t i = 0; i < f_ref.size(); i++) {
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

  // TODO(marcus): input should alwasy be with unit translation, no need to
  // check
  // gtsam::Point3 unit_t = camCur_T_camRef_mono.translation() /
  // camCur_T_camRef_mono.translation().norm();
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
  gtsam::Pose3 camCur_T_camRef_stereo(R, scaled_t);
  transformCameraPose2BodyPose(camCur_T_camRef_stereo, bodyCur_T_bodyRef);
  return true;

  // TODO(marcus): add some sort of check for returning failure

  return false;
}

void LoopClosureDetector::initializePGO() {
  gtsam::NonlinearFactorGraph init_nfg;
  gtsam::Values init_val;
  init_val.insert(gtsam::Symbol(0), gtsam::Pose3());

  pgo_->update(init_nfg, init_val);
}

void LoopClosureDetector::initializePGO(const VioFactor& factor) {
  gtsam::NonlinearFactorGraph init_nfg;
  gtsam::Values init_val;

  init_val.insert(gtsam::Symbol(0), factor.W_Pose_Blkf_);

  init_nfg.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol(0), factor.W_Pose_Blkf_, factor.noise_));

  pgo_->update(init_nfg, init_val);
}

// TODO(marcus): only add nodes if they're x dist away from previous node
void LoopClosureDetector::addVioFactorAndOptimize(const VioFactor& factor) {
  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values value;

  if (factor.cur_key_ <= W_Pose_Bkf_estimates_.size() && factor.cur_key_ > 1) {
    value.insert(gtsam::Symbol(factor.cur_key_-1), factor.W_Pose_Blkf_);

    gtsam::Pose3 B_llkf_Pose_lkf =
        W_Pose_Bkf_estimates_.at(factor.cur_key_-2).between(
            factor.W_Pose_Blkf_);

    nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol(factor.cur_key_-2), gtsam::Symbol(factor.cur_key_-1),
        B_llkf_Pose_lkf, factor.noise_));

    pgo_->update(nfg, value);
  } else {
    LOG(WARNING) << "LoopClosureDetector: Not enough factors for optimization.";
  }
}

void LoopClosureDetector::addLoopClosureFactorAndOptimize(
    const LoopClosureFactor& factor) {
  gtsam::NonlinearFactorGraph nfg;

  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol(factor.ref_key_), gtsam::Symbol(factor.cur_key_),
      factor.ref_Pose_cur_, factor.noise_));

  pgo_->update(nfg);
}

}  // namespace VIO
