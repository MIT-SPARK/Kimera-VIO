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

#include <algorithm>
#include <fstream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include <KimeraRPGO/RobustSolver.h>

#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DEFINE_string(vocabulary_path,
              "../vocabulary/ORBvoc.yml",
              "Path to BoW vocabulary file for LoopClosureDetector module.");

/** Verbosity settings: (cumulative with every increase in level)
      0: Runtime errors and warnings, spin start and frequency are reported.
      1: Loop closure detections are reported as warnings.
      2: Loop closure failures are reported as errors.
      3: Statistics are reported at relevant steps.
**/

using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
using SacProblemMono =
    opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using AdapterStereo = opengv::point_cloud::PointCloudAdapter;
using SacProblemStereo =
    opengv::sac_problems::point_cloud::PointCloudSacProblem;

namespace VIO {

/* ------------------------------------------------------------------------ */
LoopClosureDetector::LoopClosureDetector(
    const LoopClosureDetectorParams& lcd_params,
    const bool log_output)
    : lcd_pgo_output_callbacks_(),
      lcd_params_(lcd_params),
      log_output_(log_output),
      set_intrinsics_(false),
      orb_feature_detector_(),
      orb_feature_matcher_(),
      db_BoW_(nullptr),
      db_frames_(),
      timestamp_map_(),
      latest_bowvec_(),
      latest_matched_island_(),
      latest_query_id_(FrameId(0)),
      temporal_entries_(0),
      B_Pose_camLrect_(),
      pgo_(nullptr),
      W_Pose_Blkf_estimates_(),
      logger_(nullptr) {
  // TODO(marcus): This should come in with every input payload, not be
  // constant.
  gtsam::Vector6 sigmas;
  sigmas << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
  shared_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Initialize the ORB feature detector object:
  orb_feature_detector_ = cv::ORB::create(lcd_params_.nfeatures_,
                                          lcd_params_.scale_factor_,
                                          lcd_params_.nlevels_,
                                          lcd_params_.edge_threshold_,
                                          lcd_params_.first_level_,
                                          lcd_params_.WTA_K_,
                                          lcd_params_.score_type_,
                                          lcd_params_.patch_sze_,
                                          lcd_params_.fast_threshold_);

  // Initialize our feature matching object:
  orb_feature_matcher_ =
      cv::DescriptorMatcher::create(lcd_params_.matcher_type_);

  // Load ORB vocabulary:
  std::ifstream f_vocab(FLAGS_vocabulary_path.c_str());
  CHECK(f_vocab.good()) << "LoopClosureDetector: Incorrect vocabulary path: "
                        << FLAGS_vocabulary_path;
  f_vocab.close();

  OrbVocabulary vocab;
  LOG(INFO) << "LoopClosureDetector:: Loading vocabulary from "
            << FLAGS_vocabulary_path;
  vocab.load(FLAGS_vocabulary_path);
  LOG(INFO) << "Loaded vocabulary with " << vocab.size() << " visual words.";

  // Initialize db_BoW_:
  db_BoW_ = VIO::make_unique<OrbDatabase>(vocab);

  // Initialize pgo_:
  // TODO(marcus): parametrize the verbosity of PGO params
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(lcd_params_.pgo_trans_threshold_,
                                  lcd_params_.pgo_rot_threshold_,
                                  KimeraRPGO::Verbosity::QUIET);
  pgo_ = VIO::make_unique<KimeraRPGO::RobustSolver>(pgo_params);

  if (log_output) logger_ = VIO::make_unique<LoopClosureDetectorLogger>();
}

LoopClosureDetector::~LoopClosureDetector() {
  LOG(INFO) << "LoopClosureDetector desctuctor called.";
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::spin(
    ThreadsafeQueue<LoopClosureDetectorInputPayload>& input_queue,
    bool parallel_run) {
  LOG(INFO) << "Spinning LoopClosureDetector.";
  utils::StatsCollector stat_lcd_timing("LoopClosureDetector Timing [ms]");

  while (!shutdown_) {
    // Get input data from queue. Wait for payload.
    is_thread_working_ = false;
    std::shared_ptr<LoopClosureDetectorInputPayload> input =
        input_queue.popBlocking();
    is_thread_working_ = true;

    if (input) {
      auto tic = utils::Timer::tic();
      LoopClosureDetectorOutputPayload output_payload = spinOnce(input);
      auto spin_duration = utils::Timer::toc(tic).count();
      stat_lcd_timing.AddSample(spin_duration);

      if (lcd_pgo_output_callbacks_.size() > 0) {
        for (LoopClosurePGOCallback& callback : lcd_pgo_output_callbacks_) {
          callback(output_payload);
        }

        LOG(WARNING) << "Current LoopClosureDetector frequency: "
                     << 1000.0 / spin_duration << " Hz. (" << spin_duration
                     << " ms).";
      } else {
        LOG(WARNING) << "LoopClosureDetector: No output callback registered. "
                     << "Either register a callback or disable LCD with "
                     << "flag use_lcd=false.";
      }

    } else {
      LOG(WARNING) << "No LoopClosureDetector Input Payload received.";
    }

    if (!parallel_run) return true;
  }
  LOG(INFO) << "LoopClosureDetector successfully shutdown.";
  return true;
}

/* ------------------------------------------------------------------------ */
LoopClosureDetectorOutputPayload LoopClosureDetector::spinOnce(
    const std::shared_ptr<LoopClosureDetectorInputPayload>& input) {
  CHECK(input) << "No LoopClosureDetector Input Payload received.";

  // One time initialization from camera parameters.
  if (!set_intrinsics_) {
    setIntrinsics(input->stereo_frame_);
  }
  CHECK_EQ(set_intrinsics_, true);
  CHECK_GT(input->cur_kf_id_, 0);

  // Update the PGO with the backend VIO estimate.
  // TODO(marcus): only add factor if it's a set distance away from previous
  // TODO(marcus): OdometryPose vs OdometryFactor
  timestamp_map_[input->cur_kf_id_ - 1] = input->timestamp_kf_;
  OdometryFactor odom_factor(
      input->cur_kf_id_, input->W_Pose_Blkf_, shared_noise_model_);

  // Initialize PGO with first frame if needed.
  if (odom_factor.cur_key_ == 1) {
    initializePGO(odom_factor);
  }

  // TODO(marcus): need a better check than this:
  CHECK_GT(pgo_->calculateEstimate().size(), 0);
  addOdometryFactorAndOptimize(odom_factor);

  // Process the StereoFrame and check for a loop closure with previous ones.
  StereoFrame working_frame = input->stereo_frame_;
  LoopResult loop_result;

  // Try to find a loop and update the PGO with the result if available.
  if (detectLoop(working_frame, &loop_result)) {
    LoopClosureFactor lc_factor(loop_result.match_id_,
                                loop_result.query_id_,
                                loop_result.relative_pose_,
                                shared_noise_model_);

    utils::StatsCollector stat_pgo_timing(
        "PGO Update/Optimization Timing [ms]");
    auto tic = utils::Timer::tic();

    addLoopClosureFactorAndOptimize(lc_factor);

    auto update_duration = utils::Timer::toc(tic).count();
    stat_pgo_timing.AddSample(update_duration);

    VLOG(1) << "LoopClosureDetector: LOOP CLOSURE detected from keyframe "
            << loop_result.match_id_ << " to keyframe "
            << loop_result.query_id_;
  } else {
    VLOG(2) << "LoopClosureDetector: No loop closure detected. Reason: "
            << LoopResult::asString(loop_result.status_);
  }

  // Timestamps for PGO and for LCD should match now.
  CHECK_EQ(db_frames_.back().timestamp_,
           timestamp_map_.at(db_frames_.back().id_));
  CHECK_EQ(timestamp_map_.size(), db_frames_.size());
  CHECK_EQ(timestamp_map_.size(), W_Pose_Blkf_estimates_.size());

  // Construct output payload.
  CHECK(pgo_);
  const gtsam::Pose3& w_Pose_map = getWPoseMap();
  const gtsam::Values& pgo_states = pgo_->calculateEstimate();
  const gtsam::NonlinearFactorGraph& pgo_nfg = pgo_->getFactorsUnsafe();

  LoopClosureDetectorOutputPayload output_payload;

  if (loop_result.isLoop()) {
    output_payload = LoopClosureDetectorOutputPayload(
        true,
        input->timestamp_kf_,
        timestamp_map_.at(loop_result.query_id_),
        timestamp_map_.at(loop_result.match_id_),
        loop_result.match_id_,
        loop_result.query_id_,
        loop_result.relative_pose_,
        w_Pose_map,
        pgo_states,
        pgo_nfg);
  } else {
    output_payload.W_Pose_Map_ = w_Pose_map;
    output_payload.states_ = pgo_states;
    output_payload.nfg_ = pgo_nfg;
  }

  if (logger_) {
    debug_info_.timestamp_ = output_payload.timestamp_kf_;
    debug_info_.loop_result_ = loop_result;
    debug_info_.pgo_size_ = pgo_->size();
    debug_info_.pgo_lc_count_ = pgo_->getNumLC();
    debug_info_.pgo_lc_inliers_ = pgo_->getNumLCInliers();

    logger_->logTimestampMap(timestamp_map_);
    logger_->logDebugInfo(debug_info_);
    logger_->logLCDResult(output_payload);
  }

  return output_payload;
}

/* ------------------------------------------------------------------------ */
FrameId LoopClosureDetector::processAndAddFrame(
    const StereoFrame& stereo_frame) {
  std::vector<cv::KeyPoint> keypoints;
  OrbDescriptor descriptors_mat;
  OrbDescriptorVec descriptors_vec;

  // Extract ORB features and construct descriptors_vec.
  orb_feature_detector_->detectAndCompute(
      stereo_frame.getLeftFrame().img_, cv::Mat(), keypoints, descriptors_mat);

  int L = orb_feature_detector_->descriptorSize();
  descriptors_vec.resize(descriptors_mat.size().height);

  for (size_t i = 0; i < descriptors_vec.size(); i++) {
    descriptors_vec[i] = cv::Mat(1, L, descriptors_mat.type());  // one row only
    descriptors_mat.row(i).copyTo(descriptors_vec[i].row(0));
  }

  // Fill StereoFrame with ORB keypoints and perform stereo matching.
  StereoFrame cp_stereo_frame(stereo_frame);
  rewriteStereoFrameFeatures(keypoints, &cp_stereo_frame);

  // Build and store LCDFrame object.
  db_frames_.push_back(LCDFrame(cp_stereo_frame.getTimestamp(),
                                db_frames_.size(),
                                cp_stereo_frame.getFrameId(),
                                keypoints,
                                cp_stereo_frame.keypoints_3d_,
                                descriptors_vec,
                                descriptors_mat,
                                cp_stereo_frame.getLeftFrame().versors_));

  CHECK(!db_frames_.empty());
  return db_frames_.back().id_;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::detectLoop(const StereoFrame& stereo_frame,
                                     LoopResult* result) {
  CHECK_NOTNULL(result);

  FrameId frame_id = processAndAddFrame(stereo_frame);
  result->query_id_ = frame_id;

  // Create BOW representation of descriptors.
  DBoW2::BowVector bow_vec;
  DCHECK(db_BoW_);
  db_BoW_->getVocabulary()->transform(db_frames_[frame_id].descriptors_vec_,
                                      bow_vec);

  int max_possible_match_id = frame_id - lcd_params_.dist_local_;
  if (max_possible_match_id < 0) max_possible_match_id = 0;

  // Query for BoW vector matches in database.
  DBoW2::QueryResults query_result;
  db_BoW_->query(bow_vec,
                 query_result,
                 lcd_params_.max_db_results_,
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
          lower_bound(query_result.begin(),
                      query_result.end(),
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
        computeIslands(query_result, &islands);

        if (islands.empty()) {
          result->status_ = LCDStatus::NO_GROUPS;
        } else {
          // Find the best island grouping using MatchIsland sorting.
          const MatchIsland& best_island =
              *std::max_element(islands.begin(), islands.end());

          // Run temporal constraint check on this best island.
          bool pass_temporal_constraint =
              checkTemporalConstraint(frame_id, best_island);

          latest_matched_island_ = best_island;
          latest_query_id_ = frame_id;

          if (!pass_temporal_constraint) {
            result->status_ = LCDStatus::FAILED_TEMPORAL_CONSTRAINT;
          } else {
            // Perform geometric verification check.
            gtsam::Pose3 camCur_T_camRef_mono;
            bool pass_geometric_verification = geometricVerificationCheck(
                frame_id, best_island.best_id_, &camCur_T_camRef_mono);

            if (!pass_geometric_verification) {
              result->status_ = LCDStatus::FAILED_GEOM_VERIFICATION;
            } else {
              gtsam::Pose3 bodyCur_T_bodyRef_stereo;
              bool pass_3d_pose_compute =
                  recoverPose(result->query_id_,
                              result->match_id_,
                              camCur_T_camRef_mono,
                              &bodyCur_T_bodyRef_stereo);

              if (!pass_3d_pose_compute) {
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
  } else {
    VLOG(3) << "LoopClosureDetector: Not enough frames processed.";
  }

  return result->isLoop();
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::geometricVerificationCheck(
    const FrameId& query_id,
    const FrameId& match_id,
    gtsam::Pose3* camCur_T_camRef_mono) {
  CHECK_NOTNULL(camCur_T_camRef_mono);
  switch (lcd_params_.geom_check_) {
    case GeomVerifOption::NISTER: {
      return geometricVerificationNister(
          query_id, match_id, camCur_T_camRef_mono);
    }
    case GeomVerifOption::NONE: {
      return true;
    }
    default: {
      LOG(FATAL) << "LoopClosureDetector: Incorrect geom_check_ option: "
                 << std::to_string(static_cast<int>(lcd_params_.geom_check_));
    }
  }

  return false;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::recoverPose(const FrameId& query_id,
                                      const FrameId& match_id,
                                      const gtsam::Pose3& camCur_T_camRef_mono,
                                      gtsam::Pose3* bodyCur_T_bodyRef_stereo) {
  CHECK_NOTNULL(bodyCur_T_bodyRef_stereo);

  bool passed_pose_recovery = false;

  switch (lcd_params_.pose_recovery_option_) {
    case PoseRecoveryOption::RANSAC_ARUN: {
      passed_pose_recovery =
          recoverPoseArun(query_id, match_id, bodyCur_T_bodyRef_stereo);
      break;
    }
    case PoseRecoveryOption::GIVEN_ROT: {
      passed_pose_recovery = recoverPoseGivenRot(
          query_id, match_id, camCur_T_camRef_mono, bodyCur_T_bodyRef_stereo);
      break;
    }
    default: {
      LOG(FATAL) << "LoopClosureDetector: Incorrect pose recovery option: "
                 << std::to_string(
                        static_cast<int>(lcd_params_.pose_recovery_option_));
    }
  }

  // Use the rotation obtained from 5pt method if needed.
  // TODO(marcus): check that the rotations are close to each other!
  if (lcd_params_.use_mono_rot_ &&
      lcd_params_.pose_recovery_option_ != PoseRecoveryOption::GIVEN_ROT) {
    gtsam::Pose3 bodyCur_T_bodyRef_mono;
    transformCameraPoseToBodyPose(camCur_T_camRef_mono,
                                  &bodyCur_T_bodyRef_mono);

    const gtsam::Rot3& bodyCur_R_bodyRef_stereo =
        bodyCur_T_bodyRef_mono.rotation();
    const gtsam::Point3& bodyCur_t_bodyRef_stereo =
        bodyCur_T_bodyRef_stereo->translation();

    *bodyCur_T_bodyRef_stereo =
        gtsam::Pose3(bodyCur_R_bodyRef_stereo, bodyCur_t_bodyRef_stereo);
  }

  return passed_pose_recovery;
}

/* ------------------------------------------------------------------------ */
const gtsam::Pose3 LoopClosureDetector::getWPoseMap() const {
  if (W_Pose_Blkf_estimates_.size() > 1) {
    CHECK(pgo_);
    gtsam::Pose3 w_Pose_Bkf_estim = W_Pose_Blkf_estimates_.back();
    const gtsam::Pose3& w_Pose_Bkf_optimal =
        pgo_->calculateEstimate().at<gtsam::Pose3>(
            W_Pose_Blkf_estimates_.size() - 1);

    return w_Pose_Bkf_optimal.between(w_Pose_Bkf_estim);
  }

  return gtsam::Pose3();
}

/* ------------------------------------------------------------------------ */
const gtsam::Values LoopClosureDetector::getPGOTrajectory() const {
  CHECK(pgo_);
  return pgo_->calculateEstimate();
}

/* ------------------------------------------------------------------------ */
const gtsam::NonlinearFactorGraph LoopClosureDetector::getPGOnfg() const {
  CHECK(pgo_);
  return pgo_->getFactorsUnsafe();
}

/* ------------------------------------------------------------------------ */
// TODO(marcus): this should be parsed from CameraParams directly
void LoopClosureDetector::setIntrinsics(const StereoFrame& stereo_frame) {
  const CameraParams& cam_param = stereo_frame.getLeftFrame().cam_param_;
  const std::vector<double>& intrinsics = cam_param.intrinsics_;

  lcd_params_.image_width_ = cam_param.image_size_.width;
  lcd_params_.image_height_ = cam_param.image_size_.height;
  lcd_params_.focal_length_ = intrinsics[0];
  lcd_params_.principle_point_ = cv::Point2d(intrinsics[2], intrinsics[3]);

  B_Pose_camLrect_ = stereo_frame.getBPoseCamLRect();
  set_intrinsics_ = true;
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::setDatabase(const OrbDatabase& db) {
  db_BoW_ = VIO::make_unique<OrbDatabase>(db);
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::print() const {
  // TODO(marcus): implement
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::rewriteStereoFrameFeatures(
    const std::vector<cv::KeyPoint>& keypoints,
    StereoFrame* stereo_frame) const {
  CHECK_NOTNULL(stereo_frame);

  // Populate frame keypoints with ORB features instead of the normal
  // VIO features that came with the StereoFrame.
  Frame* left_frame_mutable = stereo_frame->getLeftFrameMutable();
  Frame* right_frame_mutable = stereo_frame->getRightFrameMutable();
  CHECK_NOTNULL(left_frame_mutable);
  CHECK_NOTNULL(right_frame_mutable);

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
  for (const cv::KeyPoint& keypoint : keypoints) {
    left_frame_mutable->keypoints_.push_back(keypoint.pt);
    left_frame_mutable->versors_.push_back(
        Frame::CalibratePixel(keypoint.pt, left_frame_mutable->cam_param_));
    left_frame_mutable->scores_.push_back(1.0);
  }

  // Automatically match keypoints in right image with those in left.
  stereo_frame->sparseStereoMatching();

  size_t num_kp = keypoints.size();
  CHECK_EQ(left_frame_mutable->keypoints_.size(), num_kp);
  CHECK_EQ(left_frame_mutable->versors_.size(), num_kp);
  CHECK_EQ(left_frame_mutable->scores_.size(), num_kp);
  CHECK_EQ(stereo_frame->keypoints_3d_.size(), num_kp);
  CHECK_EQ(stereo_frame->keypoints_depth_.size(), num_kp);
  CHECK_EQ(stereo_frame->left_keypoints_rectified_.size(), num_kp);
  CHECK_EQ(stereo_frame->right_keypoints_rectified_.size(), num_kp);
}

/* ------------------------------------------------------------------------ */
cv::Mat LoopClosureDetector::computeAndDrawMatchesBetweenFrames(
    const cv::Mat& query_img,
    const cv::Mat& match_img,
    const FrameId& query_id,
    const FrameId& match_id,
    bool cut_matches) const {
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> good_matches;

  // Use the Lowe's Ratio Test only if asked.
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  // TODO(marcus): this can use computeMatchedIndices() as well...
  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
                                 db_frames_[match_id].descriptors_mat_,
                                 matches,
                                 2u);

  for (const std::vector<cv::DMatch>& match : matches) {
    if (match.at(0).distance < lowe_ratio * match.at(1).distance) {
      good_matches.push_back(match[0]);
    }
  }

  // Draw matches.
  cv::Mat img_matches;
  cv::drawMatches(query_img,
                  db_frames_.at(query_id).keypoints_,
                  match_img,
                  db_frames_.at(match_id).keypoints_,
                  good_matches,
                  img_matches,
                  cv::Scalar(255, 0, 0),
                  cv::Scalar(255, 0, 0));

  return img_matches;
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::transformCameraPoseToBodyPose(
    const gtsam::Pose3& camCur_T_camRef,
    gtsam::Pose3* bodyCur_T_bodyRef) const {
  CHECK_NOTNULL(bodyCur_T_bodyRef);
  *bodyCur_T_bodyRef =
      B_Pose_camLrect_ * camCur_T_camRef * B_Pose_camLrect_.inverse();
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::transformBodyPoseToCameraPose(
    const gtsam::Pose3& bodyCur_T_bodyRef,
    gtsam::Pose3* camCur_T_camRef) const {
  CHECK_NOTNULL(camCur_T_camRef);
  *camCur_T_camRef =
      B_Pose_camLrect_.inverse() * bodyCur_T_bodyRef * B_Pose_camLrect_;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::checkTemporalConstraint(const FrameId& id,
                                                  const MatchIsland& island) {
  if (temporal_entries_ == 0 || static_cast<int>(id - latest_query_id_) >
                                    lcd_params_.max_distance_between_queries_) {
    temporal_entries_ = 1;
  } else {
    // Check whether current island encloses latest island or vice versa
    bool cur_encloses_old =
        island.start_id_ <= latest_matched_island_.start_id_ &&
        latest_matched_island_.start_id_ <= island.end_id_;
    bool old_encloses_cur =
        latest_matched_island_.start_id_ <= island.start_id_ &&
        island.start_id_ <= latest_matched_island_.end_id_;

    bool pass_group_constraint = cur_encloses_old || old_encloses_cur;
    if (!pass_group_constraint) {
      int d1 = static_cast<int>(latest_matched_island_.start_id_) -
               static_cast<int>(island.end_id_);
      int d2 = static_cast<int>(island.start_id_) -
               static_cast<int>(latest_matched_island_.end_id_);

      int gap;
      if (d1 > d2) {
        gap = d1;
      } else {
        gap = d2;
      }

      pass_group_constraint = gap <= lcd_params_.max_distance_between_groups_;
    }

    if (pass_group_constraint) {
      temporal_entries_++;
    } else {
      temporal_entries_ = 1;
    }
  }

  return temporal_entries_ > lcd_params_.min_temporal_matches_;
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::computeIslands(
    DBoW2::QueryResults& q,
    std::vector<MatchIsland>* islands) const {
  CHECK_NOTNULL(islands);
  islands->clear();

  // The case of one island is easy to compute and is done separately
  if (q.size() == 1) {
    MatchIsland island(q[0].Id, q[0].Id, q[0].Score);
    island.best_id_ = q[0].Id;
    island.best_score_ = q[0].Score;
    islands->push_back(island);
  } else if (!q.empty()) {
    // sort query results in ascending order of ids
    std::sort(q.begin(), q.end(), DBoW2::Result::ltId);

    // create long enough islands
    DBoW2::QueryResults::const_iterator dit = q.begin();
    int first_island_entry = static_cast<int>(dit->Id);
    int last_island_entry = static_cast<int>(dit->Id);

    // these are indices of q
    FrameId i_first = 0;
    FrameId i_last = 0;

    double best_score = dit->Score;
    DBoW2::EntryId best_entry = dit->Id;

    ++dit;
    for (FrameId idx = 1; dit != q.end(); ++dit, ++idx) {
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
          MatchIsland island =
              MatchIsland(first_island_entry,
                          last_island_entry,
                          computeIslandScore(q, i_first, i_last));

          islands->push_back(island);
          islands->back().best_score_ = best_score;
          islands->back().best_id_ = best_entry;
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
                                       last_island_entry,
                                       computeIslandScore(q, i_first, i_last));

      islands->push_back(island);
      islands->back().best_score_ = best_score;
      islands->back().best_id_ = static_cast<FrameId>(best_entry);
    }
  }
}

/* ------------------------------------------------------------------------ */
double LoopClosureDetector::computeIslandScore(const DBoW2::QueryResults& q,
                                               const FrameId& start_id,
                                               const FrameId& end_id) const {
  CHECK_GT(q.size(), start_id);
  CHECK_GT(q.size(), end_id);
  double score_sum = 0.0;
  for (FrameId id = start_id; id <= end_id; id++) {
    score_sum += q.at(id).Score;
  }

  return score_sum;
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::computeMatchedIndices(const FrameId& query_id,
                                                const FrameId& match_id,
                                                std::vector<FrameId>* i_query,
                                                std::vector<FrameId>* i_match,
                                                bool cut_matches) const {
  CHECK_NOTNULL(i_query);
  CHECK_NOTNULL(i_match);
  // Get two best matches between frame descriptors.
  std::vector<std::vector<cv::DMatch>> matches;
  double lowe_ratio = 1.0;
  if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

  orb_feature_matcher_->knnMatch(db_frames_[query_id].descriptors_mat_,
                                 db_frames_[match_id].descriptors_mat_,
                                 matches,
                                 2u);

  // TODO(marcus): Worth it to reserve space ahead of time? even if it's
  // over-reserved Keep only the best matches using Lowe's ratio test and store
  // indicies.
  for (const std::vector<cv::DMatch>& match : matches) {
    if (match.at(0).distance < lowe_ratio * match.at(1).distance) {
      i_query->push_back(match[0].queryIdx);
      i_match->push_back(match[0].trainIdx);
    }
  }
}

/* ------------------------------------------------------------------------ */
// TODO(marcus): both geometrticVerification and recoverPose run the matching
// alg. this is wasteful. Store the matched indices as latest for use in the
// compute step
bool LoopClosureDetector::geometricVerificationNister(
    const FrameId& query_id,
    const FrameId& match_id,
    gtsam::Pose3* camCur_T_camRef_mono) {
  CHECK_NOTNULL(camCur_T_camRef_mono);

  // Find correspondences between keypoints.
  std::vector<FrameId> i_query, i_match;
  computeMatchedIndices(query_id, match_id, &i_query, &i_match, true);

  BearingVectors query_versors, match_versors;

  CHECK_EQ(i_query.size(), i_match.size());
  query_versors.resize(i_query.size());
  match_versors.resize(i_match.size());
  for (size_t i = 0; i < i_match.size(); i++) {
    query_versors[i] = (db_frames_[query_id].versors_[i_query[i]]);
    match_versors[i] = (db_frames_[match_id].versors_[i_match[i]]);
  }

  // Recover relative pose between frames, with translation up to a scalar.
  if (static_cast<int>(match_versors.size()) >=
      lcd_params_.min_correspondences_) {
    AdapterMono adapter(match_versors, query_versors);

    // Use RANSAC to solve the central-relative-pose problem.
    opengv::sac::Ransac<SacProblemMono> ransac;
    std::shared_ptr<SacProblemMono> relposeproblem_ptr(
        new SacProblemMono(adapter,
                           SacProblemMono::Algorithm::NISTER,
                           lcd_params_.ransac_randomize_mono_));

    ransac.sac_model_ = relposeproblem_ptr;
    ransac.max_iterations_ = lcd_params_.max_ransac_iterations_mono_;
    ransac.probability_ = lcd_params_.ransac_probability_mono_;
    ransac.threshold_ = lcd_params_.ransac_threshold_mono_;

    // Compute transformation via RANSAC.
    bool ransac_success = ransac.computeModel();
    VLOG(3) << "ransac 5pt size of input: " << query_versors.size()
            << "\nransac 5pt inliers: " << ransac.inliers_.size()
            << "\nransac 5pt iterations: " << ransac.iterations_;
    debug_info_.mono_input_size_ = query_versors.size();
    debug_info_.mono_inliers_ = ransac.inliers_.size();
    debug_info_.mono_iter_ = ransac.iterations_;

    if (!ransac_success) {
      VLOG(3) << "LoopClosureDetector Failure: RANSAC 5pt could not solve.";
    } else {
      double inlier_percentage =
          static_cast<double>(ransac.inliers_.size()) / query_versors.size();

      if (inlier_percentage >= lcd_params_.ransac_inlier_threshold_mono_) {
        if (ransac.iterations_ < lcd_params_.max_ransac_iterations_mono_) {
          opengv::transformation_t transformation = ransac.model_coefficients_;
          *camCur_T_camRef_mono = UtilsOpenCV::Gvtrans2pose(transformation);

          return true;
        }
      }
    }
  }

  return false;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::recoverPoseArun(const FrameId& query_id,
                                          const FrameId& match_id,
                                          gtsam::Pose3* bodyCur_T_bodyRef) {
  CHECK_NOTNULL(bodyCur_T_bodyRef);

  // Find correspondences between frames.
  std::vector<FrameId> i_query, i_match;
  computeMatchedIndices(query_id, match_id, &i_query, &i_match, false);

  Points3d f_ref, f_cur;

  // Fill point clouds with matched 3D keypoints.
  CHECK_EQ(i_query.size(), i_match.size());
  f_ref.resize(i_match.size());
  f_cur.resize(i_query.size());
  for (size_t i = 0; i < i_match.size(); i++) {
    f_cur[i] = (db_frames_[query_id].keypoints_3d_.at(i_query[i]));
    f_ref[i] = (db_frames_[match_id].keypoints_3d_.at(i_match[i]));
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

  // Compute transformation via RANSAC.
  bool ransac_success = ransac.computeModel();
  VLOG(3) << "ransac 3pt size of input: " << f_ref.size()
          << "\nransac 3pt inliers: " << ransac.inliers_.size()
          << "\nransac 3pt iterations: " << ransac.iterations_;
  debug_info_.stereo_input_size_ = f_ref.size();
  debug_info_.stereo_inliers_ = ransac.inliers_.size();
  debug_info_.stereo_iter_ = ransac.iterations_;

  if (!ransac_success) {
    VLOG(3) << "LoopClosureDetector Failure: RANSAC 3pt could not solve.";
  } else {
    double inlier_percentage =
        static_cast<double>(ransac.inliers_.size()) / f_ref.size();

    if (inlier_percentage >= lcd_params_.ransac_inlier_threshold_stereo_) {
      if (ransac.iterations_ < lcd_params_.max_ransac_iterations_stereo_) {
        transformation = ransac.model_coefficients_;

        // Transform pose from camera frame to body frame.
        gtsam::Pose3 camCur_T_camRef =
            UtilsOpenCV::Gvtrans2pose(transformation);
        transformCameraPoseToBodyPose(camCur_T_camRef, bodyCur_T_bodyRef);

        return true;
      }
    }
  }

  return false;
}

/* ------------------------------------------------------------------------ */
bool LoopClosureDetector::recoverPoseGivenRot(
    const FrameId& query_id,
    const FrameId& match_id,
    const gtsam::Pose3& camCur_T_camRef_mono,
    gtsam::Pose3* bodyCur_T_bodyRef) {
  CHECK_NOTNULL(bodyCur_T_bodyRef);

  gtsam::Rot3 R = camCur_T_camRef_mono.rotation();

  // Find correspondences between frames.
  std::vector<FrameId> i_query, i_match;
  computeMatchedIndices(query_id, match_id, &i_query, &i_match, true);

  // Fill point clouds with matched 3D keypoints.
  size_t n_matches = i_match.size();
  CHECK_EQ(i_query.size(), n_matches);

  if (n_matches > 0) {
    std::vector<double> x_coord, y_coord, z_coord;
    x_coord.reserve(n_matches);
    y_coord.reserve(n_matches);
    z_coord.reserve(n_matches);

    for (size_t i = 0; i < n_matches; i++) {
      const gtsam::Vector3& keypoint_cur =
          db_frames_[query_id].keypoints_3d_.at(i_query[i]);
      const gtsam::Vector3& keypoint_ref =
          db_frames_[match_id].keypoints_3d_.at(i_match[i]);

      gtsam::Vector3 rotated_keypoint_diff = keypoint_ref - (R * keypoint_cur);
      x_coord.push_back(rotated_keypoint_diff[0]);
      y_coord.push_back(rotated_keypoint_diff[1]);
      z_coord.push_back(rotated_keypoint_diff[2]);
    }

    CHECK_EQ(x_coord.size(), n_matches);
    CHECK_EQ(y_coord.size(), n_matches);
    CHECK_EQ(z_coord.size(), n_matches);

    // TODO(marcus): decide between median check and scaling factor
    std::sort(x_coord.begin(), x_coord.end());
    std::sort(y_coord.begin(), y_coord.end());
    std::sort(z_coord.begin(), z_coord.end());

    gtsam::Point3 scaled_t(
        x_coord.at(std::floor(static_cast<int>(x_coord.size() / 2))),
        y_coord.at(std::floor(static_cast<int>(y_coord.size() / 2))),
        z_coord.at(std::floor(static_cast<int>(z_coord.size() / 2))));

    // Transform pose from camera frame to body frame.
    gtsam::Pose3 camCur_T_camRef_stereo(R, scaled_t);
    transformCameraPoseToBodyPose(camCur_T_camRef_stereo, bodyCur_T_bodyRef);

    return true;
  }

  return false;

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
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::initializePGO() {
  gtsam::NonlinearFactorGraph init_nfg;
  gtsam::Values init_val;
  init_val.insert(gtsam::Symbol(0), gtsam::Pose3());

  CHECK(pgo_);
  pgo_->update(init_nfg, init_val);
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::initializePGO(const OdometryFactor& factor) {
  gtsam::NonlinearFactorGraph init_nfg;
  gtsam::Values init_val;

  init_val.insert(gtsam::Symbol(0), factor.W_Pose_Blkf_);

  init_nfg.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol(0), factor.W_Pose_Blkf_, factor.noise_));

  CHECK(pgo_);
  pgo_->update(init_nfg, init_val);
}

/* ------------------------------------------------------------------------ */
// TODO(marcus): only add nodes if they're x dist away from previous node
// TODO(marcus): consider making the keys of OdometryFactor minus one each so
// that the extra check in here isn't needed...
void LoopClosureDetector::addOdometryFactorAndOptimize(
    const OdometryFactor& factor) {
  W_Pose_Blkf_estimates_.push_back(factor.W_Pose_Blkf_);
  CHECK(factor.cur_key_ <= W_Pose_Blkf_estimates_.size())
      << "LoopClosureDetector: new odometry factor has a key that is too high.";

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values value;

  if (factor.cur_key_ > 1) {
    value.insert(gtsam::Symbol(factor.cur_key_ - 1), factor.W_Pose_Blkf_);

    gtsam::Pose3 B_llkf_Pose_lkf =
        W_Pose_Blkf_estimates_.at(factor.cur_key_ - 2)
            .between(factor.W_Pose_Blkf_);

    nfg.add(
        gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(factor.cur_key_ - 2),
                                           gtsam::Symbol(factor.cur_key_ - 1),
                                           B_llkf_Pose_lkf,
                                           factor.noise_));

    CHECK(pgo_);
    pgo_->update(nfg, value);
  } else {
    LOG(WARNING) << "LoopClosureDetector: Not enough factors for optimization.";
  }
}

/* ------------------------------------------------------------------------ */
void LoopClosureDetector::addLoopClosureFactorAndOptimize(
    const LoopClosureFactor& factor) {
  gtsam::NonlinearFactorGraph nfg;

  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(factor.ref_key_),
                                             gtsam::Symbol(factor.cur_key_),
                                             factor.ref_Pose_cur_,
                                             factor.noise_));

  CHECK(pgo_);
  pgo_->update(nfg);
}

}  // namespace VIO
