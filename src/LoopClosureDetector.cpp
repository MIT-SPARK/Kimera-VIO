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
      log_output_(log_output) {
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

  // loop_detector_ = OrbLoopDetector(vocab_, params); // TODO: Why doesn't this work?
  loop_detector_ = OrbLoopDetector(params);
  loop_detector_.setVocabulary(vocab_);
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

  // TODO: Frame.h says img_ is public but it probably shouldn't be.
  DLoopDetector::DetectionResult loop_result = processImage(
    input->stereo_frame_.getLeftFrame().img_);

  // TODO: calculate transformation between the frames

  // TODO: add LoggerMatlab options

  return processResult(loop_result, input->stereo_frame_.getTimestamp());
}

DLoopDetector::DetectionResult LoopClosureDetector::processImage(
    const cv::Mat& img) {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Mat> descriptors;

  // clock_t begin = clock();
  extractOrb(img, keypoints, descriptors);
  // clock_t end = clock();
  // double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000.0;
  // std::cout << "extractOrb took " << elapsed_ms << " ms." << std::endl;

  DLoopDetector::DetectionResult result;
  loop_detector_.detectLoop(keypoints, descriptors, result);

  return result;
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

LoopClosureDetectorOutputPayload LoopClosureDetector::processResult(
    const DLoopDetector::DetectionResult& loop_result,
    const Timestamp& timestamp_kf) {
  if (loop_result.detection()) {
    LOG(ERROR) << "LoopClosureDetector: LOOP CLOSURE detected from image "
               << loop_result.match << " to image " << loop_result.query
               << std::endl;

    LoopClosureDetectorOutputPayload output_payload(true, timestamp_kf,
                                                    loop_result.query,
                                                    loop_result.match,
                                                    loop_result.transformation);
    return output_payload;
  }
  else {
    // Can print the status here for when loops are not detected
    return LoopClosureDetectorOutputPayload(false,-1,0,0,cv::Mat());
  }
}

} // namespace VIO
