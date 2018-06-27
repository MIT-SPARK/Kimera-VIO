/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoggerMatlab.h
 * @brief  Logging information for matlab stats and visualizations
 * @author Luca Carlone, Antoni Rosinol
 */

#ifndef LoggerMatlab_H_
#define LoggerMatlab_H_

#include <unordered_map>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <VioBackEnd.h>
#include "UtilsOpenCV.h"
#include "ETH_parser.h"
#include "StereoVisionFrontEnd.h"

namespace VIO {

////////////////////////////////////////////////////////////////////////////////
class LoggerMatlab {
public:
  LoggerMatlab();

  // Path where to store output files.
  const std::string output_path_;
  std::ofstream outputFile_;
  std::ofstream outputFile_posesVIO_;
  std::ofstream outputFile_posesVIO_csv_;
  std::ofstream outputFile_posesGT_;
  std::ofstream outputFile_landmarks_;
  std::ofstream outputFile_normals_;
  std::ofstream outputFile_smartFactors_;
  std::ofstream outputFile_timingVIO_;
  std::ofstream outputFile_timingTracker_;
  std::ofstream outputFile_statsTracker_;
  std::ofstream outputFile_statsFactors_;
  std::ofstream outputFile_mesh_;

  gtsam::Pose3 W_Pose_Bprevkf_vio_;

  double timing_loadStereoFrame_, timing_processStereoFrame_,
  timing_featureSelection_, timing_vio_, timing_loggerBackend_,
  timing_loggerFrontend_;

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void openLogFiles(int i = -1);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void closeLogFiles(int i = -1);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logFrontendResults(const ETHDatasetParser& dataset,
                          const StereoVisionFrontEnd& stereoTracker,
                          const Timestamp timestamp_lkf,
                          const Timestamp timestamp_k);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logLandmarks(const VioBackEnd::PointsWithId& lmks);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logLandmarks(const cv::Mat& lmks);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logNormals(const std::vector<cv::Point3f>& normals);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logMesh(const cv::Mat& lmks, const cv::Mat& colors, const cv::Mat& mesh);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void logBackendResults(const ETHDatasetParser& dataset,
                         const StereoVisionFrontEnd& stereoTracker,
                         const boost::shared_ptr<VioBackEnd>& vio,
                         const Timestamp timestamp_lkf,
                         const Timestamp timestamp_k,
                         const size_t k);

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void displayInitialStateVioInfo(const ETHDatasetParser& dataset,
                                  const boost::shared_ptr<VIO::VioBackEnd>& vio,
                                  gtNavState initialStateGT,
                                  const ImuAccGyr& imu_accgyr,
                                  const Timestamp timestamp_k) const;

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  void displayOverallTiming() const;
};

} // namespace VIO
#endif /* LoggerMatlab_H_ */

