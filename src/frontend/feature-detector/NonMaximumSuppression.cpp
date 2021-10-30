/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   NonMaximumSuppression.cpp
 * @brief  Base class for non maximum suppresion interface
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/frontend/feature-detector/NonMaximumSuppression.h"

#include <numeric>
#include <vector>

#include <opencv2/opencv.hpp>

#include <glog/logging.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/feature-detector/anms/anms.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

AdaptiveNonMaximumSuppression::AdaptiveNonMaximumSuppression(
    const AnmsAlgorithmType& anms_algorithm_type)
    : NonMaximumSuppression(), anms_algorithm_type_(anms_algorithm_type){};

std::vector<cv::KeyPoint> AdaptiveNonMaximumSuppression::suppressNonMax(
    const std::vector<cv::KeyPoint>& keyPoints,
    const int& numRetPoints,
    const float& tolerance,
    const int& cols,
    const int& rows) {
  auto tic = utils::Timer::tic();
  if (keyPoints.size() == 0) {
    LOG(WARNING) << "No keypoints for non-max suppression...";
    return std::vector<cv::KeyPoint>();
  }

  // Sorting keypoints by deacreasing order of strength
  VLOG(5) << "Sorting keypoints in decreasing order of strength.";
  std::vector<int> responseVector;
  for (unsigned int i = 0; i < keyPoints.size(); i++) {
    responseVector.push_back(keyPoints[i].response);
  }
  std::vector<int> Indx(responseVector.size());
  std::iota(std::begin(Indx), std::end(Indx), 0);
  cv::sortIdx(responseVector, Indx, cv::SortFlags::SORT_DESCENDING);
  std::vector<cv::KeyPoint> keyPointsSorted;
  for (unsigned int i = 0; i < keyPoints.size(); i++) {
    keyPointsSorted.push_back(keyPoints[Indx[i]]);
  }

  std::vector<cv::KeyPoint> keypoints;
  VLOG(5) << "Starting Adaptive Non-Maximum Suppression.";
  switch (anms_algorithm_type_) {
    case AnmsAlgorithmType::TopN: {
      VLOG(1) << "Running TopN: " << VIO::to_underlying(anms_algorithm_type_);
      keypoints = anms::TopN(keyPoints, numRetPoints);
      break;
    };
    case AnmsAlgorithmType::BrownANMS: {
      VLOG(1) << "Running BrownANMS: "
              << VIO::to_underlying(anms_algorithm_type_);
      keypoints = anms::BrownANMS(keyPoints, numRetPoints);
      break;
    };
    case AnmsAlgorithmType::SDC: {
      VLOG(1) << "Running SDC: " << VIO::to_underlying(anms_algorithm_type_);
      keypoints =
          anms::Sdc(keyPointsSorted, numRetPoints, tolerance, cols, rows);
      break;
    };
    case AnmsAlgorithmType::KdTree: {
      VLOG(1) << "Running KdTree: " << VIO::to_underlying(anms_algorithm_type_);
      keypoints =
          anms::KdTree(keyPointsSorted, numRetPoints, tolerance, cols, rows);
      break;
    };
    case AnmsAlgorithmType::RangeTree: {
      VLOG(1) << "Running RangeTree: "
              << VIO::to_underlying(anms_algorithm_type_);
      keypoints =
          anms::RangeTree(keyPointsSorted, numRetPoints, tolerance, cols, rows);
      break;
    };
    case AnmsAlgorithmType::Ssc: {
      VLOG(1) << "Running SSC: " << VIO::to_underlying(anms_algorithm_type_);
      keypoints =
          anms::Ssc(keyPointsSorted, numRetPoints, tolerance, cols, rows);
      break;
    };
    case AnmsAlgorithmType::Binning: {
      VLOG(1) << "Running Binning: " << VIO::to_underlying(anms_algorithm_type_);
      keypoints = binning(keyPointsSorted, numRetPoints, tolerance, cols, rows);
      break;
    };
    default: {
      VLOG(1) << "Unknown ANMS algorithm requested: "
              << VIO::to_underlying(anms_algorithm_type_);
      break;
    };
  }
  VLOG(1) << "Non Maximum Suppression Timing [ms]: "
          << utils::Timer::toc(tic).count();
  return keypoints;
}

// ---------------------------------------------------------------------------------
std::vector<cv::KeyPoint> AdaptiveNonMaximumSuppression::binning(
    const std::vector<cv::KeyPoint>& keyPoints, const int& numKptsToRetain,
    const float& tolerance, const int& imgCols, const int& imgRows){

  if (numKptsToRetain > keyPoints.size()) {
    return keyPoints;
  }

  size_t nrHorizontalBins = 5;
  size_t nrVerticalBins = 5;

  float binRowSize = float(imgRows) / float(nrVerticalBins);
  float binColSize = float(imgCols) / float(nrHorizontalBins);

  // 0. Note: features should be already sorted by score at this point from detect
  // 1. assign keypoints to bins
  // 1a: allocate bins
  std::vector<std::vector<std::vector<cv::KeyPoint> > > keypoints_at_bin_ij (
      nrVerticalBins,
      std::vector <std::vector<cv::KeyPoint> >(nrHorizontalBins, std::vector<cv::KeyPoint>(0)));

  // 1b: assign each kpt to bins
  for (int i = 0; i < keyPoints.size(); i++){
      const size_t binRowInd = static_cast<size_t>(keyPoints[i].pt.y / binRowSize);
      const size_t binColInd = static_cast<size_t>(keyPoints[i].pt.x / binColSize);
      keypoints_at_bin_ij[binRowInd][binColInd].push_back(keyPoints[i]);
  }

  // 2. compute how many features we want to retain in each bin numRetPointsPerBin
  const int numRetPointsPerBin = std::ceil( float(numKptsToRetain) / float(nrHorizontalBins*nrVerticalBins) );

  // 3. select top numRetPointsPerBin for each bin
  std::vector<cv::KeyPoint> binnedKpts; // binned keypoints we want to output
  for(int binRowInd=0; binRowInd<nrVerticalBins; binRowInd++){
      for(int binColInd=0; binColInd<nrHorizontalBins; binColInd++){
	  int nrKptsInBin = keypoints_at_bin_ij[binRowInd][binColInd].size();
	  for(int i=0; i< std::min(numRetPointsPerBin,nrKptsInBin); i++){
	      binnedKpts.push_back(keypoints_at_bin_ij[binRowInd][binColInd][i]);
	  }
      }
  }

  return binnedKpts;
}

}  // namespace VIO
