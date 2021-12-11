/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   NonMaximumSuppression.h
 * @brief  Base class for non maximum suppression interface
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/**
 * Base class for non maximum suppression algorithms
 */
class NonMaximumSuppression {
 public:
  KIMERA_POINTER_TYPEDEFS(NonMaximumSuppression);
  KIMERA_DELETE_COPY_CONSTRUCTORS(NonMaximumSuppression);

  NonMaximumSuppression() = default;
  virtual ~NonMaximumSuppression() = default;

 public:
  virtual std::vector<cv::KeyPoint> suppressNonMax(
      const std::vector<cv::KeyPoint>& keyPoints,
      const int& numRetPoints,
      const float& tolerance,
      const int& cols,
      const int& rows,
      const int& nr_horizontal_bins,
      const int& nr_vertical_bins,
      const Eigen::MatrixXd& binning_mask) = 0;
};

/**
 * Types of supported Adaptive Non Maximum Suppression algorithms
 */
enum class AnmsAlgorithmType : unsigned int {
  TopN = 0,
  BrownANMS = 1,
  SDC = 2,
  KdTree = 3,
  RangeTree = 4,
  Ssc = 5,
  Binning = 6
};

/**
 * Base class for Adaptive Non Maximum Suppression algorithms.
 * Currently a simple wrapper over ANMS algorithms as presented in:
 * Bailo, Oleksandr and Rameau, Francois and Joo, Kyungdon and Park, Jinsun and
 Bogdan, Oleksandr and Kweon, In So
 * "Efficient adaptive non-maximal suppression algorithms for homogeneous
 spatial keypoint distribution}, Pattern Recognition Letters
 with an extra option to do feature binning as an alternative nms approach.
 */
class AdaptiveNonMaximumSuppression : public NonMaximumSuppression {
 public:
  KIMERA_POINTER_TYPEDEFS(AdaptiveNonMaximumSuppression);
  KIMERA_DELETE_COPY_CONSTRUCTORS(AdaptiveNonMaximumSuppression);
  AdaptiveNonMaximumSuppression(const AnmsAlgorithmType& anms_algorithm_type);
  virtual ~AdaptiveNonMaximumSuppression() = default;

 public:
  std::vector<cv::KeyPoint> suppressNonMax(
      const std::vector<cv::KeyPoint>& keyPoints,
      const int& numRetPoints,
      const float& tolerance,
      const int& cols,
      const int& rows,
      const int& nr_horizontal_bins,
      const int& nr_vertical_bins,
      const Eigen::MatrixXd& binning_mask) override;

  std::vector<cv::KeyPoint> binning(const std::vector<cv::KeyPoint>& keyPoints,
                                    const int& numRetPoints,
                                    const int& cols,
                                    const int& rows,
                                    const int& nr_horizontal_bins,
                                    const int& nr_vertical_bins,
                                    const Eigen::MatrixXd& binning_mask);

  /**
   * @brief setAnmsAlgorithm in case the user wants to dynamically change the
   * ANMS algorithm (not sure why someone would do that, but here it is).
   * @param anms_algorithm_type
   */
  inline void setAnmsAlgorithm(const AnmsAlgorithmType& anms_algorithm_type) {
    anms_algorithm_type_ = anms_algorithm_type;
  }

 protected:
  AnmsAlgorithmType anms_algorithm_type_;
};

}  // namespace VIO
