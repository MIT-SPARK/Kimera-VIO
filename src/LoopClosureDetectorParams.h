/* ----------------------------------------------------------------------------
* Copyright 2017, Massachusetts Institute of Technology,
* Cambridge, MA 02139
* All Rights Reserved
* Authors: Luca Carlone, et al. (see THANKS for the full author list)
* See LICENSE for the license information
* -------------------------------------------------------------------------- */

/**
* @file   LoopClosureDetectorParams.h
* @brief  Class collecting the parameters used for loop closure detection
* @author Marcus Abate
*/

#include <string>

#ifndef LoopClosureDetectorParams_H_
#define LoopClosureDetectorParams_H_

namespace VIO {

class LoopClosureDetectorParams {
public:
  LoopClosureDetectorParams()
    : match_threshold_(0.5),
    // TODO: This can't be hardcoded
      vocabulary_path_("/home/marcus/catkin_ws/src/loop_closure_detector/param/surf64_k10L6.voc.gz")
  {}

  LoopClosureDetectorParams(const double match_threshold,
                            const std::string vocabulary_path)
    : match_threshold_(match_threshold),
      vocabulary_path_(vocabulary_path) {}

  inline double getMatchThreshold() const { return match_threshold_; }

  inline std::string getVocabPath() const { return vocabulary_path_; }

private:
  double match_threshold_;
  std::string vocabulary_path_;
};
} // namespace VIO

#endif
