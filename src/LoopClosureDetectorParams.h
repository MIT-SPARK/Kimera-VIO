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


#ifndef LoopClosureDetectorParams_H_
#define LoopClosureDetectorParams_H_

#include <string>

#include <glog/logging.h>

#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>

namespace VIO {

class LoopClosureDetectorParams {
public:
  // TODO: vocabulary path cannot be hardcoded
  LoopClosureDetectorParams(
      std::string vocabulary_path="/home/marcus/code/VIO/vocabulary/ORBvoc.txt",
      double match_threshold=0.5,
      int nfeatures=500,
      float scaleFactor=1.2f,
      int nlevels=8,
      int edgeThreshold=31,
      int firstLevel=0,
      int WTA_K=2,
      int scoreType=cv::ORB::HARRIS_SCORE,
      int patchSize=31,
      int fastThreshold=20)
      : vocabulary_path_(vocabulary_path),
        match_threshold_(match_threshold),
        nfeatures_(nfeatures),
        scaleFactor_(scaleFactor),
        nlevels_(nlevels),
        edgeThreshold_(edgeThreshold),
        firstLevel_(firstLevel),
        WTA_K_(WTA_K),
        scoreType_(scoreType),
        patchSize_(patchSize),
        fastThreshold_(fastThreshold_) {
    CHECK(match_threshold_ > 0);
    CHECK(nfeatures_ >= 100);
  }

public:
  // Loop detection and vocabulary params
  std::string vocabulary_path_;
  double match_threshold_;

  // ORB feature detector params
  int nfeatures_;
  float scaleFactor_;
  int nlevels_;
  int edgeThreshold_;
  int firstLevel_;
  int WTA_K_;
  int scoreType_;
  int patchSize_;
  int fastThreshold_;

  virtual ~LoopClosureDetectorParams() = default;

  virtual bool parseYAML(const std::string& filepath) {
    // make sure that each YAML file has %YAML:1.0 as first line
    cv::FileStorage fs;
    openFile(filepath, &fs);
    bool result = parseYAMLLCDParams(fs);
    closeFile(&fs);
    return result;
  }

  virtual void print() const { printLCDParams(); }

protected:
  void openFile(const std::string& filepath, cv::FileStorage* fs) const {
    CHECK_NOTNULL(fs);
    fs->open(filepath, cv::FileStorage::READ);
    if (!fs->isOpened()) {
      std::cout << "Cannot open file in parseYAML: " << filepath << std::endl;
      throw std::runtime_error(
        "parseYAML (LCD): cannot open file (remember first line: %YAML:1.0)");
    }
  }

  void closeFile(cv::FileStorage* fs) {
    CHECK_NOTNULL(fs);
    fs->release();
  }

  bool parseYAMLLCDParams(const cv::FileStorage& fs) {
    cv::FileNode file_handle;

    file_handle = fs["vocabulary_path"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> vocabulary_path_;

    file_handle = fs["match_threshold"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> match_threshold_;

    file_handle = fs["nfeatures"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> nfeatures_;

    file_handle = fs["scaleFactor"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> scaleFactor_;

    file_handle = fs["nlevels"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> nlevels_;

    file_handle = fs["edgeThreshold"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> edgeThreshold_;

    file_handle = fs["firstLevel"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> firstLevel_;

    file_handle = fs["WTA_K"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> WTA_K_;

    int scoreTypeId;
    file_handle = fs["scoreType"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> scoreTypeId;
    switch (scoreTypeId) {
      case 0:
        scoreType_ = cv::ORB::HARRIS_SCORE;
        break;
      // TODO: add the rest of the options here
      default:
        throw std::runtime_error("LCDparams parseYAML: wrong scoreTypeId");
        break;
    }

    file_handle = fs["patchSize"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> patchSize_;

    file_handle = fs["fastThreshold"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> fastThreshold_;

    return true;
  }

  void printLCDParams() const {
    LOG(INFO) << "$$$$$$$$$$$$$$$$$$$$$ LCD PARAMETERS $$$$$$$$$$$$$$$$$$$$$\n"
              << "vocabulary_path_: " << vocabulary_path_ << '\n'
              << "match_threshold_: " << match_threshold_ << '\n'
              << "nfeatures_: " << nfeatures_ << '\n'
              << "scaleFactor_: " << scaleFactor_ << '\n'
              << "nlevels_: " << nlevels_ << '\n'
              << "edgeThreshold_: " << edgeThreshold_ << '\n'
              << "firstLevel_: " << firstLevel_ << '\n'
              << "WTA_K_: " << WTA_K_ << '\n'
              << "scoreType_: " << scoreType_ << '\n'
              << "patchSize_: " << patchSize_ << '\n'
              << "fastThreshold_: " << fastThreshold_;
  }
}; // class LoopClosureDetectorParams
} // namespace VIO

#endif
