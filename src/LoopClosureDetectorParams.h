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
#include <DLoopDetector/DLoopDetector.h>

namespace VIO {

class LoopClosureDetectorParams {
public:
  // TODO: vocabulary path cannot be hardcoded
  LoopClosureDetectorParams(
      std::string vocabulary_path="/home/marcus/code/VIO/vocabulary/ORBvoc.txt",
      int image_height=480,
      int image_width=752,
      bool use_nss=true,
      float alpha=0.1,
      int min_temporal_matches=4,
      DLoopDetector::GeometricalCheck geom_check=DLoopDetector::GEOM_DI,
      int di_levels=0,
      int dist_local=20,
      int max_db_results=50,
      float min_nss_factor=0.005,
      int min_matches_per_group=1,
      int max_intragroup_gap=3,
      int max_distance_between_groups=3,
      int max_distance_between_queries=2,
      int min_Fpoints=12,
      int max_ransac_iterations=500,
      double ransac_probability=0.99,
      double max_reprojection_error=2.0,
      double max_neighbor_ratio=0.6,
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
        image_height_(image_height),
        image_width_(image_width),
        use_nss_(use_nss),
        alpha_(alpha),
        min_temporal_matches_(min_temporal_matches),
        geom_check_(geom_check),
        di_levels_(di_levels),
        dist_local_(dist_local),
        max_db_results_(max_db_results),
        min_nss_factor_(min_nss_factor),
        min_matches_per_group_(min_matches_per_group),
        max_intragroup_gap_(max_intragroup_gap),
        max_distance_between_groups_(max_distance_between_groups),
        max_distance_between_queries_(max_distance_between_queries),
        min_Fpoints_(min_Fpoints),
        max_ransac_iterations_(max_ransac_iterations),
        ransac_probability_(ransac_probability),
        max_reprojection_error_(max_reprojection_error),
        max_neighbor_ratio_(max_neighbor_ratio),
        nfeatures_(nfeatures),
        scaleFactor_(scaleFactor),
        nlevels_(nlevels),
        edgeThreshold_(edgeThreshold),
        firstLevel_(firstLevel),
        WTA_K_(WTA_K),
        scoreType_(scoreType),
        patchSize_(patchSize),
        fastThreshold_(fastThreshold) {
    // Trivial sanity checks:
    CHECK(image_width_ > 0);
    CHECK(image_height_ > 0);
    CHECK(alpha_ > 0);
    CHECK(nfeatures_ >= 100);
  }

public:
  //////////////////// Loop detection and vocabulary params ////////////////////
  std::string vocabulary_path_;
  int image_height_;
  int image_width_;

  bool use_nss_; // Use normalized similarity score?
  float alpha_; // Alpha threshold for matches
  int min_temporal_matches_; // Min consistent matches to pass temporal check
  DLoopDetector::GeometricalCheck geom_check_; // Geometrical check
  int di_levels_; // If using DI for geometrical checking, DI levels

  // These are less deciding parameters of the system:
  int dist_local_; // Distance between entries to be consider a match
  int max_db_results_; // Max number of results from db queries to consider
  float min_nss_factor_; // Min raw score between entries to consider a match
  int min_matches_per_group_; // Min number of close matches in a group
  int max_intragroup_gap_; // Max separation btwn matches of the same group
  int max_distance_between_groups_; // Max separation between groups
  int max_distance_between_queries_; // Max separation between two queries

  // These are for the RANSAC to compute the F:
  int min_Fpoints_; // Min number of inliers when computing a fundamental matrix
  int max_ransac_iterations_; // Max number of iterations of RANSAC
  double ransac_probability_; // Success probability of RANSAC
  double max_reprojection_error_; // Max reprojection error of fundamental mats

  // This is to compute correspondences:
  double max_neighbor_ratio_;
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////// ORB feature detector params ////////////////////////
  int nfeatures_;
  float scaleFactor_;
  int nlevels_;
  int edgeThreshold_;
  int firstLevel_;
  int WTA_K_;
  int scoreType_;
  int patchSize_;
  int fastThreshold_;
  //////////////////////////////////////////////////////////////////////////////

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

  // TODO: add all the other params to this and print
  bool parseYAMLLCDParams(const cv::FileStorage& fs) {
    cv::FileNode file_handle;

    file_handle = fs["vocabulary_path"];
    CHECK(file_handle.type() != cv::FileNode:: NONE);
    file_handle >> vocabulary_path_;

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

#endif /* LoopClosureDetectorParams_H_ */
