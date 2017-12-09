/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Frame.h
 * @brief  Class describing a single image
 * @author Luca Carlone
 */

#ifndef Frame_H_
#define Frame_H_

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <vector>

// Including opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CameraParams.h"
#include "UtilsOpenCV.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace VIO {

////////////////////////////////////////////////////////////////////////////
// Class for storing/processing a single image
class Frame {

public:
  // constructors
  Frame(const FrameId id, const int64_t timestamp, const std::string img_name, const CameraParams& cam_param,
      const bool equalizeImage = false):
    id_(id), timestamp_(timestamp), img_(UtilsOpenCV::ReadAndConvertToGrayScale(img_name,equalizeImage)),
        cam_param_(cam_param), isKeyframe_(false) {}

  // copy constructor
  Frame(const Frame& f) :
    id_(f.id_), timestamp_(f.timestamp_), img_(f.img_),
        cam_param_(f.cam_param_), isKeyframe_(f.isKeyframe_),
        keypoints_(f.keypoints_), scores_(f.scores_),
        landmarks_(f.landmarks_),landmarksAge_(f.landmarksAge_),versors_(f.versors_)
  {}

  // @ TODO: add constructor which takes images as input (relevant for real tests)
  const FrameId id_;
  const Timestamp timestamp_;

  // These are nonconst since they will be changed during rectification
  CameraParams cam_param_;

  // image cv::Mat
  const cv::Mat img_;

  // results of image processing
  bool isKeyframe_ = false;

  // These containers must have same size.
  KeypointsCV keypoints_;
  std::vector<double> scores_; // quality of extracted keypoints
  LandmarkIds landmarks_;
  std::vector<int> landmarksAge_; // how many consecutive *keyframes* saw the keypoint
  BearingVectors versors_; // in the ref frame of the UNRECTIFIED left frame
  cv::Mat descriptors_; // not currently used

public:
  /* +++++++++++++++++++++++++++++++ NONCONST FUNCTIONS ++++++++++++++++++++++++++++++++++++ */
  // ExtractCorners using goodFeaturesToTrack
  void extractCorners(const double qualityLevel = 0.01, const double minDistance = 10,
      const int blockSize = 3, const bool useHarrisDetector = false, const double k = 0.04, const int maxCorners = 100)
  {
    keypoints_ = UtilsOpenCV::ExtractCorners(img_, qualityLevel, minDistance, blockSize, k, maxCorners, useHarrisDetector);
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
    // NOT TESTED:
  void setLandmarksToMinus1(const LandmarkIds lmkIds){ // TODO: this has quadratic complexity
    for(LandmarkId lmkId : lmkIds ){ // for each landmark we want to discard
      bool found = false;
      for(size_t ind = 0; ind < landmarks_.size(); ind++){ // we look for it among landmarks_
        if(landmarks_.at(ind) == lmkId){ // if found, we set it to -1
          landmarks_.at(ind) = -1;
          found = true; break;
        }
      }
      if(!found) throw std::runtime_error("setLandmarksToMinus1: lmk not found");
    }
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // NOT TESTED: get surf descriptors
  //  void extractDescriptors()
  //  {
  //    int descriptor_radius = 5;
  //
  //    // Convert points to keypoints
  //    std::vector<cv::KeyPoint> keypoints;
  //    keypoints.reserve(keypoints_.size());
  //    for (int i = 0; i < keypoints_.size(); i++) {
  //      keypoints.push_back(cv::KeyPoint(keypoints_[i], 15));
  //    }
  //
  //    cv::SurfDescriptorExtractor extractor;
  //    extractor.compute(img_, keypoints, descriptors_);
  //  }
  /* ---------------------------- CONST FUNCTIONS ------------------------------------------- */
  // NOT TESTED: undistort and return
  //  cv::Mat undistortImage() const {
  //    cv::Mat undistortedImage, undistortedCameraMatrix,undist_map_x,undist_map_y;
  //    cv::Size imageSize;
  //    cv::initUndistortRectifyMap(cam_param_.camera_matrix_, cam_param_.distortion_coeff_, cv::Mat(),
  //        undistortedCameraMatrix, imageSize, CV_16SC2, undist_map_x, undist_map_y);
  //
  //    cv::remap(img_, undistortedImage, undist_map_x, undist_map_y, cv::INTER_LINEAR);
  //    return undistortedImage;
  //  }
  /* --------------------------------------------------------------------------------------- */
  size_t getNrValidKeypoints() const
  {
    size_t count = 0;
    for(size_t i=0; i < landmarks_.size(); i++){
      if(landmarks_.at(i)!=-1)// it is valid
        count += 1;
    }
    return count;
  }
  /* --------------------------------------------------------------------------------------- */
  KeypointsCV getValidKeypoints() const
  {
    KeypointsCV validKeypoints;
    for(size_t i=0; i < landmarks_.size(); i++){
      if(landmarks_.at(i) != -1)// it is valid
        validKeypoints.push_back(keypoints_[i]);
    }
    return validKeypoints;
  }
  /* --------------------------------------------------------------------------------------- */
  LandmarkId findLmkIdFromPixel(KeypointCV px) const
  {
    for(size_t i=0; i < keypoints_.size(); i++){
      if(keypoints_.at(i).x == px.x && keypoints_.at(i).y == px.y)// it's matching the query point
        return landmarks_[i];
    }
    for(size_t i=0; i < keypoints_.size(); i++)
      std::cout << "px: " << px << " kpi: " << keypoints_.at(i) << std::endl;
    throw std::runtime_error("findLmkIdFromPixel: px not found");
  }
  /* --------------------------------------------------------------------------------------- */
  void print() const
  {
    std::cout << "Frame id: " << id_ <<  " at timestamp: " << timestamp_ << std::endl;
    std::cout << "isKeyframe_: " << isKeyframe_ << std::endl;
    std::cout << "nr keypoints_: " << keypoints_.size() << std::endl;
    std::cout << "nr valid keypoints_: " << getNrValidKeypoints() << std::endl;
    std::cout << "nr landmarks_: " << landmarks_.size() << std::endl;
    std::cout << "nr versors_: " << versors_.size() << std::endl;
    std::cout << "size descriptors_: " << descriptors_.size() << std::endl;
    cam_param_.print();
  }
  /* --------------------------------------------------------------------------------------- */
  static Vector3 CalibratePixel(const KeypointCV& cv_px, const CameraParams cam_param)
  {
    // calibrate pixel
    cv::Mat_<KeypointCV> uncalibrated_px(1,1); // matrix of px with a single entry, i.e., a single pixel
    uncalibrated_px(0) = cv_px;
    cv::Mat calibrated_px;

    cv::undistortPoints(uncalibrated_px, calibrated_px, cam_param.camera_matrix_, cam_param.distortion_coeff_);

    // transform to unit vector
    Vector3 versor = Vector3(calibrated_px.at<float>(0,0), calibrated_px.at<float>(0,1), 1.0);

    // sanity check, try to distort point using gtsam and make sure you get original pixel
    //gtsam::Point2 uncalibrated_px_gtsam = cam_param.calibration_.uncalibrate(gtsam::Point2(versor(0),versor(1)));
    //gtsam::Point2 uncalibrated_px_opencv = gtsam::Point2(uncalibrated_px.at<float>(0,0),uncalibrated_px.at<float>(0,1));
    //gtsam::Point2 px_mismatch  = uncalibrated_px_opencv - uncalibrated_px_gtsam;
    //
    //if(px_mismatch.vector().norm() > 1){
    //  std::cout << "uncalibrated_px: \n" << uncalibrated_px << std::endl;
    //  std::cout << "uncalibrated_px_gtsam: \n" << uncalibrated_px_gtsam << std::endl;
    //  std::cout << "px_mismatch: \n" << px_mismatch << std::endl;
    //  throw std::runtime_error("CalibratePixel: possible calibration mismatch");
    //}
    return versor/versor.norm(); // return unit norm vector
  }
};

} // namespace VIO
#endif /* Frame_H_ */
