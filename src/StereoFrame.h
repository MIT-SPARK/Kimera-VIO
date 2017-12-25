/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Frame.h
 * @brief  Class describing a pair of stereo images
 * @author Luca Carlone
 */

#ifndef StereoFrame_H_
#define StereoFrame_H_

//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <opencv2/video/tracking.hpp>
//#include "StereoFrame.h"
//#include <vector>
//#include <string>
//#include <algorithm>
//#include <iostream>
//#include <iterator>
//#include <stdio.h>
//#include <stdlib.h>
//#include <ctype.h>

#include "Frame.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include "UtilsGeometry.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

namespace VIO {

using SmartStereoMeasurement = std::pair<LandmarkId,gtsam::StereoPoint2>;
using SmartStereoMeasurements = std::vector<SmartStereoMeasurement>;

//////////////////////////////////////////////////////////////////////
class StereoMatchingParams{
public:
  const double toleranceTemplateMatching;
  const double nominalBaseline;
  const int templ_cols; // must be odd
  const int templ_rows; // must be odd
  const int stripe_extra_rows; // must be even
  const double minPointDist; // stereo points triangulated below this distance are discarded
  const double maxPointDist; // stereo points triangulated beyond this distance are discarded=
  const bool bidirectionalMatching; // check best match left->right and right->left
  const bool subpixelRefinement; // refine stereo matches with subpixel accuracy
  const bool equalizeImage_; // do equalize image before processing

public:
  StereoMatchingParams(const double in_tolTemplateMatching, const int in_templ_cols, const int in_templ_rows,
      const int in_stripe_extra_rows,
      const double in_minPointDist, const double in_maxPointDist, const bool in_bidirectionalMatching,
      const double in_nominalBaseline, const bool in_subpixelRefinement, const bool equalizeImage) :
        toleranceTemplateMatching(in_tolTemplateMatching), templ_cols(in_templ_cols), templ_rows(in_templ_rows), stripe_extra_rows(in_stripe_extra_rows),
        minPointDist(std::max(in_minPointDist,1e-3)), maxPointDist(in_maxPointDist), bidirectionalMatching(in_bidirectionalMatching),
        nominalBaseline(in_nominalBaseline), subpixelRefinement(in_subpixelRefinement), equalizeImage_(equalizeImage)
  {
    if(templ_cols % 2 != 1 || templ_rows % 2 != 1) // check that they are odd
      throw std::runtime_error("StereoMatchingParams: template size must be odd!");
    if(stripe_extra_rows % 2 != 0) // check that they are even
      throw std::runtime_error("StereoMatchingParams: stripe_extra_rows size must be even!");
  }
};

//////////////////////////////////////////////////////////////////////
struct KeypointWithDepth{
  KeypointCV px;
  double depth;

  // constructor
  KeypointWithDepth(const KeypointCV p, const double d) : px(p),depth(d) {}
  KeypointWithDepth() {}
};
using KeypointsWithDepth = std::vector<KeypointWithDepth>;

// Definitions relevant to StereoFrame type
using Points3d = std::vector<Vector3, Eigen::aligned_allocator<Vector3>>;

//////////////////////////////////////////////////////////////////////
class StereoFrame {
public:
  // constructor
  StereoFrame(const FrameId id, const int64_t timestamp,
      const std::string left_image_name, const std::string right_image_name,
      const CameraParams& cam_param_left, const CameraParams& cam_param_right,
      const gtsam::Pose3 L_Pose_R, const StereoMatchingParams stereoMatchingParams) :
        id_(id), timestamp_(timestamp),
        left_frame_(id,timestamp,left_image_name,cam_param_left,stereoMatchingParams.equalizeImage_),
        right_frame_(id,timestamp,right_image_name,cam_param_right,stereoMatchingParams.equalizeImage_),
        isRectified_(false), isKeyframe_(false), camL_Pose_camR(L_Pose_R),
        sparseStereoParams_(stereoMatchingParams) {}

public:
  struct LandmarkInfo{
    KeypointCV keypoint;
    double score;
    int age;
    gtsam::Vector3 keypoint_3d;
  };

  const FrameId id_;
  const int64_t timestamp_;

  Frame left_frame_, right_frame_;
  bool isRectified_; // make sure to do that on each captured image
  bool isKeyframe_;

  cv::Mat left_img_rectified_, right_img_rectified_;

  KeypointsCV left_keypoints_rectified_;
  KeypointsCV right_keypoints_rectified_;
  std::vector<Kstatus> right_keypoints_status_;
  std::vector<double> keypoints_depth_;
  std::vector<Vector3> keypoints_3d_; // in the ref frame of the UNRECTIFIED left frame

  StereoMatchingParams sparseStereoParams_;

  std::vector<cv::Vec6f> triangulation2Dobs_; // list of triangles such that each triangle defines
  // is a planar surface observed in by the stereo camera (each triangle is described by 3 pairs of pixels)

  // RELATIVE POSE BEFORE RECTIFICATION
  const gtsam::Pose3 camL_Pose_camR; // relative pose between left and right camera

  // QUANTITIES AFTER RECTIFICATION
  gtsam::Cal3_S2 left_undistRectCameraMatrix_, right_undistRectCameraMatrix_; // note: rectification
  // is something that belongs to a stereo camera, and that's why these are stored here!
  double baseline_; // after rectification!
  gtsam::Pose3 B_Pose_camLrect; // pose of the left camera wrt the body frame - after rectification!

public:
  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // NON-CONST functions:
  void setIsKeyframe(bool isKf){
    isKeyframe_ = isKf;
    left_frame_.isKeyframe_ = isKf;
    right_frame_.isKeyframe_ = isKf;
  }
  // Create a 2D mesh only including triangles corresponding to obstables (planar surfaces)
  // gradBound = 50 (255):if pixels in triangle have all grad smaller than gradBound, triangle is rejected
  void createMesh2Dobs(float gradBound = 50, double min_elongation_ratio = 0.5);
  // visualize stored mesh
  void visualizeMesh2Dobs(const double waitTime = 0) const;
  // copy rectification parameters from another stereo camera
  void cloneRectificationParameters(const StereoFrame& sf);
  // compute rectification parameters
  void computeRectificationParameters();
  // returns left and right rectified images and left and right rectified camera calibration
  void getRectifiedImages();
  // for each keypoint in the left frame, get (i) keypoint in right frame, (ii) depth, (iii) corresponding 3D point
  void sparseStereoMatching(const int verbosity = 0);
  // use optical flow to get right frame correspondences
  // deprecated
  void getRightKeypointsLKunrectified();
  /* --------------------------------------------------------------------------------------- */
  // CONST functions:
  // deprecated
  cv::Mat getDisparityImage(const cv::Mat imgLeft, const cv::Mat imgRight, const int verbosity = 0) const;
  StatusKeypointsCV undistortRectifyPoints(KeypointsCV left_keypoints_unrectified, const CameraParams cam_param,
      const gtsam::Cal3_S2 rectCameraMatrix) const;
  StatusKeypointsCV getRightKeypointsRectified(const cv::Mat left_rectified, const cv::Mat right_rectified,
      const StatusKeypointsCV left_keypoints_rectified, const double fx, const double baseline) const;
  std::vector<double> getDepthFromRectifiedMatches(StatusKeypointsCV& left_keypoints_rectified, StatusKeypointsCV& right_keypoints_rectified,
      const double fx, const double baseline) const;
  static std::pair<KeypointsCV, std::vector<Kstatus>> DistortUnrectifyPoints(const StatusKeypointsCV keypoints_rectified,
      const cv::Mat map_x, const cv::Mat map_y);
  void checkStereoFrame() const;
  std::pair<StatusKeypointCV,double> findMatchingKeypointRectified(const cv::Mat left_rectified, const KeypointCV left_rectified_i,
      const cv::Mat right_rectified, const int templ_cols, const int templ_rows, const int stripe_cols, const int stripe_rows,
      const double tol_corr, const bool debugStereoMatching = false) const;
  /* --------------------------------------------------------------------------------------- */
  // query 3D point for a valid landmark (with right pixel = VALID)
  gtsam::Point3 getPoint3DinCameraFrame(LandmarkId i) const{
    if(right_keypoints_status_.at(i)==Kstatus::VALID){
      throw std::runtime_error("getPoint3DinCameraFrame: asked for invalid keypoint3d");
    }
    return gtsam::Point3(keypoints_3d_.at(i));
  }
  /* --------------------------------------------------------------------------------------- */
  LandmarkInfo getLandmarkInfo(const LandmarkId i) const{
    // output to populate:
    LandmarkInfo lInfo;
    if(left_frame_.landmarks_.size() != keypoints_3d_.size())
      throw std::runtime_error("StereoFrame: getLandmarkKeypointAgekeypoint_3d size mismatch");
    if(left_frame_.landmarks_.size() != left_frame_.scores_.size())
      throw std::runtime_error("StereoFrame: scores_ size mismatch");

    for(size_t ind = 0; ind < left_frame_.landmarks_.size(); ind++){
      if(left_frame_.landmarks_.at(ind)==i){ // this is the desired landmark
        lInfo.keypoint = left_frame_.keypoints_.at(ind);
        lInfo.score = left_frame_.scores_.at(ind);
        lInfo.age = left_frame_.landmarksAge_.at(ind);
        lInfo.keypoint_3d = keypoints_3d_.at(ind);
        return lInfo;
      }
    }
    // if we got here without finding the landmark there is something wrong:
    throw std::runtime_error("getLandmarkKeypointAgeVersor: landmark not found");
  }
  /* --------------------------------------------------------------------------------------- */
  double baseline() const { return baseline_; }
  /* --------------------------------------------------------------------------------------- */
  void print() const
  {
    std::cout << "=====================" << std::endl;
    std::cout << "id_ " << id_ << std::endl;
    std::cout << "timestamp_ " << timestamp_ << std::endl;
    std::cout << "isRectified_ " << isRectified_ << std::endl;
    std::cout << "isKeyframe_ " << isKeyframe_ << std::endl;
    std::cout << "nr keypoints in left " << left_frame_.keypoints_.size() << std::endl;
    std::cout << "nr keypoints in right " << right_frame_.keypoints_.size() << std::endl;
    std::cout << "nr keypoints_depth_ " << keypoints_depth_.size() << std::endl;
    std::cout << "nr keypoints_3d_ " << keypoints_3d_.size() << std::endl;
    camL_Pose_camR.print("camL_Pose_camR \n");
    std::cout << "\n left_frame_.cam_param_.body_Pose_cam_ " << left_frame_.cam_param_.body_Pose_cam_ << std::endl;
    std::cout << "right_frame_.cam_param_.body_Pose_cam_ " << right_frame_.cam_param_.body_Pose_cam_ << std::endl;
  }
  /* --------------------------------------------------------------------------------------- */
  void showOriginal(const int verbosity) const
  {
    if (isRectified_){
      throw std::runtime_error("showOriginal: but images are already rectified");
    }else{
      showImagesSideBySide(left_frame_.img_,right_frame_.img_,"original: left-right",verbosity);
    }
  }
  /* --------------------------------------------------------------------------------------- */
  void showRectified(const int verbosity) const
  {
    if (!isRectified_){
      throw std::runtime_error("showRectified: please perform rectification before asking to visualize rectified images");
    }else{
      // showImagesSideBySide(left_frame_.img_,right_frame_.img_,"rectified: left-right");
      cv::Mat canvas_undistort = drawEpipolarLines(left_frame_.img_,right_frame_.img_, 15);
      if(verbosity>1){
        std::string img_name = "./outputImages/rectified_" + std::to_string(id_) + ".png";
        cv::imwrite(img_name, canvas_undistort);
      }
      cv::imshow("Rectified!", canvas_undistort);
      cv::waitKey(50);
    }
  }
  /* --------------------------------------------------------------------------------------- */
  void showImagesSideBySide(const cv::Mat imL, const cv::Mat imR, const std::string title, const int verbosity = 0) const
  {
    if(verbosity==0)
      return;

    cv::Mat originalLR = UtilsOpenCV::ConcatenateTwoImages(imL,imR);
    cv::namedWindow(title, cv::WINDOW_AUTOSIZE);
    if(verbosity==1){
      cv::imshow("originalLR", originalLR);
      cv::waitKey(200);
    }
    if(verbosity==2){
      std::string img_name = "./outputImages/" + title + std::to_string(id_) + ".png";
      cv::imwrite(img_name, originalLR);
    }
  }
  /* --------------------------------------------------------------------------------------- */
  cv::Mat drawEpipolarLines(const cv::Mat img1, const cv::Mat img2, const int numLines = 20, const int verbosity = 0) const {
    cv::Mat canvas = UtilsOpenCV::ConcatenateTwoImages(img1, img2);
    int lineGap = canvas.rows / (numLines + 1);
    for (int l = 0; l < numLines; l++) {
      float yPos = (l + 1) * lineGap;
      cv::line(canvas, cv::Point2f(0, yPos),
          cv::Point2f(canvas.cols - 1, yPos),
          cv::Scalar(0, 255, 0));
    }
    if(verbosity>1){
      std::string img_name = "./outputImages/drawEpipolarLines_" + std::to_string(id_) + ".png";
      cv::imwrite(img_name, canvas);
    }
    return canvas;
  }
  /* --------------------------------------------------------------------------------------- */
  void displayLeftRightMatches() const
  {
    if(left_frame_.keypoints_.size() != right_frame_.keypoints_.size())
      throw std::runtime_error("displayLeftRightMatches: error -  nr of corners in left and right cameras must be the same");

    // Draw the matchings: assumes that keypoints in the left and right keyframe are ordered in the same way
    std::vector<cv::DMatch> matches;
    for (int i = 0; i < left_frame_.keypoints_.size(); i++) {
      matches.push_back(cv::DMatch(i, i, 0));
    }
    cv::Mat match_vis = UtilsOpenCV::DrawCornersMatches(
        left_frame_.img_, left_frame_.keypoints_,
        right_frame_.img_, right_frame_.keypoints_, matches);
    cv::imshow("match_visualization", match_vis);
    cv::waitKey(50);
  }
  /* --------------------------------------------------------------------------------------- */
  // visualize statistics on the performance of the sparse stereo matching
  void displayKeypointStats(const StatusKeypointsCV right_keypoints_rectified) const
  {
    int nrValid = 0, nrNoLeftRect = 0, nrNoRightRect = 0, nrNoDepth = 0;
    for(size_t i=0; i<right_keypoints_rectified.size(); i++){
      if(right_keypoints_rectified.at(i).first == Kstatus::VALID)
        nrValid++;
      if(right_keypoints_rectified.at(i).first == Kstatus::NO_LEFT_RECT)
        nrNoLeftRect++;
      if(right_keypoints_rectified.at(i).first == Kstatus::NO_RIGHT_RECT)
        nrNoRightRect++;
      if(right_keypoints_rectified.at(i).first == Kstatus::NO_DEPTH)
        nrNoDepth++;
    }
    std::cout << "Nr of right keypoints: " << right_keypoints_rectified.size()  << " of which: " << std::endl <<
        "valid: " << nrValid << " nrNoLeftRect: " << nrNoLeftRect <<
        " nrNoRightRect: " << nrNoRightRect << " nrNoDepth: " << nrNoDepth << std::endl;
  }
};

} // namespace VIO

#endif /* StereoFrame_H_ */
