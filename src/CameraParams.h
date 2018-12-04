/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   CameraParams.h
 * @brief  Parameters describing a monocular camera parameters
 * @author Luca Carlone
 */

#ifndef CameraParams_H_
#define CameraParams_H_

#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include "UtilsOpenCV.h"

namespace VIO {

// shortcuts
using Calibration = gtsam::Cal3DS2;

/*
 * Class describing the camera parameters, to be read from the ETH dataset
 */
class CameraParams {
public:
  std::vector<double> intrinsics_; // fu, fv, cu, cv
  gtsam::Pose3 body_Pose_cam_;  // Sensor extrinsics wrt. the body-frame

  // image info
  double frame_rate_;
  cv::Size image_size_;

  // GTSAM structures: to calibrate points
  Calibration calibration_;

  // OpenCV structures: For radial distortion and rectification
  cv::Mat camera_matrix_;    // needed to compute the undistorsion map
  cv::Mat distortion_coeff_; // 5 parameters (last is zero): distortion_model: radial-tangential

  cv::Mat undistRect_map_x_, undistRect_map_y_;

  cv::Mat R_rectify_;   // rotation resulting from rectification
  cv::Mat P_; // camera matrix after rectification

  /* ---------------------------------------------------------------------- */
  // parse YAML file describing camera parameters
  bool parseYAML(std::string filepath){
    // make sure that each YAML file has %YAML:1.0 as first line
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cout << "Cannot open file in parseYAML: " << filepath << std::endl;
      throw std::runtime_error("parseYAML (CameraParams): cannot open file (remember first line: %YAML:1.0)");
    }

    // intrinsics
    intrinsics_.clear();
    fs["intrinsics"] >> intrinsics_;

    // 4 parameters (read from file): distortion_model: radial-tangential
    // TODO allow for different distortion models, at least equidistant as well!
    std::vector<double> distortion_coeff4_;
    distortion_coeff4_.clear();
    fs["distortion_coefficients"] >> distortion_coeff4_;
    // Convert distortion coefficients to OpenCV Format
    distortion_coeff_ = cv::Mat::zeros(1, 5, CV_64F);
    for (int k = 0; k < 4; k++) {
      distortion_coeff_.at<double>(0, k) = distortion_coeff4_[k];
    }

    // camera resolution
    std::vector<int> resolution;
    resolution.clear();
    fs["resolution"] >> resolution;
    image_size_ = cv::Size(resolution[0], resolution[1]);

    // Camera frame rate
    int rate = fs["rate_hz"];
    frame_rate_ = 1 / double(rate);

    // body_Pose_cam_
    int n_rows = (int)fs["T_BS"]["rows"];
    int n_cols = (int)fs["T_BS"]["cols"];
    std::vector<double> vec;
    fs["T_BS"]["data"] >> vec;
    body_Pose_cam_ = UtilsOpenCV::Vec2pose(vec,n_rows,n_cols);

    fs.release();

    // Convert intrinsics to OpenCV Format
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = intrinsics_[0];
    camera_matrix_.at<double>(1, 1) = intrinsics_[1];
    camera_matrix_.at<double>(0, 2) = intrinsics_[2];
    camera_matrix_.at<double>(1, 2) = intrinsics_[3];

    calibration_ =  gtsam::Cal3DS2(intrinsics_[0], // fx
        intrinsics_[1], // fy
        0.0,           // skew
        intrinsics_[2], // u0
        intrinsics_[3], // v0
        distortion_coeff4_[0],  //  k1
        distortion_coeff4_[1],  //  k2
        distortion_coeff4_[2],  //  p1
        distortion_coeff4_[3]); //  p2

    return true;
  }
  /* ---------------------------------------------------------------------- */
  // display all params
  void print() const {
    std::cout << "------------ CameraParams::print -------------" << std::endl;
    std::cout << "intrinsics_: ";
    for(size_t i=0;i<intrinsics_.size();i++)
      std::cout <<  intrinsics_.at(i) << " , ";
    std::cout << std::endl;

    std::cout << "body_Pose_cam_: \n" << body_Pose_cam_ << std::endl;

    calibration_.print("gtsam calibration:\n");

    std::cout << "frame_rate_: " << frame_rate_ << std::endl;
    std::cout << "image_size_: width= " << image_size_.width << " height= " << image_size_.height << std::endl;
    std::cout << "camera_matrix_: \n" << camera_matrix_ << std::endl;
    std::cout << "distortion_coeff_: \n" << distortion_coeff_ << std::endl;
    std::cout << "R_rectify_: \n" << R_rectify_ << std::endl;
    std::cout << "undistRect_map_y_ too large to display (only created in StereoFrame)" << std::endl;
    std::cout << "P_: \n" << P_ << std::endl;
  }
  /* ---------------------------------------------------------------------- */
  // assert equality up to a tolerance
  bool equals(const CameraParams& camPar, double tol = 1e-9) const {
    bool areIntrinsicEqual = true;
    for(size_t i=0; i<intrinsics_.size();i++){
      if( fabs( intrinsics_[i] - camPar.intrinsics_[i] ) > tol )
        areIntrinsicEqual = false;
    }
    return body_Pose_cam_.equals(camPar.body_Pose_cam_,tol) &&
        areIntrinsicEqual &&
        ( fabs(frame_rate_ - camPar.frame_rate_) < tol)  &&
        ( image_size_.width == camPar.image_size_.width)  &&
        ( image_size_.height == camPar.image_size_.height)  &&
        calibration_.equals(camPar.calibration_,tol) &&
        UtilsOpenCV::CvMatCmp(camera_matrix_,camPar.camera_matrix_) &&
        UtilsOpenCV::CvMatCmp(distortion_coeff_,camPar.distortion_coeff_) &&
        UtilsOpenCV::CvMatCmp(undistRect_map_x_,camPar.undistRect_map_x_) &&
        UtilsOpenCV::CvMatCmp(undistRect_map_y_,camPar.undistRect_map_y_) &&
        UtilsOpenCV::CvMatCmp(R_rectify_,camPar.R_rectify_) &&
        UtilsOpenCV::CvMatCmp(P_,camPar.P_);
  }
};

} // namespace VIO

#endif /* CameraParams_H_ */
