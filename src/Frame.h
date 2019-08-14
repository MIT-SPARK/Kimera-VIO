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

#include <stdio.h>
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <cstdlib>
#include <numeric>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>

#include "CameraParams.h"
#include "UtilsOpenCV.h"

namespace VIO {

////////////////////////////////////////////////////////////////////////////
// Class for storing/processing a single image
class Frame {
 public:
  // Constructors.
  /// @param img: does a shallow copy of the image by defaults,
  ///  if Frame should have ownership of the image, clone it.
  Frame(const FrameId& id, const int64_t& timestamp,
        const CameraParams& cam_param, const cv::Mat& img)
      : id_(id),
        timestamp_(timestamp),
        cam_param_(cam_param),
        img_(img),
        isKeyframe_(false) {}

  // TODO delete Frame copy constructor
  // Copy constructor.
  // Also does a shallow copy of the image!
  Frame(const Frame& f)
      : id_(f.id_),
        timestamp_(f.timestamp_),
        cam_param_(f.cam_param_),
        img_(f.img_),
        isKeyframe_(f.isKeyframe_),
        keypoints_(f.keypoints_),
        scores_(f.scores_),
        landmarks_(f.landmarks_),
        landmarksAge_(f.landmarksAge_),
        versors_(f.versors_) {}

  const FrameId id_;
  const Timestamp timestamp_;

  // These are non-const since they will be changed during rectification.
  CameraParams cam_param_;

  // Actual image stored by the class frame.
  // This must be const otw, we have to reimplement the copy ctor to allow
  // for deep copies.
  const cv::Mat img_;

  // Results of image processing.
  bool isKeyframe_ = false;

  // These containers must have same size.
  KeypointsCV keypoints_;
  std::vector<double> scores_;  // quality of extracted keypoints
  LandmarkIds landmarks_;
  std::vector<int>
      landmarksAge_;        // how many consecutive *keyframes* saw the keypoint
  BearingVectors versors_;  // in the ref frame of the UNRECTIFIED left frame
  cv::Mat descriptors_;     // not currently used
  std::vector<cv::Vec6f> triangulation2D_;

 public:
  /* ++++++++++++++++++++++ NONCONST FUNCTIONS ++++++++++++++++++++++++++++++ */
  // ExtractCorners using goodFeaturesToTrack
  // This is only used for testing purposes.
  // TODO, define it in the tests, rather than here, as it distracts coders,
  // since it is not actually used by the VIO pipeline.
  void extractCorners(const double qualityLevel = 0.01,
                      const double minDistance = 10, const int blockSize = 3,
                      const bool useHarrisDetector = false,
                      const double k = 0.04) {
    UtilsOpenCV::ExtractCorners(img_, &keypoints_, qualityLevel, minDistance,
                                blockSize, k, useHarrisDetector);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // NOT TESTED:
  void setLandmarksToMinus1(const LandmarkIds& lmkIds) {
    // TODO: this has quadratic complexity
    for (const LandmarkId& lmkId : lmkIds) {
      // For each landmark we want to discard.
      bool found = false;
      for (size_t ind = 0; ind < landmarks_.size(); ind++) {
        // We look for it among landmarks_.
        if (landmarks_.at(ind) == lmkId) {
          // We found it, so we set it to -1.
          landmarks_.at(ind) = -1;
          found = true;
          break;
        }
      }
      CHECK(found) << "setLandmarksToMinus1: lmk not found";
    }
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  // It considers all valid keypoints for the mesh.
  // Optionally, it returns a 2D mesh via its argument.
  std::vector<cv::Vec6f> createMesh2D() const {
    std::vector<size_t> selectedIndices(keypoints_.size());
    // Fills selectedIndices with the indices of ALL keypoints: 0, 1, 2...
    std::iota(selectedIndices.begin(), selectedIndices.end(), 0);
    return createMesh2D(*this, selectedIndices);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static std::vector<cv::Vec6f> createMesh2D(
      const Frame& frame, const std::vector<size_t>& selectedIndices) {
    // Sanity check.
    CHECK_EQ(frame.landmarks_.size(), frame.keypoints_.size())
        << "Frame: wrong dimension for the landmarks";

    cv::Size size = frame.img_.size();
    cv::Rect2f rect(0, 0, size.width, size.height);

    // Add points from Frame.
    std::vector<cv::Point2f> keypointsToTriangulate;
    for (const auto& i : selectedIndices) {
      cv::Point2f kp_i(float(frame.keypoints_.at(i).x),
                       float(frame.keypoints_.at(i).y));
      if (frame.landmarks_.at(i) != -1 && rect.contains(kp_i)) {
        // Only for valid keypoints (some keypoints may
        // end up outside image after tracking which causes subdiv to crash).
        keypointsToTriangulate.push_back(kp_i);
      }
    }
    return createMesh2D(frame.img_.size(), &keypointsToTriangulate);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  // Returns the actual keypoints used to perform the triangulation.
  static std::vector<cv::Vec6f> createMesh2D(
      const cv::Size& img_size,
      std::vector<cv::Point2f>* keypoints_to_triangulate) {
    CHECK_NOTNULL(keypoints_to_triangulate);
    // Nothing to triangulate.
    if (keypoints_to_triangulate->size() == 0) return std::vector<cv::Vec6f>();

    // Rectangle to be used with Subdiv2D.
    cv::Rect2f rect(0, 0, img_size.width, img_size.height);
    cv::Subdiv2D subdiv(
        rect);  // subdiv has the delaunay triangulation function

    // TODO Luca: there are kpts outside image, probably from tracker. This
    // check should be in the tracker.
    // -> Make sure we only pass keypoints inside the image!
    for (auto it = keypoints_to_triangulate->begin();
         it != keypoints_to_triangulate->end();) {
      if (!rect.contains(*it)) {
        LOG(ERROR) << "createMesh2D - error, keypoint out of image frame.";
        it = keypoints_to_triangulate->erase(it);
        // Go backwards, otherwise it++ will jump one keypoint...
      } else {
        it++;
      }
    }

    // Perform triangulation.
    try {
      subdiv.insert(*keypoints_to_triangulate);
    } catch (...) {
      LOG(FATAL) << "CreateMesh2D: subdiv.insert error (2).\n Keypoints to "
                    "triangulate: "
                 << keypoints_to_triangulate->size();
    }

    // getTriangleList returns some spurious triangle with vertices outside
    // image
    // TODO I think that the spurious triangles are due to ourselves sending
    // keypoints out of the image... Compute actual triangulation.
    std::vector<cv::Vec6f> triangulation2D;
    subdiv.getTriangleList(triangulation2D);

    // Retrieve "good triangles" (all vertices are inside image).
    for (auto it = triangulation2D.begin(); it != triangulation2D.end();) {
      if (!rect.contains(cv::Point2f((*it)[0], (*it)[1])) ||
          !rect.contains(cv::Point2f((*it)[2], (*it)[3])) ||
          !rect.contains(cv::Point2f((*it)[4], (*it)[5]))) {
        it = triangulation2D.erase(it);
        // Go backwards, otherwise it++ will jump one keypoint...
      } else {
        it++;
      }
    }
    return triangulation2D;
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
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

  /* ----------------------- CONST FUNCTIONS -------------------------------- */
  // NOT TESTED: undistort and return
  //  cv::Mat undistortImage() const {
  //    cv::Mat undistortedImage,
  //    undistortedCameraMatrix,undist_map_x,undist_map_y; cv::Size imageSize;
  //    cv::initUndistortRectifyMap(cam_param_.camera_matrix_,
  //    cam_param_.distortion_coeff_, cv::Mat(),
  //        undistortedCameraMatrix, imageSize, CV_16SC2, undist_map_x,
  //        undist_map_y);
  //
  //    cv::remap(img_, undistortedImage, undist_map_x, undist_map_y,
  //    cv::INTER_LINEAR); return undistortedImage;
  //  }

  /* ------------------------------------------------------------------------ */
  size_t getNrValidKeypoints() const {
    // TODO: can we cache this value?
    size_t count = 0;
    for (size_t i = 0; i < landmarks_.size(); i++) {
      if (landmarks_.at(i) != -1)  // It is valid.
        count += 1;
    }
    return count;
  }

  /* ------------------------------------------------------------------------ */
  KeypointsCV getValidKeypoints() const {
    // TODO: can we cache this result?
    KeypointsCV validKeypoints;
    for (size_t i = 0; i < landmarks_.size(); i++) {
      if (landmarks_.at(i) != -1) {  // It is valid.
        validKeypoints.push_back(keypoints_[i]);
      }
    }
    return validKeypoints;
  }

  /* ------------------------------------------------------------------------ */
  LandmarkId findLmkIdFromPixel(
      const KeypointCV& px,
      boost::optional<int&> idx_in_keypoints = boost::none) const {
    // Iterate over all current keypoints_.
    for (int i = 0; i < keypoints_.size(); i++) {
      // If we have found the pixel px in the set of keypoints, return the
      // landmark id of the keypoint and return the index of it at keypoints_.
      if (keypoints_.at(i).x == px.x && keypoints_.at(i).y == px.y) {
        if (idx_in_keypoints) {
          *idx_in_keypoints = i;  // Return index.
        }
        return landmarks_.at(i);
      }
    }
    // We did not find the keypoint.
    if (idx_in_keypoints) idx_in_keypoints = boost::optional<int&>();
    return -1;
  }

  /* ------------------------------------------------------------------------ */
  void print() const {
    LOG(INFO) << "Frame id: " << id_ << " at timestamp: " << timestamp_ << "\n"
              << "isKeyframe_: " << isKeyframe_ << "\n"
              << "nr keypoints_: " << keypoints_.size() << "\n"
              << "nr valid keypoints_: " << getNrValidKeypoints() << "\n"
              << "nr landmarks_: " << landmarks_.size() << "\n"
              << "nr versors_: " << versors_.size() << "\n"
              << "size descriptors_: " << descriptors_.size();
    cam_param_.print();
  }

  /* ------------------------------------------------------------------------ */
  static Vector3 CalibratePixel(const KeypointCV& cv_px,
                                const CameraParams& cam_param) {
    // Calibrate pixel.
    cv::Mat_<KeypointCV> uncalibrated_px(
        1, 1);  // matrix of px with a single entry, i.e., a single pixel
    uncalibrated_px(0) = cv_px;
    cv::Mat calibrated_px;

    if (cam_param.distortion_model_ == "radtan" ||
        cam_param.distortion_model_ == "radial-tangential") {
      // TODO optimize this in just one call, the s in Points is there for
      // something I hope.
      cv::undistortPoints(uncalibrated_px, calibrated_px,
                          cam_param.camera_matrix_,
                          cam_param.distortion_coeff_);
    } else if ((cam_param.distortion_model_ == "equidistant")) {
      // TODO: Create unit test for fisheye / equidistant model
      cv::fisheye::undistortPoints(uncalibrated_px, calibrated_px,
                                   cam_param.camera_matrix_,
                                   cam_param.distortion_coeff_);
    } else {
      LOG(ERROR) << "Camera distortion model not found in CalibratePixel()!";
    }

    // Transform to unit vector.
    Vector3 versor(calibrated_px.at<float>(0, 0), calibrated_px.at<float>(0, 1),
                   1.0);

    // sanity check, try to distort point using gtsam and make sure you get
    // original pixel
    // gtsam::Point2 uncalibrated_px_gtsam =
    // cam_param.calibration_.uncalibrate(gtsam::Point2(versor(0),versor(1)));
    // gtsam::Point2 uncalibrated_px_opencv =
    // gtsam::Point2(uncalibrated_px.at<float>(0,0),uncalibrated_px.at<float>(0,1));
    // gtsam::Point2 px_mismatch  = uncalibrated_px_opencv -
    // uncalibrated_px_gtsam;
    //
    // if(px_mismatch.vector().norm() > 1){
    //  std::cout << "uncalibrated_px: \n" << uncalibrated_px << std::endl;
    //  std::cout << "uncalibrated_px_gtsam: \n" << uncalibrated_px_gtsam <<
    //  std::endl; std::cout << "px_mismatch: \n" << px_mismatch << std::endl;
    //  throw std::runtime_error("CalibratePixel: possible calibration
    //  mismatch");
    //}
    return versor / versor.norm();  // return unit norm vector
  }
};

}  // namespace VIO
#endif /* Frame_H_ */
