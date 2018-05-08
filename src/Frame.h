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
#include <boost/foreach.hpp>

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

#include <glog/logging.h>

namespace VIO {

////////////////////////////////////////////////////////////////////////////
// Class for storing/processing a single image
class Frame {

public:
  // constructors
  Frame(const FrameId id,
        const int64_t timestamp,
        const std::string img_name,
        const CameraParams& cam_param,
        const bool equalizeImage = false):
    id_(id),
    timestamp_(timestamp),
    img_(UtilsOpenCV::ReadAndConvertToGrayScale(img_name, equalizeImage)),
    cam_param_(cam_param),
    isKeyframe_(false) {}

  // copy constructor
  Frame(const Frame& f) :
    id_(f.id_), timestamp_(f.timestamp_), img_(f.img_),
    cam_param_(f.cam_param_), isKeyframe_(f.isKeyframe_),
    keypoints_(f.keypoints_), scores_(f.scores_),
    landmarks_(f.landmarks_), landmarksAge_(f.landmarksAge_),
    versors_(f.versors_) {}

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
  std::vector<cv::Vec6f> triangulation2D_;

public:
  /* ++++++++++++++++++++++ NONCONST FUNCTIONS ++++++++++++++++++++++++++++++ */
  // ExtractCorners using goodFeaturesToTrack
  void extractCorners(const double qualityLevel = 0.01,
                      const double minDistance = 10,
                      const int blockSize = 3,
                      const bool useHarrisDetector = false,
                      const double k = 0.04,
                      const int maxCorners = 100) {
    keypoints_ = UtilsOpenCV::ExtractCorners(img_, qualityLevel, minDistance,
                                             blockSize, k, maxCorners,
                                             useHarrisDetector);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // NOT TESTED:
  void setLandmarksToMinus1(const LandmarkIds& lmkIds) {
    // TODO: this has quadratic complexity
    for (const LandmarkId& lmkId: lmkIds) {
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
      if (!found) {
        throw std::runtime_error("setLandmarksToMinus1: lmk not found");
      }
    }
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  void createMesh2D() { // called without input, it considers all valid keypoints for the mesh
    std::vector<int> selectedIndices;
    selectedIndices.reserve(keypoints_.size()); // preallocate
    for(int i = 0; i < keypoints_.size(); i++) {
      selectedIndices.push_back(i);
    }
    triangulation2D_ = createMesh2D(*this, selectedIndices);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static std::vector<cv::Vec6f> createMesh2D(const Frame& frame,
                                      const std::vector<int>& selectedIndices) {

    if(frame.landmarks_.size() != frame.keypoints_.size()) // sanity check
      throw std::runtime_error("Frame: wrong dimension for the landmarks");

    cv::Size size = frame.img_.size();
    cv::Rect2f rect(0, 0, size.width, size.height);

    // add points from Frame
    std::vector<cv::Point2f> keypointsToTriangulate;
    BOOST_FOREACH(int i, selectedIndices){
      cv::Point2f kp_i = cv::Point2f(float(frame.keypoints_.at(i).x),
                                     float(frame.keypoints_.at(i).y));
      if (frame.landmarks_.at(i) != -1 && rect.contains(kp_i)) { // only for valid keypoints (some keypoints may
        // end up outside image after tracking which causes subdiv to crash)
        keypointsToTriangulate.push_back(kp_i);
      }
    }
    return createMesh2D(frame.img_.size(), keypointsToTriangulate);
  }

  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static std::vector<cv::Vec6f> createMesh2D(
                       const cv::Size& img_size,
                       std::vector<cv::Point2f>& keypoints_to_triangulate) {

    // Define output (+ a temporary variable).
    std::vector<cv::Vec6f> triangulation2D, triangulation2DwithExtraTriangles;
    if (keypoints_to_triangulate.size() == 0)
      return triangulation2D; //nothing to triangulate

    // Rectangle to be used with Subdiv2D.
    cv::Rect2f rect (0, 0, img_size.width, img_size.height);
    cv::Subdiv2D subdiv(rect); // subdiv has the delaunay triangulation function

    // TODO Luca: there are kpts outside image, probably from tracker. This check
    // should be in the tracker.
    // -> Make sure we only pass keypoints inside the image!
    for (auto it = keypoints_to_triangulate.begin();
         it != keypoints_to_triangulate.end(); it++) {
      if (!rect.contains(*it)) {
        LOG(ERROR) << "createMesh2D - error, keypoint out of image frame.";
        keypoints_to_triangulate.erase(it);
        // Go backwards, otherwise it++ will jump one keypoint...
        it--;
      }
    }

    // perform triangulation
    try {
      subdiv.insert(keypoints_to_triangulate);
    } catch(...) {
      std::cout << keypoints_to_triangulate.size() << std::endl;
      std::runtime_error("CreateMesh2D: subdiv.insert error (2)");
    }

    // getTriangleList returns some spurious triangle with vertices outside image
    // TODO I think that the spurious triangles are due to ourselves sending keypoints
    // out of the image...
    subdiv.getTriangleList(triangulation2DwithExtraTriangles); // do triangulation

    // retrieve "good triangles" (all vertices are inside image)
    std::vector<cv::Point2f> pt(3);
    for(size_t i = 0; i < triangulation2DwithExtraTriangles.size(); i++) {
      cv::Vec6f t = triangulation2DwithExtraTriangles[i];
      pt[0] = cv::Point2f(t[0], t[1]);
      pt[1] = cv::Point2f(t[2], t[3]);
      pt[2] = cv::Point2f(t[4], t[5]);
      if(rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])){
        triangulation2D.push_back(t);
      }
    }

    return triangulation2D;
  }

  /* ------------------------------------------------------------------------ */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  void visualizeMesh2D(const double& waitTime = 0) const {
    cv::Scalar delaunay_color(0, 255, 0), points_color(255, 0, 0);

    // Sanity check.
    if(landmarks_.size() != keypoints_.size())
      throw std::runtime_error("Frame: wrong dimension for the landmarks");

    // Duplicate image for annotation and visualization.
    cv::Mat img = img_.clone();
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    cv::Size size = img.size();
    cv::Rect rect(0, 0, size.width, size.height);
    std::vector<cv::Point> pt(3);
    for (size_t i = 0; i < triangulation2D_.size(); i++) {
      cv::Vec6f t = triangulation2D_[i];

      // Visualize mesh vertices.
      pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
      pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
      pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

      // Visualize mesh edges.
      cv::line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
      cv::line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
      cv::line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
    }

    // Visualize extra vertices.
    for (size_t i = 0; i < keypoints_.size(); i++) {
      if(landmarks_.at(i) != -1) // only for valid keypoints
        cv::circle(img, keypoints_.at(i), 2, points_color, CV_FILLED, CV_AA, 0);
    }

    cv::imshow("visualizeMesh2D", img);
    cv::waitKey(waitTime);
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
  //    cv::Mat undistortedImage, undistortedCameraMatrix,undist_map_x,undist_map_y;
  //    cv::Size imageSize;
  //    cv::initUndistortRectifyMap(cam_param_.camera_matrix_, cam_param_.distortion_coeff_, cv::Mat(),
  //        undistortedCameraMatrix, imageSize, CV_16SC2, undist_map_x, undist_map_y);
  //
  //    cv::remap(img_, undistortedImage, undist_map_x, undist_map_y, cv::INTER_LINEAR);
  //    return undistortedImage;
  //  }

  /* ------------------------------------------------------------------------ */
  size_t getNrValidKeypoints() const {
    size_t count = 0;
    for (size_t i = 0; i < landmarks_.size(); i++) {
      if(landmarks_.at(i)!=-1)// It is valid.
        count += 1;
    }
    return count;
  }

  /* ------------------------------------------------------------------------ */
  KeypointsCV getValidKeypoints() const {
    KeypointsCV validKeypoints;
    for (size_t i = 0; i < landmarks_.size(); i++) {
      if (landmarks_.at(i) != -1) {// It is valid.
        validKeypoints.push_back(keypoints_[i]);
      }
    }
    return validKeypoints;
  }

  /* ------------------------------------------------------------------------ */
  LandmarkId findLmkIdFromPixel(
                  const KeypointCV& px,
                  boost::optional<int &> idx_in_keypoints = boost::none) const {

    // Iterate over all current keypoints_.
    for (int i = 0; i < keypoints_.size(); i++) {
      // If we have found the pixel px in the set of keypoints, return the
      // landmark id of the keypoint and return the index of it at keypoints_.
      if (keypoints_.at(i).x == px.x &&
          keypoints_.at(i).y == px.y) {
        if (idx_in_keypoints) {
          *idx_in_keypoints = i; // Return index.
        }
        return landmarks_.at(i);
      }
    }
    // We did not find the keypoint.
    if (idx_in_keypoints) idx_in_keypoints = boost::optional<int &>();
    return -1;
  }

  /* ------------------------------------------------------------------------ */
  void print() const {
    std::cout << "Frame id: " << id_ <<  " at timestamp: " << timestamp_ << "\n"
              << "isKeyframe_: " << isKeyframe_ << "\n"
              << "nr keypoints_: " << keypoints_.size() << "\n"
              << "nr valid keypoints_: " << getNrValidKeypoints() << "\n"
              << "nr landmarks_: " << landmarks_.size() << "\n"
              << "nr versors_: " << versors_.size() << "\n"
              << "size descriptors_: " << descriptors_.size() << std::endl;
    cam_param_.print();
  }

  /* ------------------------------------------------------------------------ */
  static Vector3 CalibratePixel(const KeypointCV& cv_px,
                                const CameraParams cam_param) {
    // Calibrate pixel.
    cv::Mat_<KeypointCV> uncalibrated_px(1, 1); // matrix of px with a single entry, i.e., a single pixel
    uncalibrated_px(0) = cv_px;
    cv::Mat calibrated_px;

    cv::undistortPoints(uncalibrated_px, calibrated_px,
                        cam_param.camera_matrix_, cam_param.distortion_coeff_);

    // Transform to unit vector.
    Vector3 versor = Vector3(calibrated_px.at<float>(0, 0),
                             calibrated_px.at<float>(0, 1), 1.0);

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
