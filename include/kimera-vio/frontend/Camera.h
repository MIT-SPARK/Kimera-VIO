/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Camera.h
 * @brief  Class describing a monocular camera
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/**
 * @brief The Camera class Implements a Monocular Camera.
 * It currently assumes the camera has been undistorted.
 */
class Camera {
 public:
  KIMERA_POINTER_TYPEDEFS(Camera);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Camera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Camera
   * @param cam_params
   */
  Camera(const CameraParams& cam_params);
  virtual ~Camera() = default;

  // Camera implementation provided by gtsam
  using CameraImpl = gtsam::PinholeCamera<gtsam::Cal3_S2>;

 public:
  /**
   * @brief project 3D Lmks into images as 2D pixels, doesn't do any check...
   * @param lmks Given in World coordinates or rather in whatever frame of
   * reference the pose of the camera is given (usually the body frame!).
   * @param kpts 2D pixel coordinates of the projection of the 3D landmark
   */
  void project(const LandmarksCV& lmks, KeypointsCV* kpts) const;
  void project(const LandmarkCV& lmks, KeypointCV* kpts) const;

  /**
   * @brief backProject keypoints given depth projected to landmarks in 3D
   * In body frame of reference (not camera).
   * @param[in] kps keypoints in 2D
   * @param[in] depths of each keypoint
   * @param[out] lmks 3D Landmarks
   */
  void backProject(const KeypointsCV& kps,
                   const Depths& depths,
                   LandmarksCV* lmks) const;
  void backProject(const KeypointCV& kp,
                   const Depth& depth,
                   LandmarkCV* lmk) const;

  void undistortKeypoints(const KeypointsCV& keypoints,
                          StatusKeypointsCV* status_keypoints) const;

  /**
   * @brief getCalibration
   * @return  The intrinsic calibration of the camera
   */
  inline gtsam::Cal3_S2 getCalibration() const { return calibration_; }
  inline gtsam::Pose3 getBodyPoseCam() const {
    return cam_params_.body_Pose_cam_;
  }
  inline const CameraParams& getCamParams() const { return cam_params_; }

  // TODO(marcus): these are here only because omnicam undistort
  // happens here instead of in UndistorterRectifier.
  // final solution is to move logic to a gtsam camera for omni
  // and call in UndistorterRectifier
  // Implemented according to this paper:
  // http://rpg.ifi.uzh.ch/docs/CCMVS2007_scaramuzza.pdf
  static void BackProjectOmni(const KeypointsCV& kps,
                              const Depths& depths,
                              const CameraParams& cam_params,
                              LandmarksCV* lmks);
  static void BackProjectOmni(const KeypointCV& kp,
                              const Depth& depth,
                              const CameraParams& cam_params,
                              LandmarkCV* lmk);
  static void UndistortKeypointsOmni(const KeypointsCV& keypoints,
                                     const CameraParams& cam_params,
                                     boost::optional<cv::Mat> P,
                                     KeypointsCV* status_keypoints);

 protected:
  void projectPinhole(const LandmarksCV& lmks, KeypointsCV* kpts) const;
  void projectPinhole(const LandmarkCV& lmks, KeypointCV* kpts) const;

  void backProjectPinhole(const KeypointsCV& kp,
                          const Depths& depth,
                          LandmarksCV* lmk) const;
  void backProjectPinhole(const KeypointCV& kp,
                          const Depth& depth,
                          LandmarkCV* lmk) const;

  void projectOmni(const LandmarksCV& lmks, KeypointsCV* Kpts) const;
  void projectOmni(const LandmarkCV& lmks, KeypointCV* Kpts) const;

  void backProjectOmni(const KeypointsCV& kps,
                       const Depths& depths,
                       LandmarksCV* lmks) const;
  void backProjectOmni(const KeypointCV& kp,
                       const Depth& depth,
                       LandmarkCV* lmk) const;

 protected:
  CameraParams cam_params_;
  gtsam::Cal3_S2 calibration_;
  UndistorterRectifier::UniquePtr undistorter_;
  std::unique_ptr<CameraImpl> camera_impl_;
};

}  // namespace VIO
