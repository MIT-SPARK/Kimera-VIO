/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   CameraParams.h
 * @brief  Parameters describing a monocular camera.
 * @author Antoni Rosinol
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

enum class DistortionModel {
  NONE,
  RADTAN,
  EQUIDISTANT,
};

/**
 * @brief The CameraParams class Contains intrinsics and extrinsics of a single
 * camera, as well as distortion coefficients. Also has some miscellaneous
 * information (frame_rate, image_size, etc).
 */
class CameraParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(CameraParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using CameraId = std::string;
  // fu, fv, cu, cv
  using Intrinsics = std::array<double, 4>;
  using Distortion = std::vector<double>;

  CameraParams()
      : PipelineParams("Camera Parameters"),
        camera_id_(),
        camera_model_(),
        intrinsics_(),
        K_(),
        body_Pose_cam_(),
        frame_rate_(),
        image_size_(),
        distortion_model_(),
        distortion_coeff_(),
        distortion_coeff_mat_() {}
  virtual ~CameraParams() = default;

  /**
   * @brief parseYAML
   * Parse YAML file describing camera parameters.
   * @param filepath Path to the yaml file with the params
   * @return
   */
  bool parseYAML(const std::string& filepath) override;

  //! Display all params.
  void print() const override;

  //! Assert equality up to a tolerance.
  bool equals(const CameraParams& cam_par, const double& tol = 1e-9) const;

 protected:
  bool equals(const PipelineParams& rhs) const override {
    return equals(static_cast<const CameraParams&>(rhs), 1e-9);
  }

 public:
  //! Id of the camera
  CameraId camera_id_;

  //! Camera model: pinhole, etc
  std::string camera_model_;

  //! fu, fv, cu, cv
  Intrinsics intrinsics_;
  //! OpenCV structures: needed to compute the undistortion map.
  //! 3x3 camera matrix K (last row is {0,0,1})
  cv::Mat K_;

  //! Sensor extrinsics wrt body-frame
  gtsam::Pose3 body_Pose_cam_;

  //! Image info.
  double frame_rate_;
  cv::Size image_size_;

  //! Distortion parameters
  DistortionModel distortion_model_;
  std::vector<double> distortion_coeff_;
  cv::Mat distortion_coeff_mat_;

 public:
  static void convertDistortionVectorToMatrix(
      const std::vector<double>& distortion_coeffs,
      cv::Mat* distortion_coeffs_mat);
  static void convertIntrinsicsVectorToMatrix(const Intrinsics& intrinsics,
                                              cv::Mat* camera_matrix);
  static void createGtsamCalibration(const cv::Mat& distortion,
                                     const Intrinsics& intrinsics,
                                     gtsam::Cal3DS2* calibration);

  /** Taken from: https://github.com/ethz-asl/image_undistort
   * @brief stringToDistortion
   * @param distortion_model
   * @param camera_model
   * @return actual distortion model enum class
   */
  static const DistortionModel stringToDistortion(
      const std::string& distortion_model,
      const std::string& camera_model);

 private:
  void parseDistortion(const YamlParser& yaml_parser);
  static void parseImgSize(const YamlParser& yaml_parser, cv::Size* image_size);
  static void parseFrameRate(const YamlParser& yaml_parser, double* frame_rate);
  static void parseBodyPoseCam(const YamlParser& yaml_parser,
                               gtsam::Pose3* body_Pose_cam);
  static void parseCameraIntrinsics(const YamlParser& yaml_parser,
                                    Intrinsics* intrinsics_);
};
// TODO(Toni): this should be a base class, so that stereo camera is a specific
// type of a multi camera sensor rig, or something along these lines.
typedef std::vector<CameraParams> MultiCameraParams;

}  // namespace VIO
