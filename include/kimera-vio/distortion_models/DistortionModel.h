/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DistortionModel.h
 * @brief  base class for distortion (camera) models
 * @author Bernd Pfrommer
 */

#ifndef INCLUDE_KIMERA_VIO_DISTORTION_MODELS_DISTORTIONMODEL_H_
#define INCLUDE_KIMERA_VIO_DISTORTION_MODELS_DISTORTIONMODEL_H_

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace VIO {
/*
 * base class for distortion models (radtan, equidistant etc)
 */
class DistortionModel {
 public:
  virtual ~DistortionModel() {}
  typedef std::array<double, 4> Intrinsics;
  // ----- methods to be implemented by the derived classes ----

  virtual gtsam::Point2 uncalibrate(const gtsam::Point2& p) const = 0;

  virtual gtsam::Point2 project(const gtsam::Pose3& pose,
                                const gtsam::Point3& p) const = 0;

  virtual bool equals(
      const std::shared_ptr<const DistortionModel>& dm) const = 0;

  virtual void print() const = 0;

  virtual bool test(const double* intr, const double* dist) const = 0;

  virtual double fx() const = 0;
  virtual double fy() const = 0;
  virtual double px() const = 0;
  virtual double py() const = 0;
  virtual double skew() const = 0;

  // ------------ static methods ------------------

  // factory method for distortion models
  static std::shared_ptr<DistortionModel> make(
      const std::string& name,
      const Intrinsics& intrinsics,  // kx, ky, cu, cv
      const std::vector<double>& dist_coeffs);

  // factory method for pinhole camera without distortion
  static std::shared_ptr<DistortionModel> make_pinhole(double fx,
                                                       double fy,
                                                       double cu,
                                                       double cv);

  // check if it is a valid model
  static bool is_valid(const std::string& name, int num_dist_coeff = 4);
};

// define shared pointers
typedef std::shared_ptr<DistortionModel> DistortionModelPtr;
typedef std::shared_ptr<const DistortionModel> DistortionModelConstPtr;

}  // namespace VIO

#endif  // INCLUDE_KIMERA_VIO_DISTORTION_MODELS_DISTORTIONMODEL_H_
