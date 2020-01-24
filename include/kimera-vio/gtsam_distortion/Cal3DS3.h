/* -*-c++-*--------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Bernd Pfrommer, based on code by
 * Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS3.h
 * @brief Calibration of a camera with radtan p1,p2, k1...k6 distortion model
 * @date November 15th, 2018
 * @author bernd.pfrommer@gmail.com
 */

#ifndef INCLUDE_KIMERA_VIO_GTSAM_DISTORTION_CAL3DS3_H_
#define INCLUDE_KIMERA_VIO_GTSAM_DISTORTION_CAL3DS3_H_

#include <gtsam/geometry/Point2.h>
#include <string>

/**
 * @brief Calibration of a camera with fisheye (radtan) radial distortion
 * @addtogroup geometry
 * \nosubgrouping
 *
 * Uses standard OpenCV radtan distortion model [aka plumb_bob]
 *
 * K = [ fx 0 u0 ; 0 fy v0 ; 0 0 1 ]
 * rr = x^2 + y^2
 * theta = atan(rr)
 * f_rad = (1 + k_[0] * r^2 + k_[1] * r^4 + k_[2] * r^6)/
 *         (1 + k_[3] * r^2 + k_[4] * r^4 + k_[5] * r^6)/
 * x' = x * f_rad + 2*p_1*x*y           +   p_2(r^2 + 2*x^2)
 * y' = y * f_rad +   p_1*(r^2 + 2y^2)  + 2*p_2*x*y
 * y' = theta_d/r * y
 * p  = K * p'
 */
class Cal3DS3 {
 public:
  enum { dimension = 12 };
  enum { num_coeff = 6 };

 protected:
  double fx_, fy_, u0_, v0_;  // focal length,  and principal point
  double p1_, p2_,
      k_[num_coeff];  // radial distortion coefficients of radtan model

 public:
  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3DS3() : fx_(1), fy_(1), u0_(0), v0_(0), p1_(0), p2_(0) {
    for (int i = 0; i < num_coeff; i++) {
      k_[i] = 0;
    }
  }

  Cal3DS3(double fx,
          double fy,
          double u0,
          double v0,
          double p1,
          double p2,
          const double* k)
      : fx_(fx), fy_(fy), u0_(u0), v0_(v0), p1_(p1), p2_(p2) {
    for (int i = 0; i < num_coeff; i++) {
      k_[i] = k[i];
    }
  }

  virtual ~Cal3DS3() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit Cal3DS3(const gtsam::Vector& v);

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  virtual void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const Cal3DS3& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// focal length x
  inline double fx() const { return fx_; }

  /// focal length x
  inline double fy() const { return fy_; }

  /// image center in x
  inline double px() const { return u0_; }

  /// image center in y
  inline double py() const { return v0_; }

  /// distortion p1
  inline double p1() const { return p1_; }

  /// image center in y
  inline double p2() const { return p2_; }

  /// distortion coefficients
  inline double k1() const { return k_[0]; }
  inline double k2() const { return k_[1]; }
  inline double k3() const { return k_[2]; }
  inline double k4() const { return k_[3]; }
  inline double k5() const { return k_[4]; }
  inline double k6() const { return k_[5]; }

  /// return calibration matrix -- not really applicable
  gtsam::Matrix3 K() const;

  /// return distortion parameter vector
  gtsam::Vector6 k() const {
    gtsam::Vector6 v;
    for (int i = 0; i < 6; i++) {
      v(i) = k_[i];
    }
    return (v);
  }

  /// Return all parameters as a vector
  Eigen::Matrix<double, 12, 1> vector() const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3DS3 retract(const gtsam::Vector& d) const;

  /// Given a different calibration, calculate update to obtain it
  gtsam::Vector localCoordinates(const Cal3DS3& T2) const;

  /// Return dimensions of calibration manifold object
  virtual size_t dim() const { return dimension; }

  /**
   * convert intrinsic coordinates xy to (distorted) image coordinates uv
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*12 Jacobian wrpt Cal3DS3 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in (distorted) image coordinates
   */
  gtsam::Point2 uncalibrate(
      const gtsam::Point2& p,
      gtsam::OptionalJacobian<2, 12> Dcal = boost::none,
      gtsam::OptionalJacobian<2, 2> Dp = boost::none) const;

  /// Convert (distorted) image coordinates uv to intrinsic coordinates xy
  gtsam::Point2 calibrate(const gtsam::Point2& p,
                          const double tol = 1e-5) const;

  /// Derivative of uncalibrate wrpt intrinsic coordinates
  gtsam::Matrix2 D2d_intrinsic(const gtsam::Point2& p) const;

  /// Derivative of uncalibrate wrpt the calibration parameters
  Eigen::Matrix<double, 2, 12> D2d_calibration(const gtsam::Point2& p) const;

  /// @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  virtual boost::shared_ptr<Cal3DS3> clone() const {
    return boost::shared_ptr<Cal3DS3>(new Cal3DS3(*this));
  }

  /// @}

 private:
  gtsam::Point2 uncalibrateNoIntrinsics(const gtsam::Point2& p) const;
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar,  // NOLINT
                 const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(fx_);
    ar& BOOST_SERIALIZATION_NVP(fy_);
    ar& BOOST_SERIALIZATION_NVP(u0_);
    ar& BOOST_SERIALIZATION_NVP(v0_);
    ar& BOOST_SERIALIZATION_NVP(p1_);
    ar& BOOST_SERIALIZATION_NVP(p2_);
    ar& BOOST_SERIALIZATION_NVP(k_[0]);
    ar& BOOST_SERIALIZATION_NVP(k_[1]);
    ar& BOOST_SERIALIZATION_NVP(k_[2]);
    ar& BOOST_SERIALIZATION_NVP(k_[3]);
    ar& BOOST_SERIALIZATION_NVP(k_[4]);
    ar& BOOST_SERIALIZATION_NVP(k_[5]);
  }

  /// @}
};
// This is really ugly, injecting stuff into gtsam's namespace!
namespace gtsam {
template <>
struct traits<Cal3DS3> : public gtsam::internal::Manifold<Cal3DS3> {};
template <>
struct traits<const Cal3DS3> : public gtsam::internal::Manifold<Cal3DS3> {};
}  // namespace gtsam

#endif  // INCLUDE_KIMERA_VIO_GTSAM_DISTORTION_CAL3DS3_H_
