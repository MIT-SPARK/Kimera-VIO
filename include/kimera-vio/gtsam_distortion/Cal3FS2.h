/* -*-c++-*--------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Bernd Pfrommer, based on code by
 * Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3FS2.h
 * @brief Calibration of a camera with equidistant (fisheye) distortion model
 * @date July 1st, 2018
 * @author bernd.pfrommer@gmail.com
 */
#ifndef INCLUDE_KIMERA_VIO_GTSAM_DISTORTION_CAL3FS2_H_
#define INCLUDE_KIMERA_VIO_GTSAM_DISTORTION_CAL3FS2_H_

#include <gtsam/geometry/Point2.h>
#include <string>

/**
 * @brief Calibration of a camera with fisheye (radtan) radial distortion
 * @addtogroup geometry
 * \nosubgrouping
 *
 * Uses same distortionmodel as OpenCV fisheye calibration
 * https://docs.opencv.org/trunk/db/d58/group__calib3d__fisheye.html
 * but does not allow for skew
 *
 * K = [ fx 0 u0 ; 0 fy v0 ; 0 0 1 ]
 * rr = x^2 + y^2
 * theta = atan(rr)
 * theta_d = theta*(1 + k1 * theta^2 + k2 * theta^4 + k3 * theta^6 + k4 *
 * theta^8) x' = theta_d/r * x y' = theta_d/r * y p  = K * p'
 */
class Cal3FS2 {
 protected:
  double fx_, fy_, u0_, v0_;  // focal length,  and principal point
  double k1_, k2_, k3_, k4_;  // radial distortion coefficients of radtan model

 public:
  enum { dimension = 8 };

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3FS2() : fx_(1), fy_(1), u0_(0), v0_(0), k1_(0), k2_(0), k3_(0), k4_(0) {}

  Cal3FS2(double fx,
          double fy,
          double u0,
          double v0,
          double k1,
          double k2,
          double k3,
          double k4)
      : fx_(fx),
        fy_(fy),
        u0_(u0),
        v0_(v0),
        k1_(k1),
        k2_(k2),
        k3_(k3),
        k4_(k4) {}

  virtual ~Cal3FS2() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit Cal3FS2(const gtsam::Vector& v);

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  virtual void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const Cal3FS2& K, double tol = 10e-9) const;

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

  /// First distortion coefficient
  inline double k1() const { return k1_; }

  /// Second distortion coefficient
  inline double k2() const { return k2_; }

  /// Third distortion coefficient
  inline double k3() const { return k3_; }

  /// Fourth distortion coefficient
  inline double k4() const { return k4_; }

  /// return calibration matrix -- not really applicable
  gtsam::Matrix3 K() const;

  /// return distortion parameter vector
  gtsam::Vector4 k() const { return gtsam::Vector4(k1_, k2_, k3_, k4_); }

  /// Return all parameters as a vector
  gtsam::Vector8 vector() const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3FS2 retract(const gtsam::Vector& d) const;

  /// Given a different calibration, calculate update to obtain it
  gtsam::Vector localCoordinates(const Cal3FS2& T2) const;

  /// Return dimensions of calibration manifold object
  virtual size_t dim() const { return dimension; }

  /**
   * convert intrinsic coordinates xy to (distorted) image coordinates uv
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*8 Jacobian wrpt Cal3FS2 parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in (distorted) image coordinates
   */
  gtsam::Point2 uncalibrate(
      const gtsam::Point2& p,
      gtsam::OptionalJacobian<2, 8> Dcal = boost::none,
      gtsam::OptionalJacobian<2, 2> Dp = boost::none) const;

  /// Convert (distorted) image coordinates uv to intrinsic coordinates xy
  gtsam::Point2 calibrate(const gtsam::Point2& p,
                          const double tol = 1e-5) const;

  /// Derivative of uncalibrate wrpt intrinsic coordinates
  gtsam::Matrix2 D2d_intrinsic(const gtsam::Point2& p) const;

  /// Derivative of uncalibrate wrpt the calibration parameters
  gtsam::Matrix28 D2d_calibration(const gtsam::Point2& p) const;

  /// @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  virtual boost::shared_ptr<Cal3FS2> clone() const {
    return boost::shared_ptr<Cal3FS2>(new Cal3FS2(*this));
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
    ar& BOOST_SERIALIZATION_NVP(k1_);
    ar& BOOST_SERIALIZATION_NVP(k2_);
    ar& BOOST_SERIALIZATION_NVP(k3_);
    ar& BOOST_SERIALIZATION_NVP(k4_);
  }

  /// @}
};
// This is really ugly, injecting stuff into gtsam's namespace!
namespace gtsam {
template <>
struct traits<Cal3FS2> : public gtsam::internal::Manifold<Cal3FS2> {};
template <>
struct traits<const Cal3FS2> : public gtsam::internal::Manifold<Cal3FS2> {};
}  // namespace gtsam

#endif  // INCLUDE_KIMERA_VIO_GTSAM_DISTORTION_CAL3FS2_H_
