/* -*-c++-*--------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Bernd Pfrommer, based on code by
 * Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3FS2.cpp
 * @brief Calibration of a camera with equidistant (fisheye) distortion model
 * @date July 1st, 2018
 * @author bernd.pfrommer@gmail.com
 */

#include "kimera-vio/gtsam_distortion/Cal3FS2.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <string>

/* ************************************************************************* */
Cal3FS2::Cal3FS2(const gtsam::Vector& v)
    : fx_(v[0]),
      fy_(v[1]),
      u0_(v[2]),
      v0_(v[3]),
      k1_(v[4]),
      k2_(v[5]),
      k3_(v[6]),
      k4_(v[7]) {}

/* ************************************************************************* */
gtsam::Matrix3 Cal3FS2::K() const {
  gtsam::Matrix3 K;
  K << fx_, 0, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
  return K;
}

/* ************************************************************************* */
gtsam::Vector8 Cal3FS2::vector() const {
  gtsam::Vector8 v;
  v << fx_, fy_, u0_, v0_, k1_, k2_, k3_, k4_;
  return v;
}

/* ************************************************************************* */
void Cal3FS2::print(const std::string& s_) const {
  gtsam::print((gtsam::Matrix)K(), s_ + ".K");
  gtsam::print(gtsam::Vector(k()), s_ + ".k");
}

/* ************************************************************************* */
bool Cal3FS2::equals(const Cal3FS2& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol ||
      fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol ||
      fabs(k1_ - K.k1_) > tol || fabs(k2_ - K.k2_) > tol ||
      fabs(k3_ - K.k3_) > tol || fabs(k4_ - K.k4_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
static gtsam::Matrix28 D2dcalibration(double x,
                                      double y,
                                      double xp,
                                      double yp,
                                      double x_r,
                                      double y_r,
                                      double theta3,
                                      double theta5,
                                      double theta7,
                                      double theta9,
                                      const gtsam::Matrix2& DK) {
  gtsam::Matrix24 DR1;
  //     fx   fy   cx   cy
  DR1 << xp, 0.0, 1.0, 0.0,  // du/d
      0.0, yp, 0.0, 1.0;     // dv/d

  gtsam::Matrix24 DR2;
  //        k1              k2            k3          k4
  DR2 << x_r * theta3, x_r * theta5, x_r * theta7, x_r * theta9,  // du/d
      y_r * theta3, y_r * theta5, y_r * theta7, y_r * theta9;     // dv/d

  gtsam::Matrix28 D;
  D << DR1, DK * DR2;
  return D;
}

/* ************************************************************************* */
static gtsam::Matrix2 D2dintrinsic(double r,
                                   double r2,
                                   double x_r,
                                   double y_r,
                                   double theta_d,
                                   double theta,
                                   double theta3,
                                   double theta5,
                                   double theta7,
                                   double k1,
                                   double k2,
                                   double k3,
                                   double k4,
                                   const gtsam::Matrix2& DK) {
  const double theta_d_r = (r < 1e-9) ? 1.0 : (theta_d / r);
  const double dtheta_d_dtheta =
      1 + theta * (k1 * 3 * theta + k2 * 5 * theta3 + k3 * 7 * theta5 +
                   k4 * 9 * theta7);
  const double dthetad_r_dr = dtheta_d_dtheta / (1 + r2) - theta_d_r;

  gtsam::Matrix2 DR;

  DR <<  //            dx                      dy
      theta_d_r + x_r * x_r * dthetad_r_dr,
      x_r * y_r * dthetad_r_dr,                                        // du
      x_r * y_r * dthetad_r_dr, theta_d_r + y_r * y_r * dthetad_r_dr;  // dv

  return DK * DR;
}

struct GeometricParams {
  GeometricParams(double xa,
                  double ya,
                  double k1,
                  double k2,
                  double k3,
                  double k4,
                  double tol = 1e-9)
      : x(xa), y(ya) {
    const double xx = x * x, yy = y * y;
    rr = xx + yy;
    r = std::sqrt(rr);
    theta = std::atan(r);
    theta2 = theta * theta;
    theta3 = theta * theta2;
    theta5 = theta3 * theta2;
    theta7 = theta5 * theta2;
    theta9 = theta7 * theta2;
    theta_d = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
    x_r = (r < tol) ? 1.0 : x / r;
    y_r = (r < tol) ? 1.0 : y / r;
    xp = theta_d * x_r;
    yp = theta_d * y_r;
  }
  double r, rr;
  double x, y;
  double theta, theta2, theta3, theta5, theta7, theta9;
  double theta_d;
  double x_r, y_r;
  double xp, yp;
};

gtsam::Point2 Cal3FS2::uncalibrateNoIntrinsics(const gtsam::Point2& p) const {
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  return gtsam::Point2(gp.xp, gp.yp);
}

/* ************************************************************************* */
gtsam::Point2 Cal3FS2::uncalibrate(const gtsam::Point2& p,
                                   gtsam::OptionalJacobian<2, 8> H1,
                                   gtsam::OptionalJacobian<2, 2> H2) const {
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  gtsam::Matrix2 DK;
  if (H1 || H2) DK << fx_, 0.0, 0.0, fy_;

  // Derivative for calibration
  if (H1)
    *H1 = D2dcalibration(gp.x,
                         gp.y,
                         gp.xp,
                         gp.yp,
                         gp.x_r,
                         gp.y_r,
                         gp.theta3,
                         gp.theta5,
                         gp.theta7,
                         gp.theta9,
                         DK);

  // Derivative for points
  if (H2)
    *H2 = D2dintrinsic(gp.r,
                       gp.rr,
                       gp.x_r,
                       gp.y_r,
                       gp.theta_d,
                       gp.theta,
                       gp.theta3,
                       gp.theta5,
                       gp.theta7,
                       k1_,
                       k2_,
                       k3_,
                       k4_,
                       DK);

  // Regular uncalibrate after distortion
  return gtsam::Point2(fx_ * gp.xp + u0_, fy_ * gp.yp + v0_);
}

/* ************************************************************************* */
gtsam::Point2 Cal3FS2::calibrate(const gtsam::Point2& pi,
                                 const double tol) const {
  // map from u,v -> x, y

  const gtsam::Point2 invKPi((1 / fx_) * (pi.x() - u0_),
                             (1 / fy_) * (pi.y() - v0_));

  // initialize by ignoring the distortion at all, might be problematic for
  // pixels around boundary
  gtsam::Point2 pn = invKPi;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    const gtsam::Point2 xpyp = uncalibrateNoIntrinsics(pn);
    if (distance2(xpyp, invKPi) <= tol) break;
    pn = pn + invKPi - uncalibrateNoIntrinsics(pn);
  }
  if (iteration >= maxIterations)
    throw std::runtime_error(
        "Cal3FS2::calibrate fails to converge. need a better initialization");

  return pn;
}

/* ************************************************************************* */
gtsam::Matrix2 Cal3FS2::D2d_intrinsic(const gtsam::Point2& p) const {
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  gtsam::Matrix2 DK;
  DK << fx_, 0.0, 0.0, fy_;
  return (D2dintrinsic(gp.r,
                       gp.rr,
                       gp.x_r,
                       gp.y_r,
                       gp.theta_d,
                       gp.theta,
                       gp.theta3,
                       gp.theta5,
                       gp.theta7,
                       k1_,
                       k2_,
                       k3_,
                       k4_,
                       DK));
}

/* ************************************************************************* */
gtsam::Matrix28 Cal3FS2::D2d_calibration(const gtsam::Point2& p) const {
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  gtsam::Matrix2 DK;
  DK << fx_, 0.0, 0.0, fy_;

  // Derivative for calibration
  return (D2dcalibration(gp.x,
                         gp.y,
                         gp.xp,
                         gp.yp,
                         gp.x_r,
                         gp.y_r,
                         gp.theta3,
                         gp.theta5,
                         gp.theta7,
                         gp.theta9,
                         DK));
}
