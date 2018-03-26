/*
 * @file RegularPlane3Factor.cpp
 * @brief RegularPlane3 Factor class
 * @author Antoni Rosinol
 * @date February 20, 2018
 */

#pragma once

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Factor to implement error between a point landmark and a plane.
 */
class BasicRegularPlane3Factor: public NoiseModelFactor2<Point3, OrientedPlane3> {

protected:
  Key pointKey_;
  Key planeKey_;

  typedef NoiseModelFactor2<Point3, OrientedPlane3> Base;

public:

  /// Constructor
  BasicRegularPlane3Factor() {
  }
  virtual ~BasicRegularPlane3Factor() {}

  /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
  BasicRegularPlane3Factor(
      const Key& pointKey, const Key& planeKey, const SharedGaussian& noiseModel) :
      Base(noiseModel, pointKey, planeKey), pointKey_(pointKey), planeKey_(planeKey) {
  }

  /// print
  virtual void print(const std::string& s = "RegularPlane3Factor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// evaluateError
  /// Hpoint: jacobian of h wrt point landmark
  /// Hplane: jacobian of h wrt plane
  virtual Vector evaluateError(const Point3& point, const OrientedPlane3& plane,
                               boost::optional<Matrix&> H_point = boost::none,
                               boost::optional<Matrix&> H_plane = boost::none) const {
    Vector err(1);
    Unit3 plane_normal = plane.normal();
    double plane_distance = plane.distance();

    if (H_point) *H_point = plane_normal.unitVector().transpose();
    if (H_plane) {
      Matrix43 H_plane_retract;
      // Jacobian of plane retraction when v = Vector3::Zero(), to speed-up
      // computations.
      H_plane_retract << plane_normal.basis(), Vector3::Zero(), 0, 0, 1;
      Vector4 p;
      p << point.vector(), -1;
      *H_plane =  p.transpose() * H_plane_retract;
    }

    err << point.dot(plane_normal.unitVector()) - plane_distance;
    return (err);
  }
};

class RegularPlane3Factor: public NoiseModelFactor4<Point3,
                                                    Point3,
                                                    Point3,
                                                    OrientedPlane3> {

protected:
  Key point1Key_;
  Key point2Key_;
  Key point3Key_;
  Key planeKey_;

  typedef NoiseModelFactor4<Point3, Point3, Point3, OrientedPlane3> Base;

public:

  /// Constructor
  RegularPlane3Factor() {
  }
  virtual ~RegularPlane3Factor() {}

  /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
  RegularPlane3Factor(
      const Key& point1,
      const Key& point2,
      const Key& point3,
      const Key& plane, const SharedGaussian& noiseModel) :
      Base(noiseModel, point1, point2, point3, plane),
      point1Key_(point1), point2Key_(point2), point3Key_(point3),
      planeKey_(plane) {
      std::cout << "Noise model covariance " << noiseModel->covariance() << std::endl;
  }

  /// print
  virtual void print(const std::string& s = "RegularPlane3Factor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// evaluateError
  /// Hpoint: jacobian of h wrt point landmark
  /// Hplane: jacobian of h wrt plane
  virtual Vector evaluateError(const Point3& point1,
                               const Point3& point2,
                               const Point3& point3,
                               const OrientedPlane3& plane,
                               boost::optional<Matrix&> Hpoint1 = boost::none,
                               boost::optional<Matrix&> Hpoint2 = boost::none,
                               boost::optional<Matrix&> Hpoint3 = boost::none,
                               boost::optional<Matrix&> Hplane = boost::none) const {
    Vector err(1);
    // n * l - d
    Vector3 plane_normal = plane.normal().unitVector();
    double plane_distance = plane.distance();
    //double dot_prod1 = plane_normal.dot(point1);
    //double dot_prod2 = plane_normal.dot(point2);
    //double dot_prod3 = plane_normal.dot(point3);
    double dot_prod1 = plane_normal(0)*point1.x() + plane_normal(1)*point1.y() + plane_normal(2)*point1.z();
    double dot_prod2 = plane_normal(0)*point2.x() + plane_normal(1)*point2.y() + plane_normal(2)*point2.z();
    double dot_prod3 = plane_normal(0)*point3.x() + plane_normal(1)*point3.y() + plane_normal(2)*point3.z();
    double error1 = dot_prod1 - plane_distance;
    double error2 = dot_prod2 - plane_distance;
    double error3 = dot_prod3 - plane_distance;

    if (Hpoint1) {
        (*Hpoint1) = (Matrix(1,3) << 2*error1*plane_normal(0),
                                     2*error1*plane_normal(1),
                                     2*error1*plane_normal(2)).finished();
        std::cout << "H1 requested\n";
        std::cout << "H1 :" << *Hpoint1 << std::endl;
    }
    if (Hpoint2) {
        (*Hpoint2) = (Matrix(1,3) << 2*error2*plane_normal(0),
                                     2*error2*plane_normal(1),
                                     2*error2*plane_normal(2)).finished();
        std::cout << "H2 requested\n";
        std::cout << "H2 :" << *Hpoint2 << std::endl;
    }
    if (Hpoint3) {
        (*Hpoint3) = (Matrix(1,3) << 2*error3*plane_normal(0),
                                     2*error3*plane_normal(1),
                                     2*error3*plane_normal(2)).finished();
        std::cout << "H3 requested\n";
        std::cout << "H3 :" << *Hpoint3 << std::endl;
    }
    Point3 sum = point1 + point2 + point3;
    if (Hplane) {
        (*Hplane) = (Matrix(1,4) <<
    2*(dot_prod1 * point1.x() + dot_prod2 * point2.x() + dot_prod3 * point3.x()
                     - plane_distance * (sum.x())),
    2*(dot_prod1 * point1.y() + dot_prod2 * point2.y() + dot_prod3 * point3.y()
                     - plane_distance * (sum.y())),
    2*(dot_prod1 * point1.z() + dot_prod2 * point2.z() + dot_prod3 * point3.z()
                     - plane_distance * (sum.z())),
    -2*(plane_normal.dot(sum) - 3 * plane_distance) // WRONG DOT PRODUCT! use sum.dot(plane_normal)
                    ).finished();
        std::cout << "Hplane requested\n";
        std::cout << "Hplane :" << *Hplane << std::endl;
    }
    Vector err1(1), err2(1), err3(1);
    err1 << std::pow(dot_prod1 - plane_distance, 2);
    err2 << std::pow(dot_prod2 - plane_distance, 2);
    err3 << std::pow(dot_prod3 - plane_distance, 2);
    std::cout << "\033[1mError plane to point 1 is: " << err1 << "\033[0m\n";
    std::cout << "\033[1mError plane to point 2 is: " << err2 << "\033[0m\n";
    std::cout << "\033[1mError plane to point 3 is: " << err3 << "\033[0m\n";
    err << err1 + err2 +err3;
    //err << std::sqrt(plane_normal.dot(point1) - plane_distance) +
    //       std::sqrt(plane_normal.dot(point2) - plane_distance) +
    //       std::sqrt(plane_normal.dot(point3) - plane_distance)
    //       ;
    std::cout << "\033[1mError is: " << err << "\033[0m\n";
    return (err);
  }
};

} // gtsam

