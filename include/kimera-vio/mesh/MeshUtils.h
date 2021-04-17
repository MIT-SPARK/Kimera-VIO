/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MeshUtils.h
 * @brief  Utilities for mesh computations
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Core>

#include <glog/logging.h>

#include <opencv2/viz.hpp>

#include "kimera-vio/common/vio_types.h"

namespace VIO {

inline bool isValidPoint(const cv::Point3f& pt,
                         const float& missing_z = 10000.0,
                         const float& min_z = 1.5,
                         const float& max_z = 5.0) {
  // Check both for disparities explicitly marked as invalid (where OpenCV maps
  // pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt.z < missing_z && !std::isinf(pt.z) && !std::isnan(pt.z) &&
         pt.z < max_z && pt.z > min_z;
}

//! Barycenter Coordinates type
using BaryCoord = float;
//! Vector3f to support vectorized math
using Vec3f = Eigen::Vector3f;

/**
 * @brief edgeFunction As in Juan Pineda in 1988 and a paper called "A Parallel
 * Algorithm for Polygon Rasterization". Implementation from
 * https://www.scratchapixel.com
 * @param a, b, c 2D points
 * @return ORDER MATTERS: returns positive number if left of line, negative otw.
 * Zero if the point is exactly on the line.
 * Equivalent to the magnitude of the cross products between the
 * vector (b-a) and (c-a).
 */
inline float edgeFunction(const KeypointCV& a,
                          const KeypointCV& b,
                          const KeypointCV& c) {
  // return (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);
  return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

/**
 * @brief checkPointInsideTriangle
 * https://www.scratchapixel.com
 * @param a, b, c Vertices of the triangle in 2D
 * @param p Vertex to query
 * @return True if inside, false otw.
 */
inline bool checkPointInsideTriangle(const KeypointCV& a,
                                     const KeypointCV& b,
                                     const KeypointCV& c,
                                     const KeypointCV& p) {
  bool inside = true;
  // TODO(Toni): this is different than the check inside tri below!
  inside &= edgeFunction(a, b, p) >= 0;
  inside &= edgeFunction(b, c, p) >= 0;
  inside &= edgeFunction(c, a, p) >= 0;
  return inside;
}

/**
 * @brief barycentricCoordinates
 * https://www.scratchapixel.com
 * @param v0, v1, v2 Triangle coordinates in 2D image.
 * @param p Query Point in 2D image.
 * @param w0, w1, w2 Barycentric coordinates.
 * @return True if Query point is inside triangle, false otherwise.
 * If the query point is at a vertex it returns false!
 */
inline bool barycentricCoordinates(const KeypointCV& v0,
                                   const KeypointCV& v1,
                                   const KeypointCV& v2,
                                   const KeypointCV& p,
                                   BaryCoord* w0,
                                   BaryCoord* w1,
                                   BaryCoord* w2) {
  CHECK_NOTNULL(w0);
  CHECK_NOTNULL(w1);
  CHECK_NOTNULL(w2);

  // area of the triangle multiplied by 2
  float area = edgeFunction(v0, v1, v2);
  // signed area of the triangle v1v2p multiplied by 2
  *w0 = edgeFunction(v1, v2, p);
  // signed area of the triangle v2v0p multiplied by 2
  *w1 = edgeFunction(v2, v0, p);
  // signed area of the triangle v0v1p multiplied by 2
  *w2 = edgeFunction(v0, v1, p);

  bool has_neg = (*w0 < 0) || (*w1 < 0) || (*w2 < 0);
  bool has_pos = (*w0 > 0) || (*w1 > 0) || (*w2 > 0);
  // if point p is inside triangles defined by vertices v0, v1, v2
  if (!(has_neg && has_pos)) {
    // barycentric coordinates are the areas of the sub-triangles divided by the
    // area of the main triangle
    *w0 /= area;
    *w1 /= area;
    *w2 /= area;
    return true;
  } else {
    return false;
  }
}

/**
 * @brief rayTriangleIntersect routine using
 * Möller–Trumbore ray-triangle intersection algorithm.
 * See https://www.scratchapixel.com for more info.
 * @param orig Origin of ray
 * @param dir Direction of ray, normalized!
 * @param v0, v1, v2 Vertices of triangle
 * @param t
 * @param u, v Barycentric coordinates of the intersection point
 * @param culling Set to true if you have double-faced triangles.
 * @return
 */
inline bool rayTriangleIntersect(const Vec3f& orig,
                                 const Vec3f& dir,
                                 const Vec3f& v0,
                                 const Vec3f& v1,
                                 const Vec3f& v2,
                                 float& t,
                                 float& u,
                                 float& v,
                                 const bool& culling = false) {
  static constexpr float kEpsilon = 1e-8;

  Vec3f v0v1 = v1 - v0;
  Vec3f v0v2 = v2 - v0;
  Vec3f pvec = dir.cross(v0v2);
  float det = v0v1.dot(pvec);
  if (culling) {
    // if the determinant is negative the triangle is backfacing
    // if the determinant is close to 0, the ray misses the triangle
    if (det < kEpsilon) return false;
  } else {
    // ray and triangle are parallel if det is close to 0
    if (std::fabs(det) < kEpsilon) return false;
  }
  float inv_det = 1.0 / det;

  Vec3f tvec = orig - v0;
  u = tvec.dot(pvec) * inv_det;
  if (u < 0.0 || u > 1.0) return false;

  Vec3f qvec = tvec.cross(v0v1);
  v = dir.dot(qvec) * inv_det;
  if (v < 0.0 || u + v > 1.0) return false;

  t = v0v2.dot(qvec) * inv_det;

  return true;
}

/**
 * Maps an input h from a value between 0.0 and 1.0 into a rainbow. Copied from
 * OctomapProvider in octomap. Copied from voxblox itself.
 */
inline cv::viz::Color rainbowColorMap(double h) {
  cv::viz::Color color;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color = cv::Scalar(255 * m, 255 * n, 255 * v, 255);
      break;
    case 1:
      color = cv::Scalar(255 * m, 255 * v, 255 * n, 255);
      break;
    case 2:
      color = cv::Scalar(255 * n, 255 * v, 255 * m, 255);
      break;
    case 3:
      color = cv::Scalar(255 * v, 255 * n, 255 * m, 255);
      break;
    case 4:
      color = cv::Scalar(255 * v, 255 * m, 255 * n, 255);
      break;
    case 5:
      color = cv::Scalar(255 * n, 255 * m, 255 * v, 255);
      break;
    default:
      color = cv::Scalar(127, 127, 255, 255);
      break;
  }

  return color;
}

/// Maps an input h from a value between 0.0 and 1.0 into a grayscale color.
inline cv::viz::Color grayColorMap(double h) {
  auto x = round(h * 255);
  return cv::Scalar(x, x, x);
}

inline cv::viz::Color randomColor() {
  return cv::Scalar(rand() % 256, rand() % 256, rand() % 256, 255);
}

template <class T>
T min3(const T& a, const T& b, const T& c) {
  return std::min(a, std::min(b, c));
}
template <class T>
T max3(const T& a, const T& b, const T& c) {
  return std::max(a, std::max(b, c));
}

}  // namespace VIO
