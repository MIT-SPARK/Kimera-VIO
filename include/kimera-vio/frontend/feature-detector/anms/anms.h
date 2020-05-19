/**
MIT License

Copyright (c) 2018 Oleksandr Bailo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include <stdlib.h>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "kimera-vio/frontend/feature-detector/anms/range-tree/ranget.h"

namespace anms {

using namespace std;

std::vector<cv::KeyPoint> TopN(const std::vector<cv::KeyPoint>& keyPoints,
                               int numRetPoints);

struct sort_pred {
  bool operator()(const pair<float, int>& left, const pair<float, int>& right) {
    return left.first > right.first;
  }
};

std::vector<cv::KeyPoint> BrownANMS(const std::vector<cv::KeyPoint>& keyPoints,
                                    int numRetPoints);

std::vector<cv::KeyPoint> Sdc(const std::vector<cv::KeyPoint>& keyPoints,
                              int numRetPoints,
                              float tolerance,
                              int cols,
                              int rows);

/*kdtree algorithm*/
template <typename T>
struct PointCloud {
  struct Point {
    T x, y;
  };
  std::vector<Point> pts;
  inline size_t kdtree_get_point_count() const {
    return pts.size();
  }  // Must return the number of data points
  // Returns the distance between the std::vector "p1[0:size-1]" and the data
  // point with index "idx_p2" stored in the class:
  inline T kdtree_distance(const T* p1,
                           const size_t idx_p2,
                           size_t /*size*/) const {
    const T d0 = p1[0] - pts[idx_p2].x;
    const T d1 = p1[1] - pts[idx_p2].y;
    return d0 * d0 + d1 * d1;
  }
  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline T kdtree_get_pt(const size_t idx, int dim) const {
    if (dim == 0)
      return pts[idx].x;
    else if (dim == 1)
      return pts[idx].y;
    return 0;
  }
  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;
  }
};

template <typename T>
inline void generatePointCloud(PointCloud<T>& point,
                               std::vector<cv::KeyPoint> keyPoints) {
  point.pts.resize(keyPoints.size());
  for (size_t i = 0; i < keyPoints.size(); i++) {
    point.pts[i].x = keyPoints[i].pt.x;
    point.pts[i].y = keyPoints[i].pt.y;
  }
}

std::vector<cv::KeyPoint> KdTree(const std::vector<cv::KeyPoint>& keyPoints,
                                 int numRetPoints,
                                 float tolerance,
                                 int cols,
                                 int rows);

std::vector<cv::KeyPoint> RangeTree(const std::vector<cv::KeyPoint>& keyPoints,
                                    int numRetPoints,
                                    float tolerance,
                                    int cols,
                                    int rows);

std::vector<cv::KeyPoint> Ssc(const std::vector<cv::KeyPoint>& keyPoints,
                              int numRetPoints,
                              float tolerance,
                              int cols,
                              int rows);

void VisualizeAll(cv::Mat Image,
                  std::vector<cv::KeyPoint> keyPoints,
                  string figureTitle);

}  // namespace anms
