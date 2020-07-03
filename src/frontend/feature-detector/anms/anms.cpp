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

#include "kimera-vio/frontend/feature-detector/anms/anms.h"

#include <stdlib.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "kimera-vio/frontend/feature-detector/anms/nanoflann.hpp"
#include "kimera-vio/frontend/feature-detector/anms/range-tree/ranget.h"

namespace anms {

std::vector<cv::KeyPoint> TopN(const std::vector<cv::KeyPoint>& keyPoints,
                               int numRetPoints) {
  if (numRetPoints > keyPoints.size()) {
    return keyPoints;
  }

  std::vector<cv::KeyPoint> kp;
  for (int i = 0; i < numRetPoints; i++)
    kp.push_back(keyPoints[i]);  // simply extracting numRetPoints keyPoints

  return kp;
}

std::vector<cv::KeyPoint> BrownANMS(const std::vector<cv::KeyPoint>& keyPoints,
                                    int numRetPoints) {
  if (numRetPoints > keyPoints.size()) {
    return keyPoints;
  }

  std::vector<pair<float, int> > results;
  results.push_back(make_pair(FLT_MAX, 0));
  for (unsigned int i = 1; i < keyPoints.size();
       ++i) {  // for every keyPoint we get the min distance to the previously
               // visited keyPoints
    float minDist = FLT_MAX;
    for (unsigned int j = 0; j < i; ++j) {
      float exp1 = (keyPoints[j].pt.x - keyPoints[i].pt.x);
      float exp2 = (keyPoints[j].pt.y - keyPoints[i].pt.y);
      float curDist = sqrt(exp1 * exp1 + exp2 * exp2);
      minDist = min(curDist, minDist);
    }
    results.push_back(make_pair(minDist, i));
  }
  sort(results.begin(), results.end(), sort_pred());  // sorting by radius
  std::vector<cv::KeyPoint> kp;
  for (int i = 0; i < numRetPoints; ++i)
    kp.push_back(
        keyPoints[results[i].second]);  // extracting numRetPoints keyPoints

  return kp;
}

std::vector<cv::KeyPoint> Sdc(const std::vector<cv::KeyPoint>& keyPoints,
                              int numRetPoints,
                              float tolerance,
                              int cols,
                              int rows) {
  double eps_var = 0.25;  // this parameter is chosen to be the most optimal in
                          // the original paper

  int low = 1;
  int high = cols;  // binary search range initialization
  int radius;
  int prevradius = -1;

  std::vector<int> ResultVec;
  bool complete = false;
  unsigned int K = numRetPoints;
  unsigned int Kmin = round(K - (K * tolerance));
  unsigned int Kmax = round(K + (K * tolerance));

  std::vector<int> result;
  result.reserve(keyPoints.size());
  while (!complete) {
    radius = low + (high - low) / 2;
    if (radius == prevradius ||
        low >
            high) {  // needed to reassure the same radius is not repeated again
      ResultVec = result;  // return the keypoints from the previous iteration
      break;
    }
    result.clear();
    double c = eps_var * radius / sqrt(2);  // initializing Grid
    int numCellCols = floor(cols / c);
    int numCellRows = floor(rows / c);
    std::vector<std::vector<bool> > coveredVec(
        numCellRows + 1, std::vector<bool>(numCellCols + 1, false));

    for (unsigned int i = 0; i < keyPoints.size(); ++i) {
      int row =
          floor(keyPoints[i].pt.y /
                c);  // get position of the cell current point is located at
      int col = floor(keyPoints[i].pt.x / c);
      if (coveredVec[row][col] == false) {  // if the cell is not covered
        result.push_back(i);
        int rowMin = ((row - floor(radius / c)) >= 0)
                         ? (row - floor(radius / c))
                         : 0;  // get range which current radius is covering
        int rowMax = ((row + floor(radius / c)) <= numCellRows)
                         ? (row + floor(radius / c))
                         : numCellRows;
        int colMin =
            ((col - floor(radius / c)) >= 0) ? (col - floor(radius / c)) : 0;
        int colMax = ((col + floor(radius / c)) <= numCellCols)
                         ? (col + floor(radius / c))
                         : numCellCols;
        for (int rowToCov = rowMin; rowToCov <= rowMax; ++rowToCov) {
          for (int colToCov = colMin; colToCov <= colMax; ++colToCov) {
            double dist = sqrt((rowToCov - row) * (rowToCov - row) +
                               (colToCov - col) * (colToCov - col));
            if (dist <= ((double)radius) / c)
              coveredVec[rowToCov][colToCov] =
                  true;  // check the distance to every cell
          }
        }
      }
    }
    if (result.size() >= Kmin && result.size() <= Kmax) {  // solution found
      ResultVec = result;
      complete = true;
    } else if (result.size() < Kmin)
      high = radius - 1;  // update binary search range
    else
      low = radius + 1;
  }
  // retrieve final keypoints
  std::vector<cv::KeyPoint> kp;
  for (unsigned int i = 0; i < ResultVec.size(); i++)
    kp.push_back(keyPoints[ResultVec[i]]);

  return kp;
}

std::vector<cv::KeyPoint> KdTree(const std::vector<cv::KeyPoint>& keyPoints,
                                 int numRetPoints,
                                 float tolerance,
                                 int cols,
                                 int rows) {
  // several temp expression variables to simplify solution equation
  int exp1 = rows + cols + 2 * numRetPoints;
  long long exp2 =
      ((long long)4 * cols + (long long)4 * numRetPoints +
       (long long)4 * rows * numRetPoints + (long long)rows * rows +
       (long long)cols * cols - (long long)2 * rows * cols +
       (long long)4 * rows * cols * numRetPoints);
  double exp3 = sqrt(exp2);
  double exp4 = numRetPoints - 1;

  double sol1 = -round((exp1 + exp3) / exp4);  // first solution
  double sol2 = -round((exp1 - exp3) / exp4);  // second solution

  int high =
      (sol1 > sol2)
          ? sol1
          : sol2;  // binary search range initialization with positive solution
  int low = floor(sqrt((double)keyPoints.size() / numRetPoints));

  PointCloud<int> cloud;  // creating k-d tree with keypoints
  generatePointCloud(cloud, keyPoints);
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<int, PointCloud<int> >,
      PointCloud<int>,
      2>
      my_kd_tree_t;
  my_kd_tree_t index(
      2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(25 /* max leaf */));
  index.buildIndex();

  bool complete = false;
  unsigned int K = numRetPoints;
  unsigned int Kmin = round(K - (K * tolerance));
  unsigned int Kmax = round(K + (K * tolerance));
  std::vector<int> ResultVec;
  int radius;
  int prevradius = -1;

  std::vector<int> result;
  result.reserve(keyPoints.size());
  while (!complete) {
    std::vector<bool> Included(keyPoints.size(), true);
    radius = low + (high - low) / 2;
    if (radius == prevradius ||
        low >
            high) {  // needed to reassure the same radius is not repeated again
      ResultVec = result;  // return the keypoints from the previous iteration
      break;
    }
    result.clear();

    for (unsigned int i = 0; i < keyPoints.size(); ++i) {
      if (Included[i] == true) {
        Included[i] = false;
        result.push_back(i);
        const int search_radius = static_cast<int>(radius * radius);
        std::vector<pair<size_t, int> > ret_matches;
        nanoflann::SearchParams params;
        const int query_pt[2] = {(int)keyPoints[i].pt.x,
                                 (int)keyPoints[i].pt.y};
        const size_t nMatches = index.radiusSearch(
            &query_pt[0], search_radius, ret_matches, params);

        for (size_t nmIdx = 0; nmIdx < nMatches; nmIdx++) {
          if (Included[ret_matches[nmIdx].first])
            Included[ret_matches[nmIdx].first] = false;
        }
      }
    }

    if (result.size() >= Kmin && result.size() <= Kmax) {  // solution found
      ResultVec = result;
      complete = true;
    } else if (result.size() < Kmin)
      high = radius - 1;  // update binary search range
    else
      low = radius + 1;

    prevradius = radius;
  }

  // retrieve final keypoints
  std::vector<cv::KeyPoint> kp;
  for (unsigned int i = 0; i < ResultVec.size(); i++)
    kp.push_back(keyPoints[ResultVec[i]]);

  return kp;
}

std::vector<cv::KeyPoint> RangeTree(const std::vector<cv::KeyPoint>& keyPoints,
                                    int numRetPoints,
                                    float tolerance,
                                    int cols,
                                    int rows) {
  // several temp expression variables to simplify solution equation
  int exp1 = rows + cols + 2 * numRetPoints;
  long long exp2 =
      ((long long)4 * cols + (long long)4 * numRetPoints +
       (long long)4 * rows * numRetPoints + (long long)rows * rows +
       (long long)cols * cols - (long long)2 * rows * cols +
       (long long)4 * rows * cols * numRetPoints);
  double exp3 = sqrt(exp2);
  double exp4 = numRetPoints - 1;

  double sol1 = -round((exp1 + exp3) / exp4);  // first solution
  double sol2 = -round((exp1 - exp3) / exp4);  // second solution

  int high =
      (sol1 > sol2)
          ? sol1
          : sol2;  // binary search range initialization with positive solution
  int low = floor(sqrt((double)keyPoints.size() / numRetPoints));

  rangetree<u16, u16> treeANMS(
      keyPoints.size(),
      keyPoints.size());  // creating range tree with keypoints
  for (unsigned int i = 0; i < keyPoints.size(); i++)
    treeANMS.add(keyPoints[i].pt.x, keyPoints[i].pt.y, (u16*)(intptr_t)i);
  treeANMS.finalize();

  bool complete = false;
  unsigned int K = numRetPoints;
  unsigned int Kmin = round(K - (K * tolerance));
  unsigned int Kmax = round(K + (K * tolerance));
  std::vector<int> ResultVec;
  int width;
  int prevwidth = -1;

  std::vector<int> result;
  result.reserve(keyPoints.size());
  while (!complete) {
    std::vector<bool> Included(keyPoints.size(), true);
    width = low + (high - low) / 2;
    if (width == prevwidth ||
        low >
            high) {  // needed to reassure the same width is not repeated again
      ResultVec = result;  // return the keypoints from the previous iteration
      break;
    }
    result.clear();

    for (unsigned int i = 0; i < keyPoints.size(); ++i) {
      if (Included[i] == true) {
        Included[i] = false;
        result.push_back(i);
        int minx = keyPoints[i].pt.x - width;
        int maxx = keyPoints[i].pt.x +
                   width;  // defining square boundaries around the point
        int miny = keyPoints[i].pt.y - width;
        int maxy = keyPoints[i].pt.y + width;
        if (minx < 0) minx = 0;
        if (miny < 0) miny = 0;

        std::vector<u16*>* he = treeANMS.search(minx, maxx, miny, maxy);
        for (unsigned int j = 0; j < he->size(); j++)
          if (Included[(u64)(*he)[j]]) Included[(u64)(*he)[j]] = false;
        delete he;
        he = NULL;
      }
    }
    if (result.size() >= Kmin && result.size() <= Kmax) {  // solution found
      ResultVec = result;
      complete = true;
    } else if (result.size() < Kmin)
      high = width - 1;  // update binary search range
    else
      low = width + 1;
    prevwidth = width;
  }
  // retrieve final keypoints
  std::vector<cv::KeyPoint> kp;
  for (unsigned int i = 0; i < ResultVec.size(); i++)
    kp.push_back(keyPoints[ResultVec[i]]);

  return kp;
}

std::vector<cv::KeyPoint> Ssc(const std::vector<cv::KeyPoint>& keyPoints,
                              int numRetPoints,
                              float tolerance,
                              int cols,
                              int rows) {
  // several temp expression variables to simplify solution equation
  int exp1 = rows + cols + 2 * numRetPoints;
  long long exp2 =
      ((long long)4 * cols + (long long)4 * numRetPoints +
       (long long)4 * rows * numRetPoints + (long long)rows * rows +
       (long long)cols * cols - (long long)2 * rows * cols +
       (long long)4 * rows * cols * numRetPoints);
  double exp3 = sqrt(exp2);
  double exp4 = numRetPoints - 1;

  double sol1 = -round((exp1 + exp3) / exp4);  // first solution
  double sol2 = -round((exp1 - exp3) / exp4);  // second solution

  int high =
      (sol1 > sol2)
          ? sol1
          : sol2;  // binary search range initialization with positive solution
  int low = floor(sqrt((double)keyPoints.size() / numRetPoints));

  int width;
  int prevWidth = -1;

  std::vector<int> ResultVec;
  bool complete = false;
  unsigned int K = numRetPoints;
  unsigned int Kmin = round(K - (K * tolerance));
  unsigned int Kmax = round(K + (K * tolerance));

  std::vector<int> result;
  result.reserve(keyPoints.size());
  while (!complete) {
    width = low + (high - low) / 2;
    if (width == prevWidth ||
        low >
            high) {  // needed to reassure the same radius is not repeated again
      ResultVec = result;  // return the keypoints from the previous iteration
      break;
    }
    result.clear();
    double c = width / 2;  // initializing Grid
    int numCellCols = floor(cols / c);
    int numCellRows = floor(rows / c);
    std::vector<std::vector<bool> > coveredVec(
        numCellRows + 1, std::vector<bool>(numCellCols + 1, false));

    for (unsigned int i = 0; i < keyPoints.size(); ++i) {
      int row =
          floor(keyPoints[i].pt.y /
                c);  // get position of the cell current point is located at
      int col = floor(keyPoints[i].pt.x / c);
      if (coveredVec[row][col] == false) {  // if the cell is not covered
        result.push_back(i);
        int rowMin = ((row - floor(width / c)) >= 0)
                         ? (row - floor(width / c))
                         : 0;  // get range which current radius is covering
        int rowMax = ((row + floor(width / c)) <= numCellRows)
                         ? (row + floor(width / c))
                         : numCellRows;
        int colMin =
            ((col - floor(width / c)) >= 0) ? (col - floor(width / c)) : 0;
        int colMax = ((col + floor(width / c)) <= numCellCols)
                         ? (col + floor(width / c))
                         : numCellCols;
        for (int rowToCov = rowMin; rowToCov <= rowMax; ++rowToCov) {
          for (int colToCov = colMin; colToCov <= colMax; ++colToCov) {
            if (!coveredVec[rowToCov][colToCov])
              coveredVec[rowToCov][colToCov] =
                  true;  // cover cells within the square bounding box with
                         // width w
          }
        }
      }
    }

    if (result.size() >= Kmin && result.size() <= Kmax) {  // solution found
      ResultVec = result;
      complete = true;
    } else if (result.size() < Kmin)
      high = width - 1;  // update binary search range
    else
      low = width + 1;
    prevWidth = width;
  }
  // retrieve final keypoints
  std::vector<cv::KeyPoint> kp;
  for (unsigned int i = 0; i < ResultVec.size(); i++)
    kp.push_back(keyPoints[ResultVec[i]]);

  return kp;
}

void VisualizeAll(cv::Mat Image,
                  std::vector<cv::KeyPoint> keyPoints,
                  string figureTitle) {
  cv::Mat resultImg;
  cv::drawKeypoints(
      Image, keyPoints, resultImg, cv::Scalar(94.0, 206.0, 165.0, 0.0));
  cv::namedWindow(figureTitle, cv::WINDOW_AUTOSIZE);
  cv::imshow(figureTitle, resultImg);
  return;
}

}  // namespace anms
