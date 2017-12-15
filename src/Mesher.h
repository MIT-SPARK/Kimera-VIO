/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher.h
 * @brief  Build and visualize 2D mesh from Frame
 * @author Luca Carlone, AJ Haeffner
 */

#ifndef Mesher_H_
#define Mesher_H_

#include "Frame.h"
#include <stdlib.h>
#include <opengv/point_cloud/methods.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace VIO {

class Mesher {


public:
  using LandmarkIdToMapPointId = std::unordered_map<LandmarkId, int>;

  cv::Mat mapPoints3d_; // set of (non-repeated) points = valid landmark positions
  LandmarkIdToMapPointId lmkIdToMapPointId_; // maps lmk id to corresponding 3D points
  int points3D_count_; // number of points

  // constructors
  Mesher(): points3D_count_(0) {}

  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  static std::vector<cv::Vec6f> CreateMesh2D(const Frame& frame){

    // sanity check
    if(frame.landmarks_.size() != frame.keypoints_.size())
      throw std::runtime_error("mesher: wrong dimension for the landmarks");

    // Rectangle to be used with Subdiv2D
    cv::Size size = frame.img_.size();
    cv::Rect2f rect(0, 0, size.width, size.height);

    // subdiv has the delaunay triangulation function
    cv::Subdiv2D subdiv(rect);

    // add points from Frame
    for(size_t i=0; i < frame.keypoints_.size(); i++){
      if(frame.landmarks_[i] != -1 && rect.contains(frame.keypoints_[i])){ // only for valid keypoints
        subdiv.insert(frame.keypoints_[i]);
      }
    }

    // do triangulation
    std::vector<cv::Vec6f> triangulation2D, triangulation2DwithExtraTriangles;

    // getTriangleList returns some spurious triangle with vertices outside image
    subdiv.getTriangleList(triangulation2DwithExtraTriangles);
    std::vector<cv::Point> pt(3);
    for(size_t i = 0; i < triangulation2DwithExtraTriangles.size(); i++)
        {
          cv::Vec6f t = triangulation2DwithExtraTriangles[i];
          pt[0] = cv::Point(t[0], t[1]);
          pt[1] = cv::Point(t[2], t[3]);
          pt[2] = cv::Point(t[4], t[5]);

          if(rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
            triangulation2D.push_back(t);
        }

    return triangulation2D;
  }
  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  cv::Mat createMesh3d_MapPointId(const Frame& frame) const{
    std::vector<cv::Vec6f> triangulation2D = Mesher::CreateMesh2D(frame);

    // Raw integer list of the form: (n,id1,id2,...,idn, n,id1,id2,...,idn, ...)
    // where n is the number of points in the poligon, and id is a zero-offset
    // index into an associated cloud.
    cv::Mat polygon(1,0,CV_32SC1);

    // Populate polygons with indices: note: we restrict to valid triangles in which
    // each landmark has a 3D point
    for(size_t i=0; i<triangulation2D.size();i++){

      cv::Vec6f t = triangulation2D[i];
      LandmarkId id_pt1 = frame.findLmkIdFromPixel(cv::Point2f(t[0], t[1]));
      LandmarkId id_pt2 = frame.findLmkIdFromPixel(cv::Point2f(t[2], t[3]));
      LandmarkId id_pt3 = frame.findLmkIdFromPixel(cv::Point2f(t[4], t[5]));

      // check if they are in the map (ie they have an associated 3D point)
      if ( lmkIdToMapPointId_.find(id_pt1) != lmkIdToMapPointId_.end() &&
          lmkIdToMapPointId_.find(id_pt2) != lmkIdToMapPointId_.end() &&
          lmkIdToMapPointId_.find(id_pt3) != lmkIdToMapPointId_.end() ){
        polygon.push_back(3);
        polygon.push_back(lmkIdToMapPointId_.at(id_pt1));
        polygon.push_back(lmkIdToMapPointId_.at(id_pt2));
        polygon.push_back(lmkIdToMapPointId_.at(id_pt3));

//        std::cout << lmkIdToMapPointId_[id_pt1] << " " << lmkIdToMapPointId_[id_pt2] << " "
//            << lmkIdToMapPointId_[id_pt3] << " - "
//            << id_pt1 << " " << id_pt2 << " " << id_pt3 << " (" <<
//            mapPoints3d_.rows-1 << ")"<< std::endl;
      }
    }
    int* data = polygon.ptr<int>();
    for(size_t i=0; i<polygon.rows;i++)
      std::cout << data[i] << std::endl;
    return polygon;
  }
  /* ----------------------------------------------------------------------------- */
  // Update map
  void updateMap3D(std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId){

    // update 3D points by adding new points or updating old reobserved ones
    for(size_t i=0; i<pointsWithId.size();i++) {

      LandmarkId lmk_id = pointsWithId.at(i).first;
      gtsam::Point3 point_i = pointsWithId.at(i).second;
      auto lm_it = lmkIdToMapPointId_.find(lmk_id);
      if (lm_it == lmkIdToMapPointId_.end()) // new landmark
      {
        cv::Point3f p(float(point_i.x()), float(point_i.y()), float(point_i.z()));
        mapPoints3d_.push_back(p);
        lmkIdToMapPointId_.insert(std::make_pair(lmk_id, points3D_count_));
        ++points3D_count_;
      }
      else // replace point for existing landmark
      {
        cv::Point3f* data = mapPoints3d_.ptr<cv::Point3f>();
        int row_id = (*lm_it).second;
        data[row_id].x = float ( point_i.x() );
        data[row_id].y = float ( point_i.y() );
        data[row_id].z = float ( point_i.z() );
      }
    }
  }
};
} // namespace VIO
#endif /* Mesher_H_ */


