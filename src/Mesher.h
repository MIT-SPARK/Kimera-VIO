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
  cv::Mat polygonsMesh_; // set of polygons
  LandmarkIdToMapPointId lmkIdToMapPointId_; // maps lmk id to corresponding 3D points
  int points3D_count_; // number of points

  // constructors
  Mesher(): polygonsMesh_(cv::Mat(0,1,CV_32SC1)), points3D_count_(0) { }

  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  cv::Mat createMesh2DTo3D_MapPointId(Frame& frame) const{

    // build 2D mesh, resticted to points with lmk!=-1
    frame.createMesh2D();
    std::vector<cv::Vec6f> triangulation2D = frame.triangulation2D_;

    // Raw integer list of the form: (n,id1,id2,...,idn, n,id1,id2,...,idn, ...)
    // where n is the number of points in the polygon, and id is a zero-offset
    // index into an associated cloud.
    cv::Mat polygon(0,1,CV_32SC1);

    // Populate polygons with indices:
    // note: we restrict to valid triangles in which each landmark has a 3D point
    for(size_t i=0; i<triangulation2D.size();i++){ // TODO: this is doing a lot of computation

      cv::Vec6f t = triangulation2D[i];
      LandmarkId id_pt1 = frame.findLmkIdFromPixel(cv::Point2f(t[0], t[1]));
      LandmarkId id_pt2 = frame.findLmkIdFromPixel(cv::Point2f(t[2], t[3]));
      LandmarkId id_pt3 = frame.findLmkIdFromPixel(cv::Point2f(t[4], t[5]));

      // check if they are in mapPoints3d_
      if ( lmkIdToMapPointId_.find(id_pt1) != lmkIdToMapPointId_.end() &&
          lmkIdToMapPointId_.find(id_pt2) != lmkIdToMapPointId_.end() &&
          lmkIdToMapPointId_.find(id_pt3) != lmkIdToMapPointId_.end() ){

        polygon.push_back(3); // add rows
        polygon.push_back(lmkIdToMapPointId_.at(id_pt1)); // row in mapPoints3d_
        polygon.push_back(lmkIdToMapPointId_.at(id_pt2)); // row in mapPoints3d_
        polygon.push_back(lmkIdToMapPointId_.at(id_pt3)); // row in mapPoints3d_
      }
    }
    return polygon;
  }
  /* ----------------------------------------------------------------------------- */
  // Update map: update structures keeping memory of the map before visualization
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
  /* ----------------------------------------------------------------------------- */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId, Frame& frame){
    // update 3D points (possibly replacing some points with new estimates)
    updateMap3D(pointsWithId);
    // concatenate mesh in the current image to existing mesh
    polygonsMesh_.push_back(createMesh2DTo3D_MapPointId(frame));
  }
  /* ----------------------------------------------------------------------------- */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId, Frame& frame){
    // update 3D points (possibly replacing some points with new estimates)
    updateMap3D(pointsWithId);
    // concatenate mesh in the current image to existing mesh
    polygonsMesh_.push_back(createMesh2DTo3D_MapPointId(frame));
  }
};
} // namespace VIO
#endif /* Mesher_H_ */


