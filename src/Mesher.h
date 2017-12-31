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

#include "StereoFrame.h"
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
  using LandmarkIdToMapPointId = std::unordered_map<LandmarkId, int>; // map a lmk id to a row in mapPoints3d_
  using KeypointToMapPointId = std::vector< std::pair<KeypointCV,int>>; // map a keypoint (without lmk id) to a row in mapPoints3d_

  cv::Mat mapPoints3d_; // set of (non-repeated) points = valid landmark positions
  cv::Mat polygonsMesh_; // set of polygons
  LandmarkIdToMapPointId lmkIdToMapPointId_; // maps lmk id to corresponding 3D points
  int points3D_count_; // number of points
  KeypointToMapPointId keypointToMapPointId_;

// constructors
  Mesher(): polygonsMesh_(cv::Mat(0,1,CV_32SC1)), points3D_count_(0) { }

  /* ----------------------------------------------------------------------------- */
  // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenSmallestAndLargestSidesquared(const int rowId_pt1,const int rowId_pt2,const int rowId_pt3,
      boost::optional<double &> d12_out = boost::none,
      boost::optional<double &> d23_out = boost::none,
      boost::optional<double &> d31_out = boost::none) const{

    // get 3D points
    cv::Point3f p1 = mapPoints3d_.at<cv::Point3f>(rowId_pt1);
    cv::Point3f p2 = mapPoints3d_.at<cv::Point3f>(rowId_pt2);
    cv::Point3f p3 = mapPoints3d_.at<cv::Point3f>(rowId_pt3);

    // measure sides:
    double d12 = double( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z) );
    double d23 = double( (p2.x - p3.x)*(p2.x - p3.x) + (p2.y - p3.y)*(p2.y - p3.y) + (p2.z - p3.z)*(p2.z - p3.z) );
    double d31 = double( (p3.x - p1.x)*(p3.x - p1.x) + (p3.y - p1.y)*(p3.y - p1.y) + (p3.z - p1.z)*(p3.z - p1.z) );

    if(d12_out && d23_out && d31_out){ // return distances, mainly for debug
      *d12_out = d12;
      *d23_out = d23;
      *d31_out = d31;
    }
    // compute and return ratio
    double ratio = std::min(d12,std::min(d23,d31)) / std::max(d12,std::max(d23,d31));
    return ratio;
  }
  /* ----------------------------------------------------------------------------- */
    // for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1), mapPoints3d_.at(rowId_pt2),
  // mapPoints3d_.at(rowId_pt3), compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenTangentialAndRadialDisplacement(
      const int rowId_pt1,const int rowId_pt2,const int rowId_pt3, gtsam::Pose3 leftCameraPose) const{

    std::vector<gtsam::Point3> points;

    // get 3D points
    cv::Point3f p1 = mapPoints3d_.at<cv::Point3f>(rowId_pt1);
    std::cout << "p1 " << p1 << std::endl;
    gtsam::Point3 p1_C = gtsam::Point3(double(p1.x),double(p1.y),double(p1.z));
    points.push_back(leftCameraPose.transform_to(p1_C)); // checks elongation in *camera frame*

    cv::Point3f p2 = mapPoints3d_.at<cv::Point3f>(rowId_pt2);
    gtsam::Point3 p2_C = gtsam::Point3(double(p2.x),double(p2.y),double(p2.z));
    points.push_back(leftCameraPose.transform_to(p2_C)); // checks elongation in *camera frame*
    std::cout << "p2 " << p2 << std::endl;

    cv::Point3f p3 = mapPoints3d_.at<cv::Point3f>(rowId_pt3);
    gtsam::Point3 p3_C = gtsam::Point3(double(p3.x),double(p3.y),double(p3.z));
    points.push_back(leftCameraPose.transform_to(p3_C)); // checks elongation in *camera frame*
    std::cout << "p3 " << p3 << std::endl;

    return UtilsGeometry::getRatioBetweenTangentialAndRadialDisplacement(points);
  }
  /* ----------------------------------------------------------------------------- */
  // searches in both the keypoints in frame as well as in the
  int findRowIdFromPixel(Frame& frame, KeypointCV px) const{
    // output
    int rowId_pt;
    bool found = false;

    // look for a lmk id
    LandmarkId id_pt = frame.findLmkIdFromPixel(px);
    if(id_pt != -1){ // if we found the point in the frame (it has a valid lmk id)
      auto it = lmkIdToMapPointId_.find(id_pt); // get row ids
      if(it != lmkIdToMapPointId_.end()){
        rowId_pt = it->second;
        found = true;
      }else{
        throw std::runtime_error("findRowIdFromPixel: kpt has lmk id but was not found in lmkIdToMapPointId_");
      }
    }else{ // we have to look for the point inside
      for(size_t i=0; i<keypointToMapPointId_.size();i++){
        if(keypointToMapPointId_.at(i).first.x == px.x &&
            keypointToMapPointId_.at(i).first.y == px.y){
          rowId_pt = keypointToMapPointId_.at(i).second;
          found = true;
          break;
        }
      }
    }
    if(found == false){ throw std::runtime_error("findRowIdFromPixel: kpt not found at all"); }
    return rowId_pt;
  }

  /* ----------------------------------------------------------------------------- */
  // Create a 2D mesh from 2D corners in an image, coded as a Frame class
  cv::Mat getTriangulationIndices(std::vector<cv::Vec6f> triangulation2D, Frame& frame,
      gtsam::Pose3 leftCameraPose, // for some geometric check
      double minRatioBetweenLargestAnSmallestSide, double min_elongation_ratio) const{
    // Raw integer list of the form: (n,id1,id2,...,idn, n,id1,id2,...,idn, ...)
    // where n is the number of points in the polygon, and id is a zero-offset
    // index into an associated cloud.
    cv::Mat polygon(0,1,CV_32SC1);

    // Populate polygons with indices:
    // note: we restrict to valid triangles in which each landmark has a 3D point
    for(size_t i=0; i<triangulation2D.size();i++){ // TODO: this is doing a lot of computation

      cv::Vec6f t = triangulation2D[i];

      // get lmk ids mapPoints3d_ (iterators)
      int rowId_pt1 = findRowIdFromPixel(frame, cv::Point2f(t[0], t[1]));
      int rowId_pt2 = findRowIdFromPixel(frame, cv::Point2f(t[2], t[3]));
      int rowId_pt3 = findRowIdFromPixel(frame, cv::Point2f(t[4], t[5]));

      double ratioSquaredSides = getRatioBetweenSmallestAndLargestSidesquared(rowId_pt1,rowId_pt2,rowId_pt3);
      double ratioTangentialRadial = getRatioBetweenTangentialAndRadialDisplacement(rowId_pt1,rowId_pt2,rowId_pt3, leftCameraPose);

      // check if triangle is not elongated
      if( (ratioSquaredSides >= minRatioBetweenLargestAnSmallestSide*minRatioBetweenLargestAnSmallestSide) &&
          (ratioTangentialRadial >= min_elongation_ratio)){
        polygon.push_back(3); // add rows
        polygon.push_back(rowId_pt1); // row in mapPoints3d_
        polygon.push_back(rowId_pt2); // row in mapPoints3d_
        polygon.push_back(rowId_pt3); // row in mapPoints3d_
      }
    }
    return polygon;
  }
  /* ----------------------------------------------------------------------------- */
  // Update map: update structures keeping memory of the map before visualization
  void updateMap3D(std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId,
      std::vector<std::pair<KeypointCV, gtsam::Point3> > pointsWithoutId =
          std::vector<std::pair<KeypointCV, gtsam::Point3> >()){

    // update 3D points by adding new points or updating old reobserved ones (for the one with lmk id)
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
    // add other extra points without lmk if:
    for(size_t i=0; i<pointsWithoutId.size();i++) {
      KeypointCV kps_i = pointsWithoutId.at(i).first;
      gtsam::Point3 point_i = pointsWithoutId.at(i).second;

      cv::Point3f p(float(point_i.x()), float(point_i.y()), float(point_i.z()));
      std::cout << "pwihtout " << p << std::endl;

      mapPoints3d_.push_back(p);
      keypointToMapPointId_.push_back(std::pair<KeypointCV,int>(kps_i,points3D_count_));
      ++points3D_count_;
    }
  }
  /* ----------------------------------------------------------------------------- */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId,
      std::shared_ptr<StereoFrame> stereoFrame,
      gtsam::Pose3 leftCameraPose,
      Mesh2Dtype mesh2Dtype = Mesh2Dtype::VALIDKEYPOINTS,
      float maxGradInTriangle = 50,
      double minRatioBetweenLargestAnSmallestSide = 0,
      double min_elongation_ratio = 0.5)
  {
    // debug:
    bool doVisualize2Dmesh = true;

    // build 2D mesh
    stereoFrame->createMesh2Dplanes(maxGradInTriangle,mesh2Dtype);
    if(doVisualize2Dmesh){stereoFrame->visualizeMesh2Dplanes(100);}

    // update 3D points (possibly replacing some points with new estimates)
    std::vector<std::pair<KeypointCV, gtsam::Point3> > pointsWithoutId;
    if(mesh2Dtype == Mesh2Dtype::DENSE){
      for(size_t i=0; i<stereoFrame->extraStereoKeypoints_.size();i++){ // convert to global coordinates
        KeypointCV px = stereoFrame->extraStereoKeypoints_.at(i).first;
        gtsam::Point3 point = leftCameraPose.transform_from(stereoFrame->extraStereoKeypoints_.at(i).second);
        pointsWithoutId.push_back(std::make_pair(px,point));
      }
    }
    updateMap3D(pointsWithId,pointsWithoutId);

    // concatenate mesh in the current image to existing mesh
    polygonsMesh_.push_back(getTriangulationIndices(stereoFrame->triangulation2Dplanes_,
        stereoFrame->left_frame_, leftCameraPose,
        minRatioBetweenLargestAnSmallestSide,min_elongation_ratio));

    // after filling in polygonsMesh_, we don't need this, and it must be reset:
    keypointToMapPointId_.resize(0);
  }
  /* ----------------------------------------------------------------------------- */
  // Update mesh: update structures keeping memory of the map before visualization
  void updateMesh3D(std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId,
      Frame& frame,
      gtsam::Pose3 leftCameraPose,
      double minRatioBetweenLargestAnSmallestSide = 0,
      double min_elongation_ratio = 0.5)
  {
    // debug:
    bool doVisualize2Dmesh = true;

    // update 3D points (possibly replacing some points with new estimates)
    updateMap3D(pointsWithId);

    // build 2D mesh, restricted to points with lmk!=-1
    frame.createMesh2D();
    if(doVisualize2Dmesh){frame.visualizeMesh2D(100);}

    // concatenate mesh in the current image to existing mesh
    polygonsMesh_.push_back(getTriangulationIndices(frame.triangulation2D_,
        frame, leftCameraPose,
        minRatioBetweenLargestAnSmallestSide,min_elongation_ratio));
  }
};
} // namespace VIO
#endif /* Mesher_H_ */


