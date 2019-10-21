/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher_cgal.h
 * @brief  Build and visualize 2D mesh from Frame, using the CGAL library
 * @author Luca Carlone, AJ Haeffner
 */

#ifndef Mesher_cgal_H_
#define Mesher_cgal_H_

#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Timer.h>

#include "kimera-vio/Frame.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Scale_space_surface_reconstruction_3<Kernel> Reconstruction;
typedef Kernel::Point_3 Point_cgal;
typedef Reconstruction::Facet_const_iterator Facet_iterator;

namespace VIO {

class Mesher_cgal {
 public:
  //
  /* -----------------------------------------------------------------------------
   */
  // Create a 3D mesh using cgal
  static cv::Mat CreateMesh3D_MapPointId(cv::Mat pointsMatcv) {
    // sanity check dimension
    if (pointsMatcv.rows == 0)  // no points to visualize
      return pointsMatcv;

    // Read the data.
    std::vector<Point_cgal> points;
    cv::Point3f* data = pointsMatcv.ptr<cv::Point3f>();
    for (size_t i = 0; i < pointsMatcv.rows; i++)
      points.push_back(
          Point_cgal(double(data[i].x), double(data[i].y), double(data[i].z)));

    std::cerr << "read points: " << points.size() << " points." << std::endl;
    std::cerr << "Reconstruction ";
    CGAL::Timer t;
    t.start();
    t.reset();
    // Construct the mesh in a scale space.
    Reconstruction reconstruct(points.begin(), points.end());
    reconstruct.increase_scale(4);
    reconstruct.reconstruct_surface();
    std::cerr << "done in " << t.time() << " sec." << std::endl;
    std::cerr << "Done." << std::endl;

    // Raw integer list of the form: (n,id1,id2,...,idn, n,id1,id2,...,idn, ...)
    // where n is the number of points in the polygon, and id is a zero-offset
    // index into an associated cloud.
    cv::Mat polygon(0, 1, CV_32SC1);
    for (auto it = reconstruct.facets_begin(); it != reconstruct.facets_end();
         it++) {
      // std:cout << "*it " << *it << std::endl <<
      //    ": "  << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " <<
      //    std::endl;
      polygon.push_back(3);              // add rows
      polygon.push_back(int((*it)[0]));  // row in mapPoints3d_
      polygon.push_back(int((*it)[1]));  // row in mapPoints3d_
      polygon.push_back(int((*it)[2]));  // row in mapPoints3d_
    }
    std::cout << "polygon.rows:" << polygon.rows
              << " polygon.cols: " << polygon.cols << std::endl;
    return polygon;
  }
};

}  // namespace VIO
#endif /* Mesher_cgal_H_ */
