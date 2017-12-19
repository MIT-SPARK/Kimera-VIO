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

#include "Frame.h"
#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Timer.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel     Kernel;
typedef CGAL::Scale_space_surface_reconstruction_3<Kernel>    Reconstruction;
typedef Kernel::Point_3 Point_cgal;
typedef Reconstruction::Facet_const_iterator Facet_iterator;

namespace VIO {

class Mesher_cgal {

public:
  // Create a 3D mesh using cgal
  static void CreateMesh3D(std::vector<std::pair<LandmarkId, gtsam::Point3> > pointsWithId){
    // sanity check dimension
    if(pointsWithId.size() == 0) // no points to visualize
      return;

    // Read the data.
    std::vector<Point_cgal> points;
    for(size_t i=0; i<pointsWithId.size(); i++){
      gtsam::Point3 pt_gtsam = pointsWithId.at(i).second;
      points.push_back(Point_cgal(pt_gtsam.x(), pt_gtsam.y(), pt_gtsam.z()));
    }

    std::cerr << "read points: " << points.size() << " points." << std::endl;
    std::cerr << "Reconstruction ";
    CGAL::Timer t;
    t.start();
    // Construct the mesh in a scale space.
    Reconstruction reconstruct (points.begin(), points.end());
    reconstruct.increase_scale(4);
    reconstruct.reconstruct_surface();
    std::cerr << "done in " << t.time() << " sec." << std::endl;
    std::cerr << "Done." << std::endl;
  }
};

} // namespace VIO
#endif /* Mesher_cgal_H_ */


