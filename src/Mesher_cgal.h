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
typedef Kernel::Point_3 Point;
typedef Reconstruction::Facet_const_iterator Facet_iterator;

namespace VIO {

class Mesher_cgal {

public:
  // Create a 3D mesh using cgal
  static void CreateMesh3D(const Frame& frame){

  }
};

} // namespace VIO
#endif /* Mesher_cgal_H_ */


