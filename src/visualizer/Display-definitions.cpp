/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Display-definitions.cpp
 * @brief  Definitions for display module
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/Display-definitions.h"

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(mesh_shading, 0, "Mesh shading:\n 0: Flat, 1: Gouraud, 2: Phong");
DEFINE_int32(mesh_representation,
             1,
             "Mesh representation:\n 0: Points, 1: Surface, 2: Wireframe");
DEFINE_bool(set_mesh_ambient,
            false,
            "Whether to use ambient light for the "
            "mesh.");
DEFINE_bool(set_mesh_lighting, true, "Whether to use lighting for the mesh.");

namespace VIO {

// Contains internal data for Visualizer3D window.
WindowData::WindowData()
    : window_(cv::viz::Viz3d("3D Visualizer")),
      mesh_representation_(FLAGS_mesh_representation),
      mesh_shading_(FLAGS_mesh_shading),
      mesh_ambient_(FLAGS_set_mesh_ambient),
      mesh_lighting_(FLAGS_set_mesh_lighting) {}

WindowData::~WindowData() {
  // OpenCV 3d Viz has an issue that I can't resolve, it throws a segfault
  // at the end of the program. Probably because of memory not released.
  // See issues in opencv git:
  // https://github.com/opencv/opencv/issues/11219 and many more...
  window_.close();
}

}  // namespace VIO
