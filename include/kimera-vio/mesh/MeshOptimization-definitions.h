/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MeshOptimization-definitions.h
 * @brief  Definitions for the mesh optimization module
 * @author Antoni Rosinol
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

enum class MeshOptimizerType {
  kConnectedMesh = 0,
  kDisconnectedMesh = 1,
  kClosedForm = 2,
  kGtsamMesh = 3
};

enum class MeshColorType {
  kVertexFlatColor = 0,
  kVertexRGB = 1,
  kVertexDepthVariance = 2,
  kVertexSupport = 3,
};

struct MeshOptimizationInput {
  KIMERA_POINTER_TYPEDEFS(MeshOptimizationInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MeshOptimizationInput);

  MeshOptimizationInput() = default;
  ~MeshOptimizationInput() = default;

  cv::Mat pcl;
  cv::Mat pcl_colors;
  Mesh2D mesh_2d;
};

struct MeshOptimizationOutput {
  KIMERA_POINTER_TYPEDEFS(MeshOptimizationOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MeshOptimizationOutput);

  MeshOptimizationOutput() = default;
  ~MeshOptimizationOutput() = default;

  Mesh3D optimized_mesh_3d;
};

}  // namespace VIO
