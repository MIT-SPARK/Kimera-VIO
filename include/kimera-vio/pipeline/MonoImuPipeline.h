/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoImuPipeline.h
 * @brief  Implements MonoVIO pipeline workflow.
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/dataprovider/MonoDataProviderModule.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/pipeline/Pipeline.h"

namespace VIO {

class MonoImuPipeline : public Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(MonoImuPipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoImuPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  MonoImuPipeline(const VioParams& params,
               Visualizer3D::UniquePtr&& visualizer = nullptr,
               DisplayBase::UniquePtr&& displayer = nullptr);

  ~MonoImuPipeline() = default;

 protected:
  Camera::ConstPtr camera_;
};

}  // namespace VIO
