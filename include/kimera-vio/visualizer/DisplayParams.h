/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DisplayParams.h
 * @brief  Parameters describing a monocular camera.
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <string>
#include <vector>

#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/**
 * @brief The DisplayType enum: enumerates the types of supported renderers.
 */
enum class DisplayType { kOpenCV = 0, kPangolin = 1 };

/*
 * Class describing display parameters.
 */
class DisplayParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(DisplayParams);

  DisplayParams();
  DisplayParams(const DisplayType& display_type);
  ~DisplayParams() override = default;

  // Parse YAML file describing camera parameters.
  bool parseYAML(const std::string& filepath) override;

  // Display all params.
  void print() const override;

  // Assert equality up to a tolerance.
  virtual bool equals(const DisplayParams& cam_par,
                      const double& tol = 1e-9) const;

 protected:
  bool equals(const PipelineParams& rhs) const override {
    return equals(static_cast<const DisplayParams&>(rhs), 1e-9);
  }

 public:
  DisplayType display_type_;
};

}  // namespace VIO
