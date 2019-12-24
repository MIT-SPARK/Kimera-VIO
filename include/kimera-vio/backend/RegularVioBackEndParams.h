/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackEndParams.h
 * @brief  Class collecting the parameters of the Visual Inertial odometry
 * pipeline for the RegularVIO implementation.
 * @author Antoni Rosinol
 */

#pragma once

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include <gtsam/slam/SmartFactorParams.h>

#include <glog/logging.h>

#include "kimera-vio/backend/RegularVioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class RegularVioBackEndParams : public VioBackEndParams {
 public:
  KIMERA_POINTER_TYPEDEFS(RegularVioBackEndParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RegularVioBackEndParams();
  virtual ~RegularVioBackEndParams() = default;

 public:
  virtual bool parseYAML(const std::string& filepath) override;

  virtual bool equals(const VioBackEndParams& vp2,
                      double tol = 1e-8) const override;

  virtual void print() const override;

  // Use this to safely cast VioBackEndParams to RegularVioBackEndParams.
  static RegularVioBackEndParams safeCast(const VioBackEndParams& params);

 protected:
  // Parse params YAML file
  bool parseYAMLRegularVioBackEndParams(const YamlParser& yaml_parser);

  bool equalsRegularVioBackEndParams(const VioBackEndParams& vp2,
                                     double tol = 1e-8) const;

  void printRegularVioBackEndParams() const {}

 public:
  RegularBackendModality backend_modality_ =
      RegularBackendModality::STRUCTURELESS_PROJECTION_AND_REGULARITY;

  double monoNoiseSigma_ = 3.0;
  double stereoNoiseSigma_ = 3.0;
  double regularityNoiseSigma_ = 0.1;
  double monoNormParam_ = 0.0;
  double stereoNormParam_ = 0.0;
  double regularityNormParam_ = 4.6851;

  // NomrType -> 0: L2 norm, 1: Huber, 2: Tukey.
  int monoNormType_ = 0;
  int stereoNormType_ = 0;
  int regularityNormType_ = 2;

  double huberParam_ = 1.345;
  double tukeyParam_ = 4.6851;
};

}  // namespace VIO
