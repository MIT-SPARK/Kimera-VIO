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

#include <fstream>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <string>
#include <unordered_map>

#include <gtsam/slam/SmartFactorParams.h>

#include <opencv2/core/core.hpp>

#include <glog/logging.h>

#include "VioBackEndParams.h"

namespace VIO {

///////////////////////////////////////////////////////////////////////////////////////
class RegularVioBackEndParams : public VioBackEndParams {
public:
 RegularVioBackEndParams(const double monoNoiseSigma = 3.0,
                         const double stereoNoiseSigma = 3.0,
                         const double regularityNoiseSigma = 0.1,
                         const double monoNormParam = 0.0,
                         const double stereoNormParam = 0.0,
                         const double regularityNormParam = 4.6851,
                         // NomrType -> 0: L2 norm, 1: Huber, 2: Tukey.
                         const size_t monoNormType = 0,
                         const size_t stereoNormType = 0,
                         const size_t regularityNormType = 2,
                         const double huberParam = 1.345,
                         const double tukeyParam = 4.6851)
     : VioBackEndParams(),  // Use the default vio parameters.
       monoNoiseSigma_(monoNoiseSigma),
       stereoNoiseSigma_(stereoNoiseSigma),
       regularityNoiseSigma_(regularityNoiseSigma),
       monoNormParam_(monoNormParam),
       stereoNormParam_(stereoNormParam),
       regularityNormParam_(regularityNormParam),
       monoNormType_(monoNormType),
       stereoNormType_(stereoNormType),
       regularityNormType_(regularityNormType),
       huberParam_(huberParam),
       tukeyParam_(tukeyParam) {
   // Trivial sanity checks.
   CHECK_GE(monoNoiseSigma_, 0.0);
   CHECK_GE(stereoNoiseSigma_, 0.0);
   CHECK_GE(regularityNoiseSigma_, 0.0);
   CHECK_GE(monoNormType_, 0);
   CHECK_GE(stereoNormType_, 0);
   CHECK_GE(regularityNormType_, 0);
   CHECK_GE(huberParam_, 0.0);
   CHECK_GE(tukeyParam_, 0.0);
  }

  double monoNoiseSigma_, stereoNoiseSigma_, regularityNoiseSigma_;
  double monoNormParam_, stereoNormParam_, regularityNormParam_;
  int monoNormType_, stereoNormType_, regularityNormType_;
  double huberParam_, tukeyParam_;

  // Needed for virtual classes.
  virtual ~RegularVioBackEndParams() = default;

public:
  virtual bool parseYAML(const std::string &filepath) {
    yaml_parser_ = std::make_shared<YamlParser>(filepath);
    return parseYAMLVioBackEndParams() && parseYAMLRegularVioBackEndParams();
  }

  /* ------------------------------------------------------------------------ */
  virtual bool equals(const VioBackEndParams &vp2, double tol = 1e-8) const {
    return equalsVioBackEndParams(vp2, tol) &&
           equalsRegularVioBackEndParams(vp2, tol);
  }

  /* ------------------------------------------------------------------------ */
  virtual void print() const { printVioBackEndParams(); }

  // Use this to safely cast VioBackEndParams to RegularVioBackEndParams.
  static RegularVioBackEndParams safeCast(const VioBackEndParams &params) {
    try {
      return dynamic_cast<const RegularVioBackEndParams &>(params);
    } catch (const std::bad_cast &e) {
      LOG(ERROR) << "Seems that you are casting VioBackEndParams to "
                    "RegularVioBackEndParams, but this object is not "
                    "a RegularVioBackEndParams!";
      LOG(FATAL) << e.what();
    } catch (...) {
      LOG(FATAL) << "Exception caught when casting to RegularVioBackEndParams.";
    }
  }

protected:
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   */
  // Parse params YAML file
  bool parseYAMLRegularVioBackEndParams() {
    CHECK(yaml_parser_ != nullptr);
    yaml_parser_->getYamlParam("monoNoiseSigma", &monoNoiseSigma_);
    yaml_parser_->getYamlParam("monoNormType", &monoNormType_);
    yaml_parser_->getYamlParam("monoNormParam", &monoNormParam_);
    yaml_parser_->getYamlParam("stereoNoiseSigma", &stereoNoiseSigma_);
    yaml_parser_->getYamlParam("stereoNormType", &stereoNormType_);
    yaml_parser_->getYamlParam("stereoNormParam", &stereoNormParam_);
    yaml_parser_->getYamlParam("regularityNoiseSigma", &regularityNoiseSigma_);
    yaml_parser_->getYamlParam("regularityNormParam", &regularityNormParam_);
    yaml_parser_->getYamlParam("regularityNormType", &regularityNormType_);
    return true;
  }

  /* -------------------------------------------------------------------------------------
   */
  bool equalsRegularVioBackEndParams(const VioBackEndParams &vp2,
                                     double tol = 1e-8) const {
    RegularVioBackEndParams rvp2 = RegularVioBackEndParams::safeCast(vp2);
    return (fabs(monoNoiseSigma_ - rvp2.monoNoiseSigma_) <= tol) &&
           (monoNormType_ == rvp2.monoNormType_) &&
           (fabs(stereoNoiseSigma_ - rvp2.stereoNoiseSigma_) <= tol) &&
           (stereoNormType_ == rvp2.stereoNormType_) &&
           (fabs(regularityNoiseSigma_ - rvp2.regularityNoiseSigma_) <= tol) &&
           (fabs(monoNormParam_ - rvp2.monoNormParam_) <= tol) &&
           (fabs(stereoNormParam_ - rvp2.stereoNormParam_) <= tol) &&
           (fabs(regularityNormParam_ - rvp2.regularityNormParam_) <= tol) &&
           (regularityNormType_ == rvp2.regularityNormType_);
  }

  /* -------------------------------------------------------------------------------------
   */
  void printRegularVioBackEndParams() const {}
};

}  // namespace VIO
