/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackEndParams.h
 * @brief  Class collecting the parameters of the Visual Inertial odometry pipeline
 * for the RegularVIO implementation.
 * @author Antoni Rosinol
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <gtsam/slam/SmartFactorParams.h>

#include "VioBackEndParams.h"

#include <glog/logging.h>

namespace VIO {

///////////////////////////////////////////////////////////////////////////////////////
class RegularVioBackEndParams: public VioBackEndParams
{
public:
  RegularVioBackEndParams(
      const double monoNoiseSigma = 3.0,
      const double stereoNoiseSigma = 3.0,
      const double regularityNoiseSigma = 0.1,
      // NomrType -> 0: L2 norm, 1: Huber, 2: Tukey.
      const size_t monoNormType = 0,
      const size_t stereoNormType = 0,
      const size_t regularityNormType = 2,
      const double huberParam = 1.345,
      const double tukeyParam = 4.6851) :
    VioBackEndParams(), // Use the default vio parameters.
    monoNoiseSigma_(monoNoiseSigma), stereoNoiseSigma_(stereoNoiseSigma),
    regularityNoiseSigma_(regularityNoiseSigma),
    monoNormType_(monoNormType), stereoNormType_(stereoNormType),
    regularityNormType_(regularityNormType),
    huberParam_(huberParam), tukeyParam_(tukeyParam)
  {
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
  int monoNormType_, stereoNormType_, regularityNormType_;
  double huberParam_, tukeyParam_;

  // Needed for virtual classes.
  virtual ~RegularVioBackEndParams() = default;

public:
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  virtual bool parseYAML(const std::string& filepath) {
    // make sure that each YAML file has %YAML:1.0 as first line
    cv::FileStorage fs;
    openFile(filepath, &fs);
    bool result =
        parseYAMLVioBackEndParams(fs) && parseYAMLRegularVioBackEndParams(fs);
    closeFile(&fs);
    return result;
  }

  /* ------------------------------------------------------------------------ */
  virtual bool equals(const VioBackEndParams& vp2, double tol = 1e-8) const {
    return equalsVioBackEndParams(vp2, tol) &&
        equalsRegularVioBackEndParams(vp2, tol);
  }

  /* ------------------------------------------------------------------------ */
  virtual void print() const {
    printVioBackEndParams();
  }

  // Use this to safely cast VioBackEndParams to RegularVioBackEndParams.
  static RegularVioBackEndParams safeCast (const VioBackEndParams& params) {
    try {
      return dynamic_cast<const RegularVioBackEndParams&>(params);
    } catch(const std::bad_cast& e) {
      LOG(ERROR) << "Seems that you are casting VioBackEndParams to "
                    "RegularVioBackEndParams, but this objest is not "
                    "a RegularVioBackEndParams!";
      CHECK(false) << e.what();
    }
  }

protected:
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  // Parse params YAML file
  bool parseYAMLRegularVioBackEndParams(const cv::FileStorage& fs){
    cv::FileNode file_handle;

    file_handle = fs["monoNoiseSigma"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> monoNoiseSigma_;
    file_handle = fs["monoNormType"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> monoNormType_;
    file_handle = fs["stereoNoiseSigma"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> stereoNoiseSigma_;
    file_handle = fs["stereoNormType"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> stereoNormType_;
    file_handle = fs["regularityNoiseSigma"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> regularityNoiseSigma_;
    file_handle = fs["huberParam"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> huberParam_;
    file_handle = fs["tukeyParam"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> tukeyParam_;
    file_handle = fs["regularityNormType"];
    CHECK(file_handle.type() != cv::FileNode::NONE); file_handle >> regularityNormType_;

    return true;
  }

  /* ------------------------------------------------------------------------------------- */
  bool equalsRegularVioBackEndParams(const VioBackEndParams& vp2, double tol = 1e-8) const{
    RegularVioBackEndParams rvp2 = RegularVioBackEndParams::safeCast(vp2);
    return (fabs(monoNoiseSigma_ - rvp2.monoNoiseSigma_) <= tol) &&
        (monoNormType_ == rvp2.monoNormType_) &&
        (fabs(stereoNoiseSigma_ - rvp2.stereoNoiseSigma_) <= tol) &&
        (stereoNormType_ == rvp2.stereoNormType_) &&
        (fabs(regularityNoiseSigma_ - rvp2.regularityNoiseSigma_) <= tol) &&
        (fabs(huberParam_ - rvp2.huberParam_) <= tol) &&
        (fabs(tukeyParam_ - rvp2.tukeyParam_) <= tol) &&
        (regularityNormType_ == rvp2.regularityNormType_);
  }

  /* ------------------------------------------------------------------------------------- */
  void printRegularVioBackEndParams() const{
  }


};

} // namespace VIO
