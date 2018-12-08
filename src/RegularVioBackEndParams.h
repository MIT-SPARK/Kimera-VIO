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
      // IMU PARAMS
      const double gyroNoiseDensity = 0.00016968,
      const double accNoiseDensity = 0.002,
      const double gyroBiasSigma = 1.9393e-05,
      const double accBiasSigma = 0.003,
      const double imuIntegrationSigma = 1e-8,
      const gtsam::Vector3 n_gravity = gtsam::Vector3(0.0,0.0,-9.81), // gravity in navigation frame, according to
      const double nominalImuRate = 0.005,
      // INITIALIZATION SETTINGS
      const bool autoInitialize = false,
      const bool roundOnAutoInitialize = false,
      const double initialPositionSigma = 0.00001,
      const double initialRollPitchSigma = 10.0 / 180.0 * M_PI,
      const double initialYawSigma = 0.1 / 180.0 * M_PI,
      const double initialVelocitySigma = 1e-3,
      const double initialAccBiasSigma = 0.1,
      const double initialGyroBiasSigma = 0.01,
      // http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets, the x axis points upwards
      // VISION PARAMS
      const gtsam::LinearizationMode linMode = gtsam::HESSIAN,
      const gtsam::DegeneracyMode degMode = gtsam::ZERO_ON_DEGENERACY,
      const double smartNoiseSigma = 3,
      // TODO inherit from this for regularVIO backend
      const double monoNoiseSigma = 3, // for regularVioBackEnd only.
      const size_t monoNormType = 0, // for regularVioBackEnd only.
      const double stereoNoiseSigma = 3, // for regularVioBackEnd only.
      const size_t stereoNormType = 0, // for regularVioBackEnd only.
      const double regularityNoiseSigma = 0.1, // for regularVioBackEnd only.
      const double huberParam = 1.345, // for regularVioBackEnd only.
      const double tukeyParam = 4.6851, // for regularVioBackEnd only.
      const size_t regularityNormType = 2, // for regularVioBackEnd only. 0: norm 2, 1: Huber, 2: Tukey.
      const double rankTolerance = 1, // we might also use 0.1
      const double landmarkDistanceThreshold = 20, // max distance to triangulate point in meters
      const double outlierRejection = 8, // max acceptable reprojection error // before tuning: 3
      const double retriangulationThreshold = 1e-3,
      const bool addBetweenStereoFactors = true,
      const double betweenRotationPrecision = 0.0, // inverse of variance
      const double betweenTranslationPrecision = 1/(0.1*0.1), // inverse of variance
      // OPTIMIZATION PARAMS
      const double relinearizeThreshold = 1e-2, // Before tuning: 1e-3
      const double relinearizeSkip = 1,
      const double zeroVelocitySigma = 1e-3, // zero velocity prior when disparity is low
      const double noMotionPositionSigma = 1e-3,
      const double noMotionRotationSigma = 1e-4,
      const double constantVelSigma = 1e-2,
      const size_t numOptimize = 2,
      const double horizon = 6, // in seconds
      const bool useDogLeg = false
      ) :
    monoNoiseSigma_(monoNoiseSigma), stereoNoiseSigma_(stereoNoiseSigma),
    regularityNoiseSigma_(regularityNoiseSigma),
    monoNormType_(monoNormType), stereoNormType_(stereoNormType),
    regularityNormType_(regularityNormType),
    huberParam_(huberParam), tukeyParam_(tukeyParam),
    VioBackEndParams(
      gyroNoiseDensity,
      accNoiseDensity,
      gyroBiasSigma,
      accBiasSigma,
      imuIntegrationSigma,
      n_gravity, // gravity in navigation frame, according to
      nominalImuRate,
      // INITIALIZATION SETTINGS
      autoInitialize,
      roundOnAutoInitialize,
      initialPositionSigma,
      initialRollPitchSigma,
      initialYawSigma,
      initialVelocitySigma,
      initialAccBiasSigma,
      initialGyroBiasSigma,
      // http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets, the x axis points upwards
      // VISION PARAMS
      linMode,
      degMode,
      smartNoiseSigma,
      rankTolerance, // we might also use 0.1
      landmarkDistanceThreshold, // max distance to triangulate point in meters
      outlierRejection, // max acceptable reprojection error // before tuning: 3
      retriangulationThreshold,
      addBetweenStereoFactors,
      betweenRotationPrecision, // inverse of variance
      betweenTranslationPrecision, // inverse of variance
      // OPTIMIZATION PARAMS
      relinearizeThreshold, // Before tuning: 1e-3
      relinearizeSkip,
      zeroVelocitySigma, // zero velocity prior when disparity is low
      noMotionPositionSigma,
      noMotionRotationSigma,
      constantVelSigma,
      numOptimize,
      horizon, // in seconds
      useDogLeg)
  {
    // Trivial sanity checks.
    CHECK(horizon >= 0);
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
