/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEndParams.h
 * @brief  Class parsing the parameters for the VIO's Backend from a YAML file.
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core/core.hpp>

#include <gtsam/slam/SmartFactorParams.h>

#include <glog/logging.h>

#include "kimera-vio/datasource/DataSource-definitions.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

class VioBackEndParams {
 public:
  VioBackEndParams(){};
  virtual ~VioBackEndParams() = default;

  //! Initialization params
  double initialPositionSigma_;
  double initialRollPitchSigma_;
  double initialYawSigma_;
  double initialVelocitySigma_;
  double initialAccBiasSigma_;
  double initialGyroBiasSigma_;

  //! Imu params
  // TODO(TONI): these imu stuff should be in its own yaml parser!!
  double gyroNoiseDensity_;
  double accNoiseDensity_;
  double imuIntegrationSigma_;
  double gyroBiasSigma_;
  double accBiasSigma_;
  double nominalImuRate_;
  gtsam::Vector3 n_gravity_;

  //! Initialization parameters
  int autoInitialize_;
  /// Only used if autoInitialize set to false.
  VioNavState initial_ground_truth_state_;
  bool roundOnAutoInitialize_;

  //! Smart factor params
  gtsam::LinearizationMode linearizationMode_;
  gtsam::DegeneracyMode degeneracyMode_;
  double smartNoiseSigma_;
  double rankTolerance_;
  double landmarkDistanceThreshold_;
  double outlierRejection_;
  double retriangulationThreshold_;

  bool addBetweenStereoFactors_;

  double betweenRotationPrecision_;
  double betweenTranslationPrecision_;

  //! iSAM params
  double relinearizeThreshold_;
  double relinearizeSkip_;
  double horizon_;
  int numOptimize_;
  bool useDogLeg_;

  //! No Motion params
  double zeroVelocitySigma_;
  double noMotionPositionSigma_;
  double noMotionRotationSigma_;
  double constantVelSigma_;

 public:
  virtual bool equals(const VioBackEndParams& vp2, double tol = 1e-8) const {
    return equalsVioBackEndParams(vp2, tol);
  }

  virtual void print() const { printVioBackEndParams(); }

  // Parse params YAML file
  virtual bool parseYAML(const std::string& filepath) {
    yaml_parser_ = std::make_shared<YamlParser>(filepath);
    return parseYAMLVioBackEndParams();
  }

 protected:
  bool parseYAMLVioBackEndParams() {
    CHECK(yaml_parser_ != nullptr);
    // IMU PARAMS
    yaml_parser_->getYamlParam("gyroNoiseDensity", &gyroNoiseDensity_);
    yaml_parser_->getYamlParam("accNoiseDensity", &accNoiseDensity_);
    yaml_parser_->getYamlParam("gyroBiasSigma", &gyroBiasSigma_);
    yaml_parser_->getYamlParam("accBiasSigma", &accBiasSigma_);
    yaml_parser_->getYamlParam("imuIntegrationSigma", &imuIntegrationSigma_);
    std::vector<double> n_gravity;
    yaml_parser_->getYamlParam("n_gravity", &n_gravity);
    CHECK_EQ(n_gravity.size(), 3);
    for (int k = 0; k < 3; k++) n_gravity_(k) = n_gravity[k];
    yaml_parser_->getYamlParam("nominalImuRate", &nominalImuRate_);

    // INITIALIZATION
    yaml_parser_->getYamlParam("autoInitialize", &autoInitialize_);
    yaml_parser_->getYamlParam("roundOnAutoInitialize",
                               &roundOnAutoInitialize_);
    yaml_parser_->getYamlParam("initialPositionSigma", &initialPositionSigma_);
    yaml_parser_->getYamlParam("initialRollPitchSigma",
                               &initialRollPitchSigma_);
    yaml_parser_->getYamlParam("initialYawSigma", &initialYawSigma_);
    yaml_parser_->getYamlParam("initialVelocitySigma", &initialVelocitySigma_);
    yaml_parser_->getYamlParam("initialAccBiasSigma", &initialAccBiasSigma_);
    yaml_parser_->getYamlParam("initialGyroBiasSigma", &initialGyroBiasSigma_);

    // VISION PARAMS
    int linearization_mode_id;
    yaml_parser_->getYamlParam("linearizationMode", &linearization_mode_id);
    switch (linearization_mode_id) {
      case 0:
        linearizationMode_ = gtsam::HESSIAN;
        break;
      case 1:
        linearizationMode_ = gtsam::IMPLICIT_SCHUR;
        break;
      case 2:
        linearizationMode_ = gtsam::JACOBIAN_Q;
        break;
      case 3:
        linearizationMode_ = gtsam::JACOBIAN_SVD;
        break;
      default:
        LOG(FATAL) << "Wrong linearizationMode in VIO backend parameters.";
    }

    int degeneracyModeId;
    yaml_parser_->getYamlParam("degeneracyMode", &degeneracyModeId);
    switch (degeneracyModeId) {
      case 0:
        degeneracyMode_ = gtsam::IGNORE_DEGENERACY;
        break;
      case 1:
        degeneracyMode_ = gtsam::ZERO_ON_DEGENERACY;
        break;
      case 2:
        degeneracyMode_ = gtsam::HANDLE_INFINITY;
        break;
      default:
        LOG(FATAL) << "Wrong degeneracyMode in VIO backend parameters.";
    }

    yaml_parser_->getYamlParam("smartNoiseSigma", &smartNoiseSigma_);
    yaml_parser_->getYamlParam("rankTolerance", &rankTolerance_);
    yaml_parser_->getYamlParam("landmarkDistanceThreshold",
                               &landmarkDistanceThreshold_);
    yaml_parser_->getYamlParam("outlierRejection", &outlierRejection_);
    yaml_parser_->getYamlParam("retriangulationThreshold",
                               &retriangulationThreshold_);
    yaml_parser_->getYamlParam("addBetweenStereoFactors",
                               &addBetweenStereoFactors_);
    yaml_parser_->getYamlParam("betweenRotationPrecision",
                               &betweenRotationPrecision_);
    yaml_parser_->getYamlParam("betweenTranslationPrecision",
                               &betweenTranslationPrecision_);

    // OPTIMIZATION PARAMS
    yaml_parser_->getYamlParam("relinearizeThreshold", &relinearizeThreshold_);
    yaml_parser_->getYamlParam("relinearizeSkip", &relinearizeSkip_);
    yaml_parser_->getYamlParam("zeroVelocitySigma", &zeroVelocitySigma_);
    yaml_parser_->getYamlParam("noMotionPositionSigma",
                               &noMotionPositionSigma_);
    yaml_parser_->getYamlParam("noMotionRotationSigma",
                               &noMotionRotationSigma_);
    yaml_parser_->getYamlParam("constantVelSigma", &constantVelSigma_);
    yaml_parser_->getYamlParam("numOptimize", &numOptimize_);
    yaml_parser_->getYamlParam("horizon", &horizon_);
    yaml_parser_->getYamlParam("useDogLeg", &useDogLeg_);

    return true;
  }

  bool equalsVioBackEndParams(const VioBackEndParams& vp2,
                              double tol = 1e-8) const {
    return
        // IMU PARAMS
        (fabs(gyroNoiseDensity_ - vp2.gyroNoiseDensity_) <= tol) &&
        (fabs(accNoiseDensity_ - vp2.accNoiseDensity_) <= tol) &&
        (fabs(imuIntegrationSigma_ - vp2.imuIntegrationSigma_) <= tol) &&
        (fabs(gyroBiasSigma_ - vp2.gyroBiasSigma_) <= tol) &&
        (fabs(accBiasSigma_ - vp2.accBiasSigma_) <= tol) &&
        (fabs(n_gravity_(0) - vp2.n_gravity_(0)) <= tol) &&
        (fabs(n_gravity_(1) - vp2.n_gravity_(1)) <= tol) &&
        (fabs(n_gravity_(2) - vp2.n_gravity_(2)) <= tol) &&
        (fabs(nominalImuRate_ - vp2.nominalImuRate_) <= tol) &&
        // INITIALIZATION
        (autoInitialize_ == vp2.autoInitialize_) &&
        initial_ground_truth_state_.equals(vp2.initial_ground_truth_state_) &&
        (roundOnAutoInitialize_ == vp2.roundOnAutoInitialize_) &&
        (fabs(initialPositionSigma_ - vp2.initialPositionSigma_) <= tol) &&
        (fabs(initialRollPitchSigma_ - vp2.initialRollPitchSigma_) <= tol) &&
        (fabs(initialYawSigma_ - vp2.initialYawSigma_) <= tol) &&
        (fabs(initialVelocitySigma_ - vp2.initialVelocitySigma_) <= tol) &&
        (fabs(initialAccBiasSigma_ - vp2.initialAccBiasSigma_) <= tol) &&
        (fabs(initialGyroBiasSigma_ - vp2.initialGyroBiasSigma_) <= tol) &&
        // VISION PARAMS
        (linearizationMode_ == vp2.linearizationMode_) &&
        (degeneracyMode_ == vp2.degeneracyMode_) &&
        (fabs(smartNoiseSigma_ - vp2.smartNoiseSigma_) <= tol) &&
        (fabs(rankTolerance_ - vp2.rankTolerance_) <= tol) &&
        (fabs(landmarkDistanceThreshold_ - vp2.landmarkDistanceThreshold_) <=
         tol) &&
        (fabs(outlierRejection_ - vp2.outlierRejection_) <= tol) &&
        (fabs(retriangulationThreshold_ - vp2.retriangulationThreshold_) <=
         tol) &&
        (addBetweenStereoFactors_ == vp2.addBetweenStereoFactors_) &&
        (fabs(betweenRotationPrecision_ - vp2.betweenRotationPrecision_) <=
         tol) &&
        (fabs(betweenTranslationPrecision_ -
              vp2.betweenTranslationPrecision_) <= tol) &&
        // OPTIMIZATION PARAMS
        (fabs(relinearizeThreshold_ - vp2.relinearizeThreshold_) <= tol) &&
        (relinearizeSkip_ == vp2.relinearizeSkip_) &&
        (fabs(zeroVelocitySigma_ - vp2.zeroVelocitySigma_) <= tol) &&
        (fabs(noMotionPositionSigma_ - vp2.noMotionPositionSigma_) <= tol) &&
        (fabs(noMotionRotationSigma_ - vp2.noMotionRotationSigma_) <= tol) &&
        (fabs(constantVelSigma_ - vp2.constantVelSigma_) <= tol) &&
        (numOptimize_ == vp2.numOptimize_) && (horizon_ == vp2.horizon_) &&
        (useDogLeg_ == vp2.useDogLeg_);
  }

  void printVioBackEndParams() const {
    LOG(INFO) << "$$$$$$$$$$$$$$$$$$$$$ VIO PARAMETERS $$$$$$$$$$$$$$$$$$$$$\n"
              << "** IMU parameters **\n"
              << "gyroNoiseDensity_: " << gyroNoiseDensity_ << '\n'
              << "accNoiseDensity_: " << accNoiseDensity_ << '\n'
              << "imuIntegrationSigma_: " << imuIntegrationSigma_ << '\n'
              << "gyroBiasSigma_: " << gyroBiasSigma_ << '\n'
              << "accBiasSigma_: " << accBiasSigma_ << '\n'
              << "n_gravity_: " << n_gravity_.transpose() << '\n'
              << "nominalImuRate_: " << nominalImuRate_ << '\n'

              << "** INITIALIZATION parameters **\n"
              << "autoInitialize_: " << autoInitialize_ << '\n'
              << "initial_ground_truth_state_: ";
    initial_ground_truth_state_.print();
    LOG(INFO) << "roundOnAutoInitialize_: " << roundOnAutoInitialize_ << '\n'
              << "initialPositionSigma: " << initialPositionSigma_ << '\n'
              << "initialRollPitchSigma: " << initialRollPitchSigma_ << '\n'
              << "initialYawSigma: " << initialYawSigma_ << '\n'
              << "initialVelocitySigma: " << initialVelocitySigma_ << '\n'
              << "initialAccBiasSigma: " << initialAccBiasSigma_ << '\n'
              << "initialGyroBiasSigma: " << initialGyroBiasSigma_ << '\n'

              << "** VISION parameters **\n"
              << "linearizationMode_: " << linearizationMode_
              << " HESSIAN, IMPLICIT_SCHUR, JACOBIAN_Q, JACOBIAN_SVD \n"
              << "degeneracyMode_: " << degeneracyMode_
              << " IGNORE_DEGENERACY, ZERO_ON_DEGENERACY, HANDLE_INFINITY \n"
              << "rankTolerance_: " << rankTolerance_ << '\n'
              << "landmarkDistanceThreshold_: " << landmarkDistanceThreshold_
              << '\n'
              << "outlierRejection_: " << outlierRejection_ << '\n'
              << "retriangulationThreshold_: " << retriangulationThreshold_
              << '\n'
              << "addBetweenStereoFactors_: " << addBetweenStereoFactors_
              << '\n'
              << "betweenRotationPrecision_: " << betweenRotationPrecision_
              << '\n'
              << "betweenTranslationPrecision_: "
              << betweenTranslationPrecision_ << '\n'

              << "** OPTIMIZATION parameters **\n"
              << "relinearizeThreshold_: " << relinearizeThreshold_ << '\n'
              << "relinearizeSkip_: " << relinearizeSkip_ << '\n'
              << "zeroVelocitySigma_: " << zeroVelocitySigma_ << '\n'
              << "noMotionPositionSigma_: " << noMotionPositionSigma_ << '\n'
              << "noMotionRotationSigma_: " << noMotionRotationSigma_ << '\n'
              << "constantVelSigma_: " << constantVelSigma_ << '\n'
              << "numOptimize_: " << numOptimize_ << '\n'
              << "horizon_: " << horizon_ << '\n'
              << "useDogLeg_: " << useDogLeg_;
  }

 protected:
  std::shared_ptr<YamlParser> yaml_parser_;
};
typedef std::shared_ptr<VioBackEndParams> VioBackEndParamsPtr;
typedef std::shared_ptr<const VioBackEndParams> VioBackEndParamsConstPtr;

}  // namespace VIO
