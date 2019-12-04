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

#include <gtsam/base/Vector.h>
#include <gtsam/slam/SmartFactorParams.h>

#include <glog/logging.h>

#include "kimera-vio/datasource/DataSource-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

/** \struct Backend Output Params
 * \brief Params controlling what the backend outputs.
 */
struct BackendOutputParams {
 public:
  BackendOutputParams(
      const bool& output_map_lmk_ids_to_3d_points_in_time_horizon,
      const int& min_num_obs_for_lmks_in_time_horizon,
      const bool& output_lmk_id_to_lmk_type_map)
      : output_map_lmk_ids_to_3d_points_in_time_horizon_(
            output_map_lmk_ids_to_3d_points_in_time_horizon),
        min_num_obs_for_lmks_in_time_horizon_(
            min_num_obs_for_lmks_in_time_horizon),
        output_lmk_id_to_lmk_type_map_(output_lmk_id_to_lmk_type_map) {}
  ~BackendOutputParams() = default;

 public:
  //! Whether to output the map from lmk ids to actual lmk 3D positions for
  //! those landmarks that are in the time-horizon of the backend optimization.
  bool output_map_lmk_ids_to_3d_points_in_time_horizon_ = false;
  //! Minimum number of observations for a landmark to be included in the
  //! output of the map from landmark ids to actual landmark 3D positions.
  int min_num_obs_for_lmks_in_time_horizon_ = 4u;
  //! Whether to output as well the type of lmk id (smart, projection, etc).
  //! This is typically used for visualization, to display lmks with different
  //! colors depending on their type.
  bool output_lmk_id_to_lmk_type_map_ = false;
};

class VioBackEndParams {
 public:
  KIMERA_POINTER_TYPEDEFS(VioBackEndParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VioBackEndParams(
      // INITIALIZATION SETTINGS
      const int autoInitialize = 0,
      /// Only used if autoInitialize is off (0)
      const VioNavState& initial_ground_truth_state = VioNavState(),
      const bool roundOnAutoInitialize = false,
      const double initialPositionSigma = 0.00001,
      const double initialRollPitchSigma = 10.0 / 180.0 * M_PI,
      const double initialYawSigma = 0.1 / 180.0 * M_PI,
      const double initialVelocitySigma = 1e-3,
      const double initialAccBiasSigma = 0.1,
      const double initialGyroBiasSigma = 0.01,
      // http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets,
      // the x axis points upwards VISION PARAMS
      const gtsam::LinearizationMode linMode = gtsam::HESSIAN,
      const gtsam::DegeneracyMode degMode = gtsam::ZERO_ON_DEGENERACY,
      const double smartNoiseSigma = 3,
      const double rankTolerance = 1,  // we might also use 0.1
      const double landmarkDistanceThreshold =
          20,  // max distance to triangulate point in meters
      const double outlierRejection =
          8,  // max acceptable reprojection error // before tuning: 3
      const double retriangulationThreshold = 1e-3,
      const bool addBetweenStereoFactors = true,
      const double betweenRotationPrecision = 0.0,  // inverse of variance
      const double betweenTranslationPrecision = 1 /
                                                 (0.1 *
                                                  0.1),  // inverse of variance
      // OPTIMIZATION PARAMS
      const double relinearizeThreshold = 1e-2,  // Before tuning: 1e-3
      const double relinearizeSkip = 1,
      const double zeroVelocitySigma =
          1e-3,  // zero velocity prior when disparity is low
      const double noMotionPositionSigma = 1e-3,
      const double noMotionRotationSigma = 1e-4,
      const double constantVelSigma = 1e-2,
      const size_t numOptimize = 2,
      const double horizon = 6,  // in seconds
      const bool useDogLeg = false)

      : initialPositionSigma_(initialPositionSigma),
        initialRollPitchSigma_(initialRollPitchSigma),
        initialYawSigma_(initialYawSigma),
        initialVelocitySigma_(initialVelocitySigma),
        initialAccBiasSigma_(initialAccBiasSigma),
        initialGyroBiasSigma_(initialGyroBiasSigma),
        autoInitialize_(autoInitialize),
        initial_ground_truth_state_(initial_ground_truth_state),
        roundOnAutoInitialize_(roundOnAutoInitialize),
        linearizationMode_(linMode),
        degeneracyMode_(degMode),
        smartNoiseSigma_(smartNoiseSigma),
        rankTolerance_(rankTolerance),
        landmarkDistanceThreshold_(landmarkDistanceThreshold),
        outlierRejection_(outlierRejection),
        retriangulationThreshold_(retriangulationThreshold),
        addBetweenStereoFactors_(addBetweenStereoFactors),
        betweenRotationPrecision_(betweenRotationPrecision),
        betweenTranslationPrecision_(betweenTranslationPrecision),
        relinearizeThreshold_(relinearizeThreshold),
        relinearizeSkip_(relinearizeSkip),
        horizon_(horizon),
        numOptimize_(numOptimize),
        useDogLeg_(useDogLeg),
        zeroVelocitySigma_(zeroVelocitySigma),
        noMotionPositionSigma_(noMotionPositionSigma),
        noMotionRotationSigma_(noMotionRotationSigma),
        constantVelSigma_(constantVelSigma),
        yaml_parser_(nullptr) {
    // Trivial sanity checks.
    CHECK_GE(horizon, 0);
    CHECK_GE(numOptimize, 0);
  }
  virtual ~VioBackEndParams() = default;

  //! Initialization params
  double initialPositionSigma_;
  double initialRollPitchSigma_;
  double initialYawSigma_;
  double initialVelocitySigma_;
  double initialAccBiasSigma_;
  double initialGyroBiasSigma_;

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
  YamlParser::Ptr yaml_parser_;
};

}  // namespace VIO
