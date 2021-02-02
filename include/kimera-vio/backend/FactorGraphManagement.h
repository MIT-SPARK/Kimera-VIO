/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FactorGraphManagement.h
 * @brief  Factor graph management utilities.
 *
 * @author Antoni Rosinol
 */

#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_map>

#include <boost/foreach.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/backend/VioBackendParams.h"
#include "kimera-vio/factors/PointPlaneFactor.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/UtilsGTSAM.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace VIO {

/**
 * @brief The ValueManager class Manages the values used for factor graph
 * optimization
 */
class ValueManager {
 public:
  KIMERA_POINTER_TYPEDEFS(ValueManager);
  KIMERA_DELETE_COPY_CONSTRUCTORS(ValueManager);

 public:
  ValueManager(const SymbolChar& symbol_pose_key,
               const SymbolChar& symbol_velocity_key,
               const SymbolChar& symbol_imu_bias_key)
      : kSymbolPoseKey(symbol_pose_key),
        kSymbolVelocityKey(symbol_velocity_key),
        kSymbolImuBiasKey(symbol_imu_bias_key) {}
  virtual ~ValueManager() = default;

 public:
  gtsam::Values getValues() const { return values_; }
  void clearValues() { values_.clear(); }

 public:
  /**
   * @brief checkValuesWithGraph Checks that the encoded values' symbols
   * are present in the provided factor graph.
   * @return True if values are consistent with graph, false otherwise.
   */
  bool checkValuesConsistencyWithGraph(
      const gtsam::NonlinearFactorGraph& graph) const {
    for (const gtsam::Values::const_iterator& key_value = values_.begin();
         key_value != values_.end();
         ++key_value) {
      if (!graph.exists(key_value->key)) {
        return false;
      }
    }
    return true;
  }

 public:
  /**
   * @brief addVioNavStateValues Convenience function to add all the values
   * of a VioNavState: pose, velocity, imu bias.
   * @param curr_kf_id_
   * @param W_Pose_B_lkf_
   * @param W_Pose_B_lkf_
   * @param imu_bias
   */
  void addVioNavStateValues(const FrameId& curr_kf_id,
                            const gtsam::Pose3& W_Pose_B_lkf,
                            const gtsam::Pose3& W_Vel_B_lkf,
                            const ImuBias& imu_bias) {
    CHECK_GT(curr_kf_id, 0u);
    addPoseValue(curr_kf_id, W_Pose_B_lkf);
    addVelocityValue(curr_kf_id, W_Vel_B_lkf);
    addImuValue(curr_kf_id, imu_bias);
  }

  /**
   * @brief addPoseValue Add initial guess for the pose value.
   * @param frame_id Id used for the pose variable in the factor graph.
   * @param pose Initial guess of the 3D pose.
   */
  void addPoseValue(const FrameId& frame_id, const gtsam::Pose3& pose) {
    gtsam::Symbol key(kSymbolPoseKey, frame_id);
    CHECK(!values_.exists(key));
    values_.insert(key, pose);
  }
  /**
   * @brief addVelocityValue Add initial guess for the velocity
   * @param frame_id Id used for the velocity variable
   * @param velocity value (W_Vel_B_lkf_).
   */
  void addVelocityValue(const FrameId& frame_id,
                        const gtsam::Vector3& velocity) {
    gtsam::Symbol key(kSymbolVelocityKey, frame_id);
    CHECK(!values_.exists(key));
    values_.insert(key, velocity);
  }
  /**
   * @brief addImuValue Add actual value for initial guess of the IMU bias.
   * @param frame_id Id used for the Bias variable.
   * @param imu_bias Initial guess of the bias.
   */
  void addImuValue(const FrameId& frame_id, const ImuBias& imu_bias) {
    gtsam::Symbol key(kSymbolImuBiasKey, frame_id);
    CHECK(!values_.exists(key));
    values_.insert(key, imu_bias);
  }

 protected:
  const unsigned char kSymbolPoseKey;
  const unsigned char kSymbolVelocityKey;
  const unsigned char kSymbolImuBiasKey;

 protected:
  //! Actual values that are being managed.
  gtsam::Values values_;
};

class FactorGraphManager {
 public:
  KIMERA_POINTER_TYPEDEFS(FactorGraphManager);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FactorGraphManager);

 public:
  FactorGraphManager(const SymbolChar& symbol_pose_key,
                     const SymbolChar& symbol_velocity_key,
                     const SymbolChar& symbol_imu_bias_key)
      : kSymbolPoseKey(symbol_pose_key),
        kSymbolVelocityKey(symbol_velocity_key),
        kSymbolImuBiasKey(symbol_imu_bias_key) {}
  virtual ~FactorGraphManager() = default;

 public:
  gtsam::NonlinearFactorGraph getGraph() const {
    // Clear graph
    graph_.resize(0);
    // Concat smart and imu factors
    graph_.push_back(smart_factors_.begin(), smart_factors_.end());
    graph_.push_back(imu_factors_.begin(), imu_factors_.end());
    graph_.push_back(between_factors_.begin(), between_factors_.end());
    return graph_;
  }
  // Don't forget to clear the factors_to_delete after the ISAM2 update.
  gtsam::FactorIndices getFactorIndicesToDelete() const {
    return smart_factors_to_delete_;
  }

 public:
  /**
   * @brief addImuFactor
   * @param from_id
   * @param to_id
   * @param pim of CombinedImuFactor type
   */
  void addImuFactor(const FrameId& from_id,
                    const FrameId& to_id,
                    const gtsam::PreintegratedCombinedMeasurements& pim) {
    imu_factors_.push_back(boost::make_shared<gtsam::CombinedImuFactor>(
        gtsam::Symbol(kSymbolPoseKey, from_id),
        gtsam::Symbol(kSymbolVelocityKey, from_id),
        gtsam::Symbol(kSymbolPoseKey, to_id),
        gtsam::Symbol(kSymbolVelocityKey, to_id),
        gtsam::Symbol(kSymbolImuBiasKey, from_id),
        gtsam::Symbol(kSymbolImuBiasKey, to_id),
        pim));
  }

  /**
   * @brief addImuFactor
   * @param from_id
   * @param to_id
   * @param pim of PreintegratedImuMeasurements type.
   */
  void addImuFactor(const FrameId& from_id,
                    const FrameId& to_id,
                    const gtsam::PreintegratedImuMeasurements& pim) {
    imu_factors_.push_back(
        boost::make_shared<gtsam::PreintegratedImuMeasurements>(
            gtsam::Symbol(kSymbolPoseKey, from_id),
            gtsam::Symbol(kSymbolVelocityKey, from_id),
            gtsam::Symbol(kSymbolPoseKey, to_id),
            gtsam::Symbol(kSymbolVelocityKey, to_id),
            gtsam::Symbol(kSymbolImuBiasKey, from_id),
            pim));
  }

  void addConstantImuBiasFactor(
      const FrameId& from_id,
      const FrameId& to_id,
      const gtsam::imuBias::ConstantBias& imu_bias,
      const gtsam::SharedNoiseModel& bias_noise_model) {
    imu_factors_.push_back(
        boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            gtsam::Symbol('b', from_id),
            gtsam::Symbol('b', to_id),
            zero_bias,
            bias_noise_model));
  }

  /**
   * @brief addSmartFactor
   */
  void addSmartFactor() {
    // Add factor without slot if new.

    // Add factor with slot if not new.
  }

  /**
   * @brief makeSmartFactorsConsistentWithSmoother Smart factors are painful.
   */
  bool checkSmartFactorsConsistencyWithSmoother(
      const gtsam::NonlinearFactor& smoother_factor_graph) {
    for (const auto& factor : smart_factors_) {
      const auto& smart_factor =
          boost::dynamic_pointer_cast<SmartStereoFactor>(factor);
      for (const gtsam::Key& key : smart_factor->keys()) {
        if (!smoother_factor_graph.exists(key)) {
          VLOG(10) << "Factor with lmk id " << lmk_id
                   << " is linking to a marginalized state!";
          return false;  // We cannot modify the smart factor??
        }
      };
    }
  }

  void updateSmartFactorsSlots(
      const gtsam::FactorIndices& new_factors_indices) {
    // Since the first added factors where the smart ones, and since ISAM2 keeps
    // a 1-to-1 correspondence between newFactorsIndices and the new factors
    // passed, we can simply iterate over the number of smart factors and update
    // the delete slot ids.
    for (size_t i = 0u; i < smart_factors_.size(); i++) {
      CHECK_LT(i, new_factors_indices.size())
          << "There are more smart factors than factors in the graph!";
      smart_factors_to_delete_.push_back(new_factors_indices.at(i));
    }
  }

  /**
   * @brief addZeroVelocityPrior Add no motion factors in case of low disparity.
   * @param frame_id
   */
  void addZeroVelocityFactor(const FrameId& frame_id) {}

  /**
   * @brief addNoMotionFactor
   * @param from_id
   * @param to_id
   */
  void addNoMotionFactor(const FrameId& from_id, const FrameId& to_id) {}

  /**
   * @brief addBetweenFactor
   * @param from_id
   * @param to_id
   * @param from_id_POSE_to_id
   */
  void addBetweenFactor(const FrameId& from_id,
                        const FrameId& to_id,
                        const gtsam::Pose3& from_id_POSE_to_id,
                        const gtsam::SharedNoiseModel& noise_model) {
    between_factors_.push_back(
        boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            gtsam::Symbol('x', from_id),
            gtsam::Symbol('x', to_id),
            from_id_POSE_to_id,
            noise_model));
  }

  // Cleaners
  /**
   * @brief cleanGraph Cleans the underlying factor graph, to be called after
   * each iteration. Mind that this does not clean the factors_to_delete_
   * since these might be still needed in the following iterations.
   */
  void cleanGraph() {
    between_factors_.clear();
    imu_factors_.clear();
    smart_factors_.clear();
    graph_.clear();
  }

  /**
   * @brief cleanFactorsToDeleteIndices Cleans the books containing the indices
   * of the smart factors to be removed in the graph. You should call this after
   * an ISAM2 update.
   */
  void cleanFactorsToDeleteIndices() { smart_factors_to_delete_.clear(); }

 protected:
  /**
   * @brief bookkeepSmartFactors
   * Updates the smart factors with their slots in the factor graph, for easy
   * removing from the factor graph in next iteration.
   */
  void bookkeepSmartFactors() {}

 protected:
  const SymbolChar kSymbolPoseKey;
  const SymbolChar kSymbolVelocityKey;
  const SymbolChar kSymbolImuBiasKey;

 protected:
  //! Actual graph to be added to the optimization problem.
  gtsam::NonlinearFactorGraph graph_;
  //! List of indices to keep track of which smart_factors are to be deleted
  //! before next optimization.
  gtsam::FactorIndices smart_factors_to_delete_;

 private:
  gtsam::NonlinearFactorGraph between_factors_;
  gtsam::NonlinearFactorGraph imu_factors_;
  gtsam::NonlinearFactorGraph smart_factors_;
};

/**
 * @brief The FactorGraphBuilder class Ensures that both graph_manager_ and
 * value_manager_ are coherent.
 */
class FactorGraphBuilder {
 public:
  KIMERA_POINTER_TYPEDEFS(FactorGraphBuilder);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FactorGraphBuilder);

 public:
  FactorGraphBuilder()
      : graph_manager_(kSymbolPoseKey, kSymbolVelocityKey, kSymbolImuBiasKey),
        value_manager_(kSymbolPoseKey, kSymbolVelocityKey, kSymbolImuBiasKey) {}
  virtual ~FactorGraphBuilder() = default;

  FactorGraphManager& getGraphManager() { return graph_manager_; }
  ValueManager& getValueManager() { return value_manager_; }

  /**
   * @brief checkValueGraphCoherence Checks that value and factor graph are
   * consistent.
   * @return
   */
  bool checkValueGraphCoherence() {
    return value_manager_.checkValuesConsistencyWithGraph(
        graph_manager_.getGraph());
  }

 protected:
  //! Symbols used in the factor graph.
  static constexpr unsigned char kSymbolPoseKey = 'x';
  static constexpr unsigned char kSymbolVelocityKey = 'v';
  static constexpr unsigned char kSymbolImuBiasKey = 'b';

 protected:
  FactorGraphManager graph_manager_;
  ValueManager value_manager_;
};

}  // namespace VIO
