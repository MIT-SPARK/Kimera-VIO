/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEndModule.h
 * @brief  Pipeline module for the backend.
 *
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/backend/RegularVioBackEnd.h"
#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEnd.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class VioBackEndModule
    : public SIMOPipelineModule<BackendInput, BackendOutput> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(VioBackEndModule);
  KIMERA_POINTER_TYPEDEFS(VioBackEndModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SIMO = SIMOPipelineModule<BackendInput, BackendOutput>;
  using InputQueue = ThreadsafeQueue<typename PIO::InputUniquePtr>;

  /**
   * @brief VioBackEndModule
   * @param input_queue
   * @param output_queue
   * @param parallel_run
   * @param vio_backend
   */
  VioBackEndModule(InputQueue* input_queue,
                   bool parallel_run,
                   VioBackEnd::UniquePtr vio_backend)
      : SIMO(input_queue, "VioBackEnd", parallel_run),
        vio_backend_(std::move(vio_backend)) {
    CHECK(vio_backend_);
  }
  virtual ~VioBackEndModule() = default;

  virtual OutputUniquePtr spinOnce(BackendInput::UniquePtr input) {
    CHECK(input);
    return vio_backend_->spinOnce(*input);
  }

 public:
  void initializeBackend(const VioNavStateTimestamped& initial_seed) {
    vio_backend_->initStateAndSetPriors(initial_seed);
  }

  void registerImuBiasUpdateCallback(
      const VioBackEnd::ImuBiasCallback& imu_bias_update_callback) {
    CHECK(vio_backend_);
    vio_backend_->registerImuBiasUpdateCallback(imu_bias_update_callback);
  }

 public:
  // TODO(TONI): REMOVE CALLS BELOW !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Get valid 3D points and corresponding lmk id.
  // Warning! it modifies old_smart_factors_!!
  PointsWithIdMap getMapLmkIdsTo3dPointsInTimeHorizon(
      LmkIdToLmkTypeMap* lmk_id_to_lmk_type_map = nullptr,
      const size_t& min_age = 2) {
    return vio_backend_->getMapLmkIdsTo3dPointsInTimeHorizon(
        lmk_id_to_lmk_type_map, min_age);
  };
  // TODO This call is unsafe, as we are sending a reference to something that
  // will be modified by the backend thread.
  // TODO send this via the output payload...
  inline const gtsam::NonlinearFactorGraph& getFactorsUnsafe() const {
    return vio_backend_->getFactorsUnsafe();
  }

 protected:
  const VioBackEnd::UniquePtr vio_backend_;
};

}  // namespace VIO
