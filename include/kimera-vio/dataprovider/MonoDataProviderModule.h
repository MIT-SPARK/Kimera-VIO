/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoDataProviderModule.h
 * @brief  Pipeline module that provides data to the VIO pipeline.
 * @details Collects camera and IMU data, publishes monocular images via
 * callback
 *          getInputPacket processes one monocular images at a time, attempting
 * to
 *          gather IMU data between the current monocular images and the
 * previous
 *          monocular images.
 * output_queue is unused-- the resulting bundle (IMU + monocular images, called
 * a
 *          monocular imagesImuSyncPacket) is published via
 * registerVioPipelineCallback.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>
#include <utility>  // for move

#include <glog/logging.h>

#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/frontend/MonoImuSyncPacket.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class MonoDataProviderModule
    : public DataProviderModule<MonoImuSyncPacket, MonoImuSyncPacket> {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoDataProviderModule);
  KIMERA_POINTER_TYPEDEFS(MonoDataProviderModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using MonoVioPipelineCallback =
      std::function<void(MonoImuSyncPacket::UniquePtr)>;

  MonoDataProviderModule(OutputQueue* output_queue,
                         const std::string& name_id,
                         const bool& parallel_run);

  virtual ~MonoDataProviderModule() = default;

  inline OutputUniquePtr spinOnce(MonoImuSyncPacket::UniquePtr input) override {
    // Called by spin(), which also calls getInputPacket().
    // Data provider syncs and publishes input sensor information, which
    // is done at the level of getInputPacket. No other action needed.
    return input;
  }

  // TODO(Toni): remove, register at ctor level.
  inline void registerVioPipelineCallback(const MonoVioPipelineCallback& cb) {
    vio_pipeline_callback_ = cb;
  }

 protected:
  // Spin the dataset: processes the input data and constructs a Mono+Imu
  // Synchronized Packet (mono + IMU measurements), the minimum data
  // needed for the VIO pipeline to do one processing iteration.
  // Any image that appear before the first IMU packet will be discarded.
  // If an image appears after another image with no IMU packets in
  // between, it will be discarded.
  // The first valid pair is used as a timing fencepost and is not published.
  InputUniquePtr getInputPacket() override;

 private:
  MonoVioPipelineCallback vio_pipeline_callback_;
};

}  // namespace VIO
