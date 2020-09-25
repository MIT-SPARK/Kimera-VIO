/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoPipeline.h
 * @brief  Implements StereoVIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndModule.h"
#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/StereoDataProviderModule.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/mesh/MesherModule.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DModule.h"

namespace VIO {

class StereoPipeline : public Pipeline<StereoImuSyncPacket, StereoFrontendOutput> {
 public:
  KIMERA_POINTER_TYPEDEFS(StereoPipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /**
     * @brief StereoPipeline
     * @param params Vio parameters
     * @param visualizer Optional visualizer for visualizing 3D results
     * @param displayer Optional displayer for visualizing 2D results
     */
  StereoPipeline(const VioParams& params,
                 Visualizer3D::UniquePtr&& visualizer = nullptr,
                 DisplayBase::UniquePtr&& displayer = nullptr);

  virtual ~StereoPipeline();

 public:
  bool spin() override {
    // Feed data to the pipeline
    CHECK(data_provider_module_);
    LOG(INFO) << "Spinning Kimera-VIO.";
    return data_provider_module_->spin();
  }

  bool shutdownWhenFinished(const int& sleep_time_ms,
                            const bool& print_stats = false) override;

  //! Callbacks to fill stereo frames
  inline void fillLeftFrameQueue(Frame::UniquePtr left_frame) {
    CHECK(data_provider_module_);
    CHECK(left_frame);
    data_provider_module_->fillLeftFrameQueue(std::move(left_frame));
  }
  inline void fillRightFrameQueue(Frame::UniquePtr right_frame) {
    CHECK(data_provider_module_);
    CHECK(right_frame);
    data_provider_module_->fillRightFrameQueue(std::move(right_frame));
  }
  //! Callbacks to fill queues but they block if queues are getting full.
  //! Useful when parsing datasets, don't use with real sensors.
  inline void fillLeftFrameQueueBlockingIfFull(Frame::UniquePtr left_frame) {
    CHECK(data_provider_module_);
    CHECK(left_frame);
    data_provider_module_->fillLeftFrameQueueBlockingIfFull(
        std::move(left_frame));
  }
  inline void fillRightFrameQueueBlockingIfFull(Frame::UniquePtr right_frame) {
    CHECK(data_provider_module_);
    CHECK(right_frame);
    data_provider_module_->fillRightFrameQueueBlockingIfFull(
        std::move(right_frame));
  }
  //! Fill one IMU measurement at a time.
  inline void fillSingleImuQueue(const ImuMeasurement& imu_measurement) {
    CHECK(data_provider_module_);
    data_provider_module_->fillImuQueue(imu_measurement);
  }
  //! Fill multiple IMU measurements in batch
  inline void fillMultiImuQueue(const ImuMeasurements& imu_measurements) {
    CHECK(data_provider_module_);
    data_provider_module_->fillImuQueue(imu_measurements);
  }

  void shutdown() override;

 protected:
  void spinSequential() override;

 protected:
  //! Definition of sensor rig used
  StereoCamera::ConstPtr stereo_camera_;

  //! Specified data provider module
  StereoDataProviderModule::UniquePtr data_provider_module_;
};

}  // namespace VIO
