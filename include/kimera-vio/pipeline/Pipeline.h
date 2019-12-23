/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline.h
 * @brief  Implements VIO pipeline workflow.
 * @author Antoni Rosinol
 */

#pragma once

#include <stddef.h>
#include <atomic>
#include <cstdlib>  // for srand()
#include <memory>
#include <thread>
#include <utility>  // for make_pair
#include <vector>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndModule.h"
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/frontend/FeatureSelector.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/VisionFrontEndModule.h"
#include "kimera-vio/initial/InitializationBackEnd-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/mesh/MesherModule.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/visualizer/Visualizer3DModule.h"

namespace VIO {

class Pipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(Pipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Pipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  explicit Pipeline(const VioParams& params);

  virtual ~Pipeline();

  //! Callbacks to fill input queues.
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

  // Run an endless loop until shutdown to visualize.
  bool spinViz();

  // Shutdown the pipeline once all data has been consumed.
  bool shutdownWhenFinished();

  // Shutdown processing pipeline: stops and joins threads, stops queues.
  // And closes logfiles.
  void shutdown();

  // Resumes all queues
  void resume();

  // Register external callback to output the VIO backend results.
  inline void registerBackendOutputCallback(
      const VioBackEndModule::OutputCallback& callback) {
    CHECK(vio_backend_module_);
    vio_backend_module_->registerCallback(callback);
  }

  // Register external callback to output the VIO frontend results.
  // TODO(marcus): once we have a base class for StereoVisionFrontend, we need 
  // that type to go here instead.
  inline void registerFrontendOutputCallback(
      const StereoVisionFrontEndModule::OutputCallback& callback) {
    CHECK(vio_frontend_module_);
    vio_frontend_module_->registerCallback(callback);
  }

  // Register external callback to output mesher results.
  inline void registerMesherOutputCallback(
      const MesherModule::OutputCallback& callback) {
    CHECK(mesher_module_);
    mesher_module_->registerCallback(callback);
  }

  // Register external callback to output the LoopClosureDetector's results.
  inline void registerLcdOutputCallback(
      const LcdModule::OutputCallback& callback) {
    if (lcd_module_) {
      lcd_module_->registerCallback(callback);
    } else {
      LOG(ERROR) << "Attempt to register LCD/PGO callback, but no "
                 << "LoopClosureDetector member is active in pipeline.";
    }
  }

  /**
   * @brief spin Spin the whole pipeline by spinning the data provider
   * If in sequential mode, it will return for each spin.
   * If in parallel mode, it will not return until the pipeline is shutdown.
   * @return True if everything goes well.
   */
  bool spin() {
    // Feed data to the pipeline
    CHECK(data_provider_module_);
    return data_provider_module_->spin();
  }

 private:
  // Spin the pipeline only once.
  void spinOnce(StereoImuSyncPacket::UniquePtr stereo_imu_sync_packet);

  // A parallel pipeline should always be able to run sequentially...
  void spinSequential();

 private:
  // Initialize random seed for repeatability (only on the same machine).
  // TODO Still does not make RANSAC REPEATABLE across different machines.
  inline void setDeterministicPipeline() const { srand(0); }

  // Initialize pipeline with desired option (flag).
  bool initialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Check if necessary to re-initialize pipeline.
  void checkReInitialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Initialize pipeline from ground truth pose.
  bool initializeFromGroundTruth(
      const StereoImuSyncPacket& stereo_imu_sync_packet,
      const VioNavState& initial_ground_truth_state);

  // Initialize pipeline from IMU readings only:
  //  - Guesses initial state assuming zero velocity.
  //  - Guesses IMU bias assuming steady upright vehicle.
  bool initializeFromIMU(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Initialize pipeline from online gravity alignment.
  bool initializeOnline(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Displaying must be done in the main thread.
  void spinDisplayOnce(const VisualizerOutput::Ptr& viz_output) const;

  StatusStereoMeasurements featureSelect(
      const VioFrontEndParams& tracker_params,
      const FeatureSelectorParams& feature_selector_params,
      const Timestamp& timestamp_k,
      const Timestamp& timestamp_lkf,
      const gtsam::Pose3& W_Pose_Blkf,
      double* feature_selection_time,
      std::shared_ptr<StereoFrame>& stereoFrame_km1,
      const StatusStereoMeasurements& smart_stereo_meas,
      int cur_kf_id,
      int save_image_selector,
      const gtsam::Matrix& curr_state_cov,
      const Frame& left_frame);

  // Launch different threads with processes.
  void launchThreads();

  // Launch frontend thread with process.
  void launchFrontendThread();

  // Launch remaining threads with processes.
  void launchRemainingThreads();

  // Shutdown processes and queues.
  void stopThreads();

  // Join threads to do a clean shutdown.
  void joinThreads();

  // Init Vio parameter
  VioBackEndParams::ConstPtr backend_params_;
  VioFrontEndParams frontend_params_;
  ImuParams imu_params_;
  BackendType backend_type_;
  bool parallel_run_;

  //! Definition of sensor rig used
  StereoCamera::UniquePtr stereo_camera_;

  //! Data provider.
  DataProviderModule::UniquePtr data_provider_module_;

  // TODO this should go to another class to avoid not having copy-ctor...
  //! Frontend.
  StereoVisionFrontEndModule::UniquePtr vio_frontend_module_;
  std::unique_ptr<FeatureSelector> feature_selector_;

  //! Stereo vision frontend payloads.
  StereoVisionFrontEndModule::InputQueue stereo_frontend_input_queue_;

  // Online initialization frontend queue.
  ThreadsafeQueue<InitializationInputPayload::UniquePtr>
      initialization_frontend_output_queue_;

  //! Backend
  VioBackEndModule::UniquePtr vio_backend_module_;

  //! Thread-safe queue for the backend.
  VioBackEndModule::InputQueue backend_input_queue_;

  //! Mesher
  MesherModule::UniquePtr mesher_module_;

  //! Loop Closure Detector
  LcdModule::UniquePtr lcd_module_;

  //! Visualizer
  VisualizerModule::UniquePtr visualizer_module_;

  // Shutdown switch to stop pipeline, threads, and queues.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_initialized_ = {false};
  std::atomic_bool is_launched_ = {false};

  // TODO(Toni): Remove this?
  int init_frame_id_;

  //! Threads.
  std::unique_ptr<std::thread> frontend_thread_ = {nullptr};
  std::unique_ptr<std::thread> backend_thread_ = {nullptr};
  std::unique_ptr<std::thread> mesher_thread_ = {nullptr};
  std::unique_ptr<std::thread> lcd_thread_ = {nullptr};
  std::unique_ptr<std::thread> visualizer_thread_ = {nullptr};

};

}  // namespace VIO
