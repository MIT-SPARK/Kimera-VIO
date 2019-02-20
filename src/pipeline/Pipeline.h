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
#include <cstdlib> // for srand()
#include <atomic>
#include <memory>
#include <thread>

#include "ETH_parser.h"
#include "LoggerMatlab.h"
#include "FeatureSelector.h"
#include "mesh/Mesher.h"
#include "Visualizer3D.h"
#include "utils/ThreadsafeQueue.h"
#include "StereoImuSyncPacket.h"
#include "pipeline/ProcessControl.h"
#include "pipeline/BufferControl.h"

namespace VIO {
// Forward-declare classes.
class VioBackEndParams;
class VioBackEnd;
class StereoVisionFrontEnd;
}

namespace VIO {
class Pipeline {
public:
  Pipeline(ETHDatasetParser* dataset);

  // Main spin, runs the pipeline.
  bool spin(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Spin the pipeline only once.
  void spinOnce(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // TODO a parallel pipeline should always be able to run sequentially...
  bool spinSequential();

  // Shutdown processing pipeline: stops and joins threads, stops queues.
  // And closes logfiles.
  void shutdown();

  // Return the mesher output queue for FUSES to process the mesh_2d and
  // mesh_3d to extract semantic information.
  ThreadsafeQueue<MesherOutputPayload>& getMesherOutputQueue() {
    return mesher_output_queue_;
  }

  inline void registerSemanticMeshSegmentationCallback(
      Mesher::Mesh3dVizPropertiesSetterCallback cb) {
    semantic_mesh_segmentation_callback_ = cb;
  }

private:
  // Initialize random seed for repeatability (only on the same machine).
  // TODO Still does not make RANSAC REPEATABLE across different machines.
  inline void setDeterministicPipeline() const {
    srand(0);
  }

  // Initialize pipeline.
  bool initialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Initialize backend.
  /// @param: vio_backend: returns the backend initialized.
  /// @param: initial_state_gt: serves as input in case there is ground-truth
  /// available for the initial state and one wants to initialize the backend
  /// using this information. And also as output by returning the eventually
  /// used initial state (either grount-truth, or guessed from imu data).
  bool initBackend(std::unique_ptr<VioBackEnd>* vio_backend,
                   const gtsam::Pose3& B_Pose_camLrect,
                   const gtsam::Cal3_S2& left_undist_rect_cam_mat,
                   const double& baseline,
                   const VioBackEndParams& vio_params,
                   std::shared_ptr<gtNavState>* initial_state_gt,
                   const Timestamp& timestamp_k,
                   const ImuAccGyrS& imu_accgyr);
  // Displaying must be done in the main thread.
  void spinDisplayOnce(
      const std::shared_ptr<VisualizerOutputPayload>& visualizer_output_payload);

  void processKeyframe(
      size_t k,
      const StatusSmartStereoMeasurements& statusSmartStereoMeasurements,
      std::shared_ptr<StereoFrame> last_stereo_keyframe,
      const Timestamp& timestamp_k,
      const Timestamp& timestamp_lkf,
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_accgyr);

  StatusSmartStereoMeasurements featureSelect(
      const VioFrontEndParams& tracker_params,
      const ETHDatasetParser& dataset,
      const Timestamp& timestamp_k,
      const Timestamp& timestamp_lkf,
      const gtsam::Pose3& W_Pose_Blkf,
      double* feature_selection_time,
      std::shared_ptr<StereoFrame>& stereoFrame_km1,
      const StatusSmartStereoMeasurements &smart_stereo_meas,
      int cur_kf_id,
      int save_image_selector,
      const gtsam::Matrix& curr_state_cov,
      const Frame& left_frame);

  // Launch different threads with processes.
  void launchThreads();

  // Shutdown processes and queues.
  void stopThreads();

  // Join threads to do a clean shutdown.
  void joinThreads();

  // Data provider.
  // TODO remove dataset_ from vio pipeline altogether!
  ETHDatasetParser* dataset_;

  Timestamp timestamp_lkf_;

  // Init Vio parameter
  VioBackEndParamsConstPtr backend_params_;
  VioFrontEndParams frontend_params_;

  // TODO this should go to another class to avoid not having copy-ctor...
  // Frontend.
  std::unique_ptr<StereoVisionFrontEnd> stereo_vision_frontend_;
  FeatureSelector feature_selector_;

  // Create VIO: class that implements estimation back-end.
  std::unique_ptr<VioBackEnd> vio_backend_;

  // Thread-safe queue for the backend.
  ThreadsafeQueue<VioBackEndInputPayload> backend_input_queue_;
  ThreadsafeQueue<VioBackEndOutputPayload> backend_output_queue_;

  // Set of planes in the scene.
  std::vector<Plane> planes_;

  // Logger class (stores data for matlab visualization).
  LoggerMatlab logger_;

  // Create class to build mesh.
  Mesher mesher_;

  // Thread-safe queue for the mesher.
  ThreadsafeQueue<MesherInputPayload> mesher_input_queue_;
  ThreadsafeQueue<MesherOutputPayload> mesher_output_queue_;

  // Visualization process.
  Visualizer3D visualizer_;

  // Thread-safe queue for the visualizer.
  ThreadsafeQueue<VisualizerInputPayload> visualizer_input_queue_;
  ThreadsafeQueue<VisualizerOutputPayload> visualizer_output_queue_;

  // High-level abstractions for workflow control.
  ProcessControl process_control_;
  BufferControl buffer_control_;

  // Shutdown switch to stop pipeline, threads, and queues.
  std::atomic_bool shutdown_ = {false};

  // Threads.
  std::thread backend_thread_;
  std::thread mesher_thread_;
  std::thread visualizer_thread_;

  // Callbacks.
  Mesher::Mesh3dVizPropertiesSetterCallback semantic_mesh_segmentation_callback_;
};
} // End of VIO namespace
