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
  Pipeline(ETHDatasetParser* dataset,
           const ImuParams& imu_params,
           bool parallel_run = true);

  ~Pipeline();

  // Main spin, runs the pipeline.
  SpinOutputContainer spin(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Run an endless loop until shutdown to visualize.
  void spinViz(bool parallel_run = true);

  // Spin the pipeline only once.
  void spinOnce(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // TODO a parallel pipeline should always be able to run sequentially...
  void spinSequential();

  // Shutdown the pipeline once all data has been consumed.
  void shutdownWhenFinished();

  // Shutdown processing pipeline: stops and joins threads, stops queues.
  // And closes logfiles.
  void shutdown();

  // Resumes all queues
  void resume();

  // Return the mesher output queue for FUSES to process the mesh_2d and
  // mesh_3d to extract semantic information.
  ThreadsafeQueue<MesherOutputPayload>& getMesherOutputQueue() {
    return mesher_output_queue_;
  }

  gtsam::Vector3 getEstimatedVelocity() {
    return vio_backend_->getWVelBLkf();
  }

  gtsam::Pose3 getEstimatedPose() {
    return vio_backend_->getWPoseBLkf();
  }

  ImuBias getEstimatedBias() {
    return vio_backend_->getLatestImuBias();
  }

  Timestamp getTimestamp() {
    return vio_backend_->getTimestampLkf();
  }

  gtsam::Matrix getEstimatedStateCovariance() {
    return vio_backend_->getStateCovarianceLkf();
  }

  DebugTrackerInfo getTrackerInfo() {
    return debug_tracker_info_;
  }

  inline void registerSemanticMeshSegmentationCallback(
      Mesher::Mesh3dVizPropertiesSetterCallback cb) {
    visualizer_.registerMesh3dVizProperties(cb);
  }

  SpinOutputContainer getSpinOutputContainer();

private:
  // Initialize random seed for repeatability (only on the same machine).
  // TODO Still does not make RANSAC REPEATABLE across different machines.
  inline void setDeterministicPipeline() const {
    srand(0);
  }

  // Initialize pipeline with desired option (flag).
  bool initialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Initialize pipeline from IMU or GT.
  bool initializeFromIMUorGT(const StereoImuSyncPacket &stereo_imu_sync_packet);

  // Initialize pipeline from online gravity alignment.
  bool initializeOnline(const StereoImuSyncPacket &stereo_imu_sync_packet);

  // Perform Bundle-Adjustment and initial gravity alignment
  bool bundleAdjustmentAndGravityAlignment(
                      const StereoImuSyncPacket &stereo_imu_sync_init,
                      const StereoFrame &stereo_frame_lkf,
                      gtsam::Vector3 *gyro_bias,
                      gtsam::Vector3 *g_iter_b0,
                      gtsam::NavState *init_navstate);

  // Initialize backend given external pose estimate (GT or OGA)
  bool initializeBackend(const StereoImuSyncPacket &stereo_imu_sync_packet,
                      std::shared_ptr<gtNavState> initial_state,
                      const StereoFrame &stereo_frame_lkf);

  // Re-initialize pipeline.
  bool reInitialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

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
  void spinDisplayOnce(VisualizerOutputPayload& visualizer_output_payload);

  void processKeyframe(
      const StatusSmartStereoMeasurements& statusSmartStereoMeasurements,
      const StereoFrame &last_stereo_keyframe,
      const ImuFrontEnd::PreintegratedImuMeasurements& pim,
      const TrackingStatus& kf_tracking_status_stereo,
      const gtsam::Pose3& relative_pose_body_stereo);

  void processKeyframePop();

  void pushToMesherInputQueue(
      VioBackEnd::PointsWithIdMap* points_with_id_VIO,
      VioBackEnd::LmkIdToLmkTypeMap* lmk_id_to_lmk_type_map,
      const StereoFrame& last_stereo_keyframe);

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

  // Launch frontend thread with process.
  void launchFrontendThread();

  // Launch remaining threads with processes.
  void launchRemainingThreads();

  // Shutdown processes and queues.
  void stopThreads();

  // Join threads to do a clean shutdown.
  void joinThreads();

  // Data provider.
  // TODO remove dataset_ from vio pipeline altogether!
  ETHDatasetParser* dataset_;

  // For bookkeeping
  Timestamp timestamp_lkf_;
  Timestamp timestamp_lkf_published_;

  // Init Vio parameter
  VioBackEndParamsConstPtr backend_params_;
  VioFrontEndParams frontend_params_;

  // TODO this should go to another class to avoid not having copy-ctor...
  // Frontend.
  std::unique_ptr<StereoVisionFrontEnd> vio_frontend_;
  FeatureSelector feature_selector_;

  // Debug information from frontend (this is the only going out)
  DebugTrackerInfo debug_tracker_info_;

  // Stereo vision frontend payloads.
  ThreadsafeQueue<StereoImuSyncPacket> stereo_frontend_input_queue_;
  ThreadsafeQueue<StereoFrontEndOutputPayload> stereo_frontend_output_queue_;

  // Online initialization frontend queue.
  ThreadsafeQueue<StereoFrontEndOutputPayload> initialization_frontend_output_queue_;

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
  std::atomic_bool is_initialized_ = {false};
  std::atomic_bool is_launched_ = {false};
  int init_frame_id_;

  // Threads.
  std::unique_ptr<std::thread> stereo_frontend_thread_ = {nullptr};
  std::unique_ptr<std::thread> wrapped_thread_ = {nullptr};
  std::unique_ptr<std::thread> backend_thread_ = {nullptr};
  std::unique_ptr<std::thread> mesher_thread_ = {nullptr};
  //std::thread visualizer_thread_;

  bool parallel_run_;
};

} // End of VIO namespace
