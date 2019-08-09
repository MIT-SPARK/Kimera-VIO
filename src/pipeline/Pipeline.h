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

#include <gtsam/navigation/ImuBias.h>

#include "FeatureSelector.h"
#include "LoggerMatlab.h"
#include "StereoImuSyncPacket.h"
#include "Visualizer3D.h"
#include "datasource/DataSource.h"
#include "initial/InitializationBackEnd-definitions.h"
#include "mesh/Mesher.h"
#include "pipeline/BufferControl.h"
#include "pipeline/ProcessControl.h"
#include "utils/ThreadsafeQueue.h"

namespace VIO {
// Forward-declare classes.
class VioBackEndParams;
class VioBackEnd;
class StereoVisionFrontEnd;
class LoopClosureDetectorParams;
class LoopClosureDetector;

}  // namespace VIO

namespace VIO {

class Pipeline {
 private:
  // Typedefs
  typedef std::function<void(const SpinOutputPacket&)>
      KeyframeRateOutputCallback;

 public:
  Pipeline(const PipelineParams& params, bool parallel_run = true);

  ~Pipeline();

  // Main spin, runs the pipeline.
  void spin(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Run an endless loop until shutdown to visualize.
  bool spinViz(bool parallel_run = true);

  // Spin the pipeline only once.
  void spinOnce(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // A parallel pipeline should always be able to run sequentially...
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
  // TODO(Toni) this should be a callback instead...
  // right now it works because no one is pulling from this queue in pipeline.
  inline ThreadsafeQueue<MesherOutputPayload>& getMesherOutputQueue() {
    return mesher_output_queue_;
  }

  // Registration of callbacks.
  // Callback to modify the mesh visual properties every time the mesher
  // has a new 3d mesh.
  inline void registerSemanticMeshSegmentationCallback(
      Mesher::Mesh3dVizPropertiesSetterCallback cb) {
    visualizer_.registerMesh3dVizProperties(cb);
  }

  // Callback to output the VIO backend results at keyframe rate.
  // This callback also allows to
  inline void registerKeyFrameRateOutputCallback(
      KeyframeRateOutputCallback callback) {
    keyframe_rate_output_callback_ = callback;
  }

 private:
  // Initialize random seed for repeatability (only on the same machine).
  // TODO Still does not make RANSAC REPEATABLE across different machines.
  inline void setDeterministicPipeline() const { srand(0); }

  // Initialize pipeline with desired option (flag).
  bool initialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Check if necessary to re-initialize pipeline.
  void checkReInitialize(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Initialize pipeline from IMU or GT.
  bool initializeFromIMUorGT(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Initialize pipeline from online gravity alignment.
  bool initializeOnline(const StereoImuSyncPacket& stereo_imu_sync_packet);

  // Initialize backend given external pose estimate (GT or OGA)
  // TODO(Sandro): Unify both functions below (init backend)
  bool initializeVioBackend(const StereoImuSyncPacket& stereo_imu_sync_packet,
                            std::shared_ptr<gtNavState> initial_state,
                            const StereoFrame& stereo_frame_lkf);

  // Initialize backend.
  /// @param: vio_backend: returns the backend initialized.
  /// @param: initial_state_gt: serves as input in case there is ground-truth
  /// available for the initial state and one wants to initialize the backend
  /// using this information. And also as output by returning the eventually
  /// used initial state (either grount-truth, or guessed from imu data).
  bool initBackend(std::unique_ptr<VioBackEnd>* vio_backend,
                   const gtsam::Pose3& B_Pose_camLrect,
                   const gtsam::Cal3_S2& left_undist_rect_cam_mat,
                   const double& baseline, const VioBackEndParams& vio_params,
                   std::shared_ptr<gtNavState>* initial_state_gt,
                   const Timestamp& timestamp_k, const ImuAccGyrS& imu_accgyr);
  // Displaying must be done in the main thread.
  void spinDisplayOnce(VisualizerOutputPayload& visualizer_output_payload);

  void processKeyframe(
      const StatusSmartStereoMeasurements& statusSmartStereoMeasurements,
      const StereoFrame& last_stereo_keyframe,
      const ImuFrontEnd::PreintegratedImuMeasurements& pim,
      const TrackingStatus& kf_tracking_status_stereo,
      const gtsam::Pose3& relative_pose_body_stereo,
      const DebugTrackerInfo& debug_tracker_info);

  void processKeyframePop();

  StatusSmartStereoMeasurements featureSelect(
      const VioFrontEndParams& tracker_params, const Timestamp& timestamp_k,
      const Timestamp& timestamp_lkf, const gtsam::Pose3& W_Pose_Blkf,
      double* feature_selection_time,
      std::shared_ptr<StereoFrame>& stereoFrame_km1,
      const StatusSmartStereoMeasurements& smart_stereo_meas, int cur_kf_id,
      int save_image_selector, const gtsam::Matrix& curr_state_cov,
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

  // Callbacks.
  KeyframeRateOutputCallback keyframe_rate_output_callback_;

  // Init Vio parameter
  VioBackEndParamsConstPtr backend_params_;
  VioFrontEndParams frontend_params_;

  // TODO this should go to another class to avoid not having copy-ctor...
  // Frontend.
  std::unique_ptr<StereoVisionFrontEnd> vio_frontend_;
  std::unique_ptr<FeatureSelector> feature_selector_;

  // Stereo vision frontend payloads.
  ThreadsafeQueue<StereoImuSyncPacket> stereo_frontend_input_queue_;
  ThreadsafeQueue<StereoFrontEndOutputPayload> stereo_frontend_output_queue_;

  // Online initialization frontend queue.
  ThreadsafeQueue<InitializationInputPayload>
      initialization_frontend_output_queue_;

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

  // Create class to detect loop closures.
  std::unique_ptr<LoopClosureDetector> loop_closure_detector_;
  LoopClosureDetectorParams lcd_params_;

  // Thread-safe queue for the loop closure detector.
  ThreadsafeQueue<LoopClosureDetectorInputPayload> lcd_input_queue_;
  ThreadsafeQueue<LoopClosureDetectorOutputPayload> lcd_output_queue_;

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
  std::unique_ptr<std::thread> lcd_thread_ = {nullptr};
  // std::thread visualizer_thread_;

  int backend_type_;
  bool parallel_run_;
};

}  // namespace VIO
