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
  Pipeline();

  // Main spin, runs the pipeline.
  bool spin();

  // Spin the pipeline only once.
  void spinOnce(size_t k);

  // TODO a parallel pipeline should always be able to run sequentially...
  bool spinSequential();

  // Shutdown processing pipeline: stops and joins threads, stops queues.
  // And closes logfiles.
  void shutdown();

private:
  // Initialize random seed for repeatability (only on the same machine).
  // TODO Still does not make RANSAC REPEATABLE across different machines.
  inline void setDeterministicPipeline() const {
    srand(0);
  }

  // Decides backend parameters depending on the backend chosen.
  // 0: Vanilla VIO 1: regularVIO
  void setBackendType(int backend_type,
                      std::shared_ptr<VioBackEndParams>* vioParams) const;

  // Initialize pipeline.
  bool initialize(size_t k);

  // Initialize frontend.
  bool initFrontend(const Timestamp& timestamp_lkf,
                    const Timestamp& timestamp_k,
                    StereoFrame* stereoFrame_k,
                    StereoVisionFrontEnd* stereo_vision_frontend,
                    ImuFrontEnd* imu_buffer,
                    ImuStamps* imu_stamps, ImuAccGyr* imu_accgyr) const;
  bool initStereoFrontend(StereoFrame* stereo_frame_k,
                          StereoVisionFrontEnd* stereo_vision_frontend) const;
  bool initImuFrontend(const Timestamp& timestamp_lkf,
                       const Timestamp& timestamp_k,
                       ImuFrontEnd* imu_buffer,
                       ImuStamps* imu_stamps, ImuAccGyr* imu_accgyr) const;
  // Initialize backend.
  /// @param: vio_backend: returns the backend initialized.
  /// @param: initial_state_gt: serves as input in case there is ground-truth
  /// available for the initial state and one wants to initialize the backend
  /// using this information. And also as output by returning the eventually
  /// used initial state (either grount-truth, or guessed from imu data).
  bool initBackend(std::shared_ptr<VioBackEnd>* vio_backend,
                   const gtsam::Pose3& B_Pose_camLrect,
                   const gtsam::Cal3_S2& left_undist_rect_cam_mat,
                   const double& baseline,
                   const VioBackEndParams& vio_params,
                   std::shared_ptr<gtNavState>* initial_state_gt,
                   const Timestamp& timestamp_k,
                   const ImuAccGyr& imu_accgyr);

  StatusSmartStereoMeasurements featureSelect(const VioFrontEndParams& tracker_params,
      const ETHDatasetParser& dataset,
      const Timestamp& timestamp_k,
      const gtsam::Pose3& W_Pose_Blkf,
      double* feature_selection_time,
      const StereoFrame& stereoFrame_km1,
      const StatusSmartStereoMeasurements &smart_stereo_meas,
      const double& cur_kf_id,
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
  ETHDatasetParser dataset_;
  size_t initial_k_, final_k_; // initial and final frame: useful to skip a bunch of images at the beginning (imu calibration)
  Timestamp timestamp_lkf_;
  Timestamp timestamp_k_;

  // Init Vio parameters (should be done inside VIO).
  VioBackEndParamsPtr vio_params_;
  VioFrontEndParams tracker_params_;

  // TODO this should go to another class to avoid not having copy-ctor...
  // Frontend.
  std::unique_ptr<StereoVisionFrontEnd> stereo_vision_frontend_;
  FeatureSelector feature_selector_;

  // Create VIO: class that implements estimation back-end.
  std::shared_ptr<VioBackEnd> vio_backend_;

  // Thread-safe queue for the backend.
  ThreadsafeQueue<VioBackEndInputPayload> backend_input_queue_;
  ThreadsafeQueue<VioBackEndOutputPayload> backend_output_queue_;

  // Structures to be filled with imu data.
  // TODO wrap in a single class.
  ImuStamps imu_stamps_;
  ImuAccGyr imu_accgyr_;

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
};
} // End of VIO namespace
