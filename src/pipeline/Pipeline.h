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

  // Structures to be filled with imu data.
  // TODO wrap in a single class.
  ImuStamps imu_stamps_;
  ImuAccGyr imu_accgyr_;

  // Set of planes in the scene.
  std::vector<Plane> planes_;

  // Logger class (stores data for matlab visualization).
  LoggerMatlab logger_;
  double start_time_; // to log timing results

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
  std::thread mesher_thread_;
  std::thread visualizer_thread_;
};
} // End of VIO namespace
