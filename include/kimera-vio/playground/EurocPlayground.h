/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   EurocPlayground.h
 * @brief  Loads Euroc datasets with trajectories, images and even pointcloud if
 * requested. It allows one to play with realistic data, while having
 * ground-truth.
 * @author Antoni Rosinol
 */

#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/dataprovider/DataProviderModule.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/DisplayModule.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

class EurocPlayground {
 public:
  EurocPlayground(const std::string& dataset_path,
                  const int& initial_k = 20,
                  const int& final_k = 10000)
      : dataset_path_(dataset_path),
        euroc_data_provider_(nullptr),
        visualizer_3d_(nullptr),
        display_module_(nullptr),
        display_input_queue_("display_input_queue") {
    // Create euroc data parser
    VioParams vio_params("");  // Use default params.
    vio_params.parallel_run_ = false;
    euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(
        dataset_path, initial_k, final_k, vio_params);
    euroc_data_provider_->registerImuSingleCallback(
          std::bind(&EurocPlayground::fillImuQueue, this,
                    std::placeholders::_1));
    euroc_data_provider_->parse();

    // Create 3D visualizer
    VisualizationType viz_type = VisualizationType::kPointcloud;
    BackendType backend_type = BackendType::kStereoImu;
    visualizer_3d_ = VisualizerFactory::createVisualizer(
        VisualizerType::OpenCV,
        static_cast<VisualizationType>(viz_type),
        backend_type);

    // Create Displayer
    display_module_ = VIO::make_unique<DisplayModule>(
        &display_input_queue_,
        nullptr,
        vio_params.parallel_run_,
        VIO::make_unique<OpenCv3dDisplay>(nullptr));
  }

  ~EurocPlayground() = default;

public:

  /**
   * @brief visualizeGtData Spawns a 3D window where all ground-truth data is
   * visualized according to given flags
   * @param viz_traj Visualize ground-truth trajectory
   * @param viz_img_in_frustum Visualize actual images inside frustums
   * @param viz_pointcloud Visualize ground-truth 3D pointcloud
   */
  void visualizeGtData(const bool& viz_traj,
                       const bool& viz_img_in_frustum,
                       const bool& viz_pointcloud) {
    VisualizerOutput::UniquePtr output = VIO::make_unique<VisualizerOutput>();
    output->visualization_type_ = VisualizationType::kPointcloud;
    if (viz_traj) {
      LOG_IF(ERROR, euroc_data_provider_->gt_data_.map_to_gt_.size() == 0)
          << "Empty ground-truth trajectory.";
      for (const auto& kv : euroc_data_provider_->gt_data_.map_to_gt_) {
        const VioNavState& state = kv.second;
        visualizer_3d_->addPoseToTrajectory(
            state.pose_.compose(euroc_data_provider_->gt_data_.body_Pose_cam_));
      }
      cv::Mat frustum_img;
      if (viz_img_in_frustum) {
      }
      visualizer_3d_->visualizeTrajectory3D(
          frustum_img, &output->frustum_pose_, &output->widgets_);
    }

    if (viz_pointcloud) {
      static bool visualize_ply_mesh_once = true;
      static const std::string pcl_ply_filename =
          dataset_path_ + "/mav0/pointcloud0/data.ply";
      if (visualize_ply_mesh_once) {
        visualizer_3d_->visualizePlyMesh(pcl_ply_filename, &output->widgets_);
        visualize_ply_mesh_once = false;
      }
    }
    display_module_->spinOnce(std::move(output));
  }

  //! Fill one IMU measurement only
  inline void fillImuQueue(const ImuMeasurement& imu_measurement) {
    imu_data_.imu_buffer_.addMeasurement(imu_measurement.timestamp_,
                                         imu_measurement.acc_gyr_);
  }

 protected:
  std::string dataset_path_;

  //! Modules
  EurocDataProvider::UniquePtr euroc_data_provider_;
  Visualizer3D::UniquePtr visualizer_3d_;
  DisplayModule::UniquePtr display_module_;
  DisplayModule::InputQueue display_input_queue_;

  //! Data
  ImuData imu_data_;
};
}
