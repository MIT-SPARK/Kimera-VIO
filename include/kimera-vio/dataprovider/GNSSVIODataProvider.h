/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GNSSVIODataProvider.h
 * @brief  Parse EUROC dataset format with additional info about GNSS position.
 * @author Antoni Rosinol
 * @author Yun Chang
 * @author Luca Carlone
 * @author Elizaveta Karaseva
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include <map>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/*
 * Parse all images and camera calibration for an ETH dataset.
 */
class GNSSVIODataProvider : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(GNSSVIODataProvider);
  KIMERA_POINTER_TYPEDEFS(GNSSVIODataProvider);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ctor with params.
  GNSSVIODataProvider(const std::string& dataset_path,
                      const int& initial_k,
                      const int& final_k,
                      const VioParams& vio_params);
  //! Ctor from gflags
  explicit GNSSVIODataProvider(const VioParams& vio_params);

    ~GNSSVIODataProvider() {
        LOG(INFO) << "GNSS DatasetParser destructor called.";
    }

 public:
  /**
   * @brief spin Spins the dataset until it finishes. If set in sequential mode,
   * it will return each time a frame is sent. In parallel mode, it will not
   * return until it finishes.
   * @return True if the dataset still has data, false otherwise.
   */
  bool spin() override;

  bool hasData() const override;

  void print() const;

 public:
  // Ground truth data.
  GroundTruthData gt_data_;

  // Retrieve absolute gt pose at *approx* timestamp.
  inline gtsam::Pose3 getGroundTruthPose(const Timestamp& timestamp) const {
    return getGroundTruthState(timestamp).pose_;
  }

  inline std::string getDatasetPath() const {
    return dataset_path_;
  }
  std::string getDatasetName();

 protected:
  /**
   * @brief spinOnce Send data to VIO pipeline on a per-frame basis
   * @return if the dataset finished or not
   */
  virtual bool spinOnce();

  void parse();

  void sendImuData() const;

  void sendGnssData() const;

  bool parseDataset();

  bool parseImuData(const std::string& input_dataset_path,
    const std::string& imu_name);

  bool parseGtData(const std::string& input_dataset_path,
                   const std::string& gtSensorName);

  bool parseGnssData(const std::string& input_dataset_path,
                     const std::string& gnss_sensor_name);

  bool parseCameraData(const std::string& cam_name,
                       CameraImageLists* cam_list_i);

  inline bool getLeftImgName(const size_t& k, std::string* img_name) const {
    return getImgName("cam0", k, img_name);
  }

  inline bool getRightImgName(const size_t& k, std::string* img_name) const {
    return getImgName("cam1", k, img_name);
  }

  gtsam::Pose3 getGroundTruthRelativePose(
      const Timestamp& previousTimestamp,
      const Timestamp& currentTimestamp) const;

  // Retrieve absolute pose at timestamp.
  VioNavState getGroundTruthState(const Timestamp& timestamp) const;

  // Compute initialization errors and stats.
  const InitializationPerformance getInitializationPerformance(
      const std::vector<Timestamp>& timestamps,
      const std::vector<gtsam::Pose3>& poses_ba,
      const VioNavState& init_nav_state,
      const gtsam::Vector3& init_gravity);

  size_t getNumImages() const;
  size_t getNumImagesForCamera(const std::string& camera_name) const;

  bool getImgName(const std::string& camera_name,
                  const size_t& k,
                  std::string* img_filename) const;

  //! Sanity checks
  bool sanityCheckCameraData(
      const std::vector<std::string>& camera_names,
      std::map<std::string, CameraImageLists>* camera_image_lists) const;

  // Sanity check: nr images is the same for left and right camera
  // Resizes left/right img lists to the minimum number of frames in case of
  // different list sizes.
  bool sanityCheckCamSize(CameraImageLists::ImgLists* left_img_lists,
                          CameraImageLists::ImgLists* right_img_lists) const;

  // Sanity check: time stamps are the same for left and right camera
  bool sanityCheckCamTimestamps(
      const CameraImageLists::ImgLists& left_img_lists,
      const CameraImageLists::ImgLists& right_img_lists,
      const CameraParams& left_cam_info) const;

  //! Utilities
  // Check if the ground truth is available.
  // (i.e., the timestamp is after the first gt state)
  bool isGroundTruthAvailable(const Timestamp& timestamp) const;

  // Get if the dataset has ground truth.
  bool isGroundTruthAvailable() const;

  // Get timestamp of a given pair of stereo images (synchronized).
  Timestamp timestampAtFrame(const FrameId& frame_number);

  // Clip final frame to the number of images in the dataset.
  void clipFinalFrame();

 protected:
  VioParams vio_params_;

  /// Images data.
  // TODO(Toni): remove camera_names_ and camera_image_lists_...
  // This matches the names of the folders in the dataset
  std::vector<std::string> camera_names_;
  // Map from camera name to its images
  std::map<std::string, CameraImageLists> camera_image_lists_;

  bool is_gt_available_;
  bool is_gnss_available_;
  std::string dataset_name_;
  std::string dataset_path_;

  FrameId current_k_;
  FrameId initial_k_;  // start frame
  FrameId final_k_;    // end frame

  //! Flag to signal when the dataset has been parsed.
  bool dataset_parsed_ = false;
  //! Flag to signal if the IMU data has been sent to the VIO pipeline
  bool is_imu_data_sent_ = false;

  bool is_gnss_data_sent_ = false;

  const std::string kLeftCamName = "cam0";
  const std::string kRightCamName = "cam1";
  const std::string kImuName = "imu0";
  const std::string kGnssName = "gnss0";

  //! Pre-stored imu-measurements
  std::vector<ImuMeasurement> imu_measurements_;
  std::vector<GnssMeasurement> gnss_measurements_;

  EurocGtLogger::UniquePtr logger_;
};

}  // namespace VIO
