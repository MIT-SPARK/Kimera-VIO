/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataProviderInterface.cpp
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */

#include "kimera-vio/dataprovider/DataProviderInterface.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/backend/RegularVioBackendParams.h"
#include "kimera-vio/backend/VioBackendParams.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"

namespace VIO {

DataProviderInterface::~DataProviderInterface() {
  LOG(INFO) << "Data provider destructor called.";
}

bool DataProviderInterface::spin() {
  // Dummy example:
  // 1) Check that the callbacks have been registered, aka that the user has
  // called the function registerVioCallback, in order to store the callback
  // function.
  //! Check only the imu callback that you want to use:
  //! - single: only one message at a time.
  //! - multi: multiple message at a time.
  CHECK(imu_single_callback_);
  CHECK(imu_multi_callback_);
  CHECK(left_frame_callback_);
  CHECK(right_frame_callback_);

  // 2) Loop over the dataset and:
  //  a) Create data packets out of the data.
  //  This one is dummy since it is filled with empty images, parameters,
  //  imu data, etc.
  //  b) Call the callbacks in order to send the data.
  if (!shutdown_) {
    left_frame_callback_(
        VIO::make_unique<Frame>(0, 0, CameraParams(), cv::Mat()));
    right_frame_callback_(
        VIO::make_unique<Frame>(0, 0, CameraParams(), cv::Mat()));
    //! Usually you would use only one of these
    imu_single_callback_(ImuMeasurement());
    imu_multi_callback_(ImuMeasurements());
  } else {
    LOG(INFO) << "Not spinning DataProviderInterface, shutdown requested.";
  }

  // 3) Once the dataset spin has finished, exit with false.
  return false;
}

void DataProviderInterface::shutdown() {
  LOG_IF(ERROR, shutdown_)
      << "Shutdown requested, but DataProviderInterface was already "
         "shutdown.";
  LOG(INFO) << "Shutting down DataProviderInterface.";
  shutdown_ = true;
}

}  // namespace VIO
