/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OAKDataProvider.h
 * @brief  Parse OAK device Streams.
 * @author Sachin Guruswamy
 */

#include "kimera-vio/dataprovider/OAKDataProvider.h"

#include <algorithm>  // for max
#include <fstream>
#include <map>
#include <string>
#include <utility>  // for pair<>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/YamlParser.h"


namespace VIO {

/* -------------------------------------------------------------------------- */
OAKDataProvider::OAKDataProvider(const VioParams& vio_params)
    : DataProviderInterface(),
      vio_params_(vio_params),
      imu_measurements_(){

  left_cam_info_ = vio_params_.camera_params_.at(0);
  right_cam_info_ = vio_params_.camera_params_.at(1);

  // Parse the actual dataset first, then run it.
  if (!shutdown_) {
    LOG(INFO) << "Parsing OAK's streams...";
    parse();
    CHECK_GT(imu_measurements_.size(), 0u);

  }
}

/* -------------------------------------------------------------------------- */
OAKDataProvider::~OAKDataProvider() {
  LOG(INFO) << "OAKDatasetParser destructor called.";
}

/* -------------------------------------------------------------------------- */
bool OAKDataProvider::spin() {
    // Spin.
    CHECK_EQ(vio_params_.camera_params_.size(), 2u);
    // We log only the first one, because we may be running in sequential mode.
    LOG_FIRST_N(INFO, 1) << "Running dataset between frame " << initial_k_
                         << " and frame " << final_k_;
    while (!shutdown_) {
      /* 
        TODO(saching): Add sequential callbacks for queues to cross check in sequencial mode. But would ne needed most lieky with dataset
      */
      if (!vio_params_.parallel_run_) {
        // Return, instead of blocking, when running in sequential mode.
            LOG(ERROR) << "Sequential mode is not implemented for OAK-D's pipeline yet.";

        return true;
      }
    }
  LOG_IF(INFO, shutdown_) << "OAKD DataProvider shutdown requested.";
  return false;
}

/* -------------------------------------------------------------------------- */
/* bool OAKDataProvider::spinOnce() {
  CHECK_LT(current_k_, std::numeric_limits<FrameId>::max())
      << "Are you sure you've initialized current_k_?";
  if (current_k_ >= final_k_) {
    LOG(INFO) << "Finished spinning Euroc dataset.";
    return false;
  }

  const CameraParams& left_cam_info = vio_params_.camera_params_.at(0);
  const CameraParams& right_cam_info = vio_params_.camera_params_.at(1);
  const bool& equalize_image =
      vio_params_.frontend_params_.stereo_matching_params_.equalize_image_;

  const Timestamp& timestamp_frame_k = timestampAtFrame(timestamp);
  VLOG(10) << "Sending left/right frames k= " << current_k_
           << " with timestamp: " << timestamp_frame_k;

  // TODO(Toni): ideally only send cv::Mat raw images...:
  // - pass params to vio_pipeline ctor
  // - make vio_pipeline actually equalize or transform images as necessary.
  std::string left_img_filename;
  bool available_left_img = getLeftImgName(current_k_, &left_img_filename);
  std::string right_img_filename;
  bool available_right_img = getRightImgName(current_k_, &right_img_filename);
  if (available_left_img && available_right_img) {
    // Both stereo images are available, send data to VIO
    CHECK(left_frame_callback_);
    left_frame_callback_(
        VIO::make_unique<Frame>(current_k_,
                                timestamp_frame_k,
                                // TODO(Toni): this info should be passed to
                                // the camera... not all the time here...
                                left_cam_info,
                                UtilsOpenCV::ReadAndConvertToGrayScale(
                                    left_img_filename, equalize_image)));
    CHECK(right_frame_callback_);
    right_frame_callback_(
        VIO::make_unique<Frame>(current_k_,
                                timestamp_frame_k,
                                // TODO(Toni): this info should be passed to
                                // the camera... not all the time here...
                                right_cam_info,
                                UtilsOpenCV::ReadAndConvertToGrayScale(
                                    right_img_filename, equalize_image)));
  } else {
    LOG(ERROR) << "Missing left/right stereo pair, proceeding to the next one.";
  }

  // This is done directly when parsing the Imu data.
  // imu_single_callback_(imu_meas);

  VLOG(10) << "Finished VIO processing for frame k = " << current_k_;
  current_k_++;
  return true;
} */

void OAKDataProvider::leftImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data){

    CHECK(left_frame_callback_) << "Did you forget to register the left image callback to the VIO Pipeline?";
    auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat imageFrame = daiDataPtr->getCvFrame();
    // TODO(saching): Add option to equalize the image from histogram
    left_frame_callback_(
        VIO::make_unique<Frame>(daiDataPtr->getSequenceNum();,
                                timestampAtFrame(daiDataPtr->getTimestamp()),
                                left_cam_info,
                                imageFrame));
}

void OAKDataProvider::rightImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data){
    CHECK(right_frame_callback_) << "Did you forget to register the right image callback to the VIO Pipeline?";
    auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat imageFrame = daiDataPtr->getCvFrame();
    // TODO(saching): Add option to equalize the image from histogram
    right_frame_callback_(
        VIO::make_unique<Frame>(daiDataPtr->getSequenceNum();,
                                timestampAtFrame(daiDataPtr->getTimestamp()),
                                right_cam_info,
                                imageFrame));
}


void OAKDataProvider::FillImuDataLinearInterpolation(std::vector<IMUPacket>& imuPackets) {
    // int accelSequenceNum = -1, gyroSequenceNum = -1;
    static std::deque<dai::IMUReportAccelerometer> accelHist;
    static std::deque<dai::IMUReportGyroscope> gyroHist;
    // std::deque<dai::IMUReportRotationVectorWAcc> rotationVecHist;

    for(int i = 0; i < imuPackets.size(); ++i) {
        if(accelHist.size() == 0) {
            accelHist.push_back(imuPackets[i].acceleroMeter);
        } else if(accelHist.back().sequence != imuPackets[i].acceleroMeter.sequence) {
            accelHist.push_back(imuPackets[i].acceleroMeter);
        }

        if(gyroHist.size() == 0) {
            gyroHist.push_back(imuPackets[i].gyroscope);
        } else if(gyroHist.back().sequence != imuPackets[i].gyroscope.sequence) {
            gyroHist.push_back(imuPackets[i].gyroscope);
        }

        if(sync_mode_ == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
            if(accelHist.size() < 3) {
                continue;
            } else {
                dai::IMUReportAccelerometer accel0, accel1;
                dai::IMUReportGyroscope currGyro;
                accel0.sequence = -1;
                LOG(DEBUG) << "IMU INTERPOLATION: ", " Interpolating LINEAR_INTERPOLATE_ACCEL mode ";
                while(accelHist.size()) {
                    if(accel0.sequence == -1) {
                        accel0 = accelHist.front();
                        accelHist.pop_front();
                    } else {
                        accel1 = accelHist.front();
                        accelHist.pop_front();
                        // auto dt = 1e-9
                        //           * static_cast<double>(
                        //               std::chrono::duration_cast<std::chrono::nanoseconds>(accel1.timestamp.get() - accel0.timestamp.get()).count());

                        // remove std::milli to get in seconds
                        std::chrono::duration<double, std::milli> duration_ms = accel1.timestamp.get() - accel0.timestamp.get();
                        double dt = duration_ms.count();

                        if(!gyroHist.size()) {
                            LOG(DEBUG) << "IMU INTERPOLATION: ", "Gyro data not found. Dropping accel data points";
                        }
                        while(gyroHist.size()) {
                            currGyro = gyroHist.front();

                            LOG(DEBUG) <<
                                "IMU INTERPOLATION: ",
                                "Accel 0: Seq => " << accel0.sequence << " timeStamp => " << (accel0.timestamp.get() - _steadyBaseTime).count();
                            LOG(DEBUG) << "IMU INTERPOLATION: ",
                                                     "currGyro 0: Seq => " << currGyro.sequence << "timeStamp => "
                                                                           << (currGyro.timestamp.get() - _steadyBaseTime).count();
                            LOG(DEBUG) <<
                                "IMU INTERPOLATION: ",
                                "Accel 1: Seq => " << accel1.sequence << " timeStamp => " << (accel1.timestamp.get() - _steadyBaseTime).count();
                            if(currGyro.timestamp.get() > accel0.timestamp.get() && currGyro.timestamp.get() <= accel1.timestamp.get()) {
                                // auto alpha = std::chrono::duration_cast<std::chrono::nanoseconds>(currGyro.timestamp.get() - accel0.timestamp.get()).count();
                                // / dt;
                                // remove std::milli to get in seconds
                                std::chrono::duration<double, std::milli> diff = currGyro.timestamp.get() - accel0.timestamp.get();
                                const double alpha = diff.count() / dt;
                                dai::IMUReportAccelerometer interpAccel = lerpImu(accel0, accel1, alpha);
                                sendImuMeasurement(interpAccel, currGyro);
                                gyroHist.pop_front();
                            } else if(currGyro.timestamp.get() > accel1.timestamp.get()) {
                                accel0 = accel1;
                                if(accelHist.size()) {
                                    accel1 = accelHist.front();
                                    accelHist.pop_front();
                                    duration_ms = accel1.timestamp.get() - accel0.timestamp.get();
                                    dt = duration_ms.count();
                                } else {
                                    break;
                                }
                            } else {
                                gyroHist.pop_front();
                                LOG(DEBUG) << "IMU INTERPOLATION: ", "Droppinh GYRO with old timestamps which are below accel10";
                            }
                        }
                        // gyroHist.push_back(currGyro); // Undecided whether this is necessary
                        accel0 = accel1;
                    }
                }
                LOG(DEBUG) << "IMU INTERPOLATION: ", "Count  ->" << i << " Placing Accel 0 Seq Number :" << accel0.sequence;

                accelHist.push_back(accel0);
            }
        } else if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
            if(gyroHist.size() < 3) {
                continue;
            } else {
                dai::IMUReportGyroscope gyro0, gyro1;
                dai::IMUReportAccelerometer currAccel;
                gyro0.sequence = -1;
                LOG(DEBUG) << "IMU INTERPOLATION: ", " Interpolating LINEAR_INTERPOLATE_GYRO mode ";
                while(gyroHist.size()) {
                    if(gyro0.sequence == -1) {
                        gyro0 = gyroHist.front();
                        gyroHist.pop_front();
                    } else {
                        gyro1 = gyroHist.front();
                        gyroHist.pop_front();
                        // remove std::milli to get in seconds
                        std::chrono::duration<double, std::milli> duration_ms = gyro1.timestamp.get() - gyro0.timestamp.get();
                        double dt = duration_ms.count();

                        if(!accelHist.size()) {
                            LOG(DEBUG) << "IMU INTERPOLATION: ", "Accel data not found. Dropping data";
                        }
                        while(accelHist.size()) {
                            currAccel = accelHist.front();
                            LOG(DEBUG) << "IMU INTERPOLATION: ",
                                                     "gyro 0: Seq => " << gyro0.sequence << std::endl
                                                                       << "       timeStamp => " << (gyro0.timestamp.get() - _steadyBaseTime).count();
                            LOG(DEBUG) << "IMU INTERPOLATION: ",
                                                     "currAccel 0: Seq => " << currAccel.sequence << std::endl
                                                                            << "       timeStamp => " << (currAccel.timestamp.get() - _steadyBaseTime).count();
                            LOG(DEBUG)  << "IMU INTERPOLATION: ",
                                                     "gyro 1: Seq => " << gyro1.sequence << std::endl
                                                                       << "       timeStamp => " << (gyro1.timestamp.get() - _steadyBaseTime).count();
                            if(currAccel.timestamp.get() > gyro0.timestamp.get() && currAccel.timestamp.get() <= gyro1.timestamp.get()) {
                                // remove std::milli to get in seconds
                                std::chrono::duration<double, std::milli> diff = currAccel.timestamp.get() - gyro0.timestamp.get();
                                const double alpha = diff.count() / dt;
                                dai::IMUReportGyroscope interpGyro = lerpImu(gyro0, gyro1, alpha);
                                sendImuMeasurement(currAccel, interpGyro);
                                accelHist.pop_front();
                            } else if(currAccel.timestamp.get() > gyro1.timestamp.get()) {
                                gyro0 = gyro1;
                                if(gyroHist.size()) {
                                    gyro1 = gyroHist.front();
                                    gyroHist.pop_front();
                                    duration_ms = gyro1.timestamp.get() - gyro0.timestamp.get();
                                    dt = duration_ms.count();
                                } else {
                                    break;
                                }
                            } else {
                                accelHist.pop_front();
                                LOG(DEBUG) << "IMU INTERPOLATION: ", "Droppinh ACCEL with old timestamps which are below accel10";
                            }
                        }
                        gyro0 = gyro1;
                    }
                }
                gyroHist.push_back(gyro0);
            }
        }
    }
}


void OAKDataProvider::SendImuMeasurement(dai::IMUReportAccelerometer accel, dai::IMUReportGyroscope gyro) {

    Vector6 imu_accgyr;
    imu_accgyr << accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z;
    ImuStamp timestamp;

    if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
        timestamp = timestampAtFrame(_rosBaseTime, _steadyBaseTime, gyro.timestamp.get());
    } else if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
        timestamp = timestampAtFrame(_rosBaseTime, _steadyBaseTime, accel.timestamp.get());
    } else {
        timestamp = timestampAtFrame(_rosBaseTime, _steadyBaseTime, accel.timestamp.get());
    }

    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
}

void imuCallback(std::string name, std::shared_ptr<dai::ADatatype> data){
  // TODO(saching): Maybe I should do this only once ?
    CHECK(imu_single_callback_) << "Did you forget to register the IMU callback to the VIO Pipeline?";

    auto daiIMUDataPtr = std::dynamic_pointer_cast<dai::IMUData>(data);
    if(_syncMode != ImuSyncMethod::COPY) {
        FillImuDataLinearInterpolation(daiIMUDataPtr->packets);
    } else {
        for(int i = 0; i < inData->packets.size(); ++i) {
            auto accel = inData->packets[i].acceleroMeter;
            auto gyro = inData->packets[i].gyroscope;
            sendUnitMessage(accel, gyro);
        }
    }
}


void OAKDataProvider::sendImuData() const {
  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
  Timestamp previous_timestamp = -1;
  for (const ImuMeasurement& imu_meas : imu_measurements_) {
    CHECK_GT(imu_meas.timestamp_, previous_timestamp)
        << "Euroc IMU data is not in chronological order!";
    previous_timestamp = imu_meas.timestamp_;
    imu_single_callback_(imu_meas);
  }
}


/* -------------------------------------------------------------------------- */
Timestamp OAKDataProvider::timestampAtFrame(const std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>& timestamp) {
  // CHECK_GT(camera_names_.size(), 0);
  // CHECK_LT(frame_number,
  //          camera_image_lists_.at(camera_names_[0]).img_lists_.size());
  return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch())
                                                                          .count();
}


}  // namespace VIO
