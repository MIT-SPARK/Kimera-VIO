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
#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <thread>
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
      imu_measurements_(),
      left_cam_info_(vio_params_.camera_params_.at(0)){}

/* -------------------------------------------------------------------------- */
OAKDataProvider::~OAKDataProvider() {
  LOG(INFO) << "OAKDatasetParser destructor called.";
}

void OAKDataProvider::setLeftImuQueues(std::shared_ptr<dai::DataOutputQueue> left_queue, std::shared_ptr<dai::DataOutputQueue> imu_queue){
    left_queue_ = left_queue;
    imu_queue_ = imu_queue;
}


/* -------------------------------------------------------------------------- */
bool OAKDataProvider::spin() {
    // Spin.
    CHECK_EQ(vio_params_.camera_params_.size(), 2u);
  LOG(INFO) << "Data OAKDataProvider Interface: <-------------- Spinning -------------->";

    while (!shutdown_) {
        std::shared_ptr<dai::ADatatype> left_image       = left_queue_->get<dai::ADatatype>();
        std::shared_ptr<dai::ADatatype> imu_measurements = imu_queue_->get<dai::ADatatype>();
        
        std::string name = "";
        imuCallback(name, imu_measurements);
        leftImageCallback("left", left_image);
        if (!vio_params_.parallel_run_) {
            return true;
        }
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  LOG_IF(INFO, shutdown_) << "OAKD DataProvider shutdown requested.";
  return false;
}

void OAKDataProvider::leftImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data){

    CHECK(left_frame_callback_) << "Did you forget to register the left image callback to the VIO Pipeline?";
    auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat imageFrame = daiDataPtr->getCvFrame();
    Timestamp localTimestamp = timestampAtFrame(daiDataPtr->getTimestamp());
    // TODO(saching): Add option to equalize the image from histogram
    left_frame_callback_(
        VIO::make_unique<Frame>(daiDataPtr->getSequenceNum(),
                                localTimestamp,
                                left_cam_info_,
                                imageFrame));
}

void OAKDataProvider::FillImuDataLinearInterpolation(std::vector<dai::IMUPacket>& imuPackets) {
    // int accelSequenceNum = -1, gyroSequenceNum = -1;
    static std::deque<dai::IMUReportAccelerometer> accelHist;
    static std::deque<dai::IMUReportGyroscope> gyroHist;
    // std::deque<dai::IMUReportRotationVectorWAcc> rotationVecHist;

    for(unsigned int i = 0; i < imuPackets.size(); ++i) {
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

        if(syncMode_ == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
            if(accelHist.size() < 3) {
                continue;
            } else {
                dai::IMUReportAccelerometer accel0, accel1;
                dai::IMUReportGyroscope currGyro;
                accel0.sequence = -1;
                // LOG(WARNING) << "IMU INTERPOLATION: Interpolating LINEAR_INTERPOLATE_ACCEL mode ";
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
                            LOG(WARNING) << "IMU INTERPOLATION: Gyro data not found. Dropping accel data points";
                        }
                        while(gyroHist.size()) {
                            currGyro = gyroHist.front();

                            /* LOG(WARNING) <<
                                "IMU INTERPOLATION: ",
                                "Accel 0: Seq => " << accel0.sequence << " timeStamp => " << (accel0.timestamp.get() - _steadyBaseTime).count();
                            LOG(WARNING) << "IMU INTERPOLATION: ",
                                                     "currGyro 0: Seq => " << currGyro.sequence << "timeStamp => "
                                                                           << (currGyro.timestamp.get() - _steadyBaseTime).count();
                            LOG(WARNING) <<
                                "IMU INTERPOLATION: ",
                                "Accel 1: Seq => " << accel1.sequence << " timeStamp => " << (accel1.timestamp.get() - _steadyBaseTime).count(); */
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
                                LOG(WARNING) << "IMU INTERPOLATION: Dropping GYRO with old timestamps which are below accel10";
                            }
                        }
                        // gyroHist.push_back(currGyro); // Undecided whether this is necessary
                        accel0 = accel1;
                    }
                }
                // LOG(WARNING) << "IMU INTERPOLATION: Count  ->" << i << " Placing Accel 0 Seq Number :" << accel0.sequence;

                accelHist.push_back(accel0);
            }
        } else if(syncMode_ == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
            if(gyroHist.size() < 3) {
                continue;
            } else {
                dai::IMUReportGyroscope gyro0, gyro1;
                dai::IMUReportAccelerometer currAccel;
                gyro0.sequence = -1;
                // LOG(WARNING) << "IMU INTERPOLATION: Interpolating LINEAR_INTERPOLATE_GYRO mode ";
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
                            LOG(WARNING) << "IMU INTERPOLATION: Accel data not found. Dropping data";
                        }
                        while(accelHist.size()) {
                            currAccel = accelHist.front();
                            /*                             
                            LOG(WARNING) << "IMU INTERPOLATION: ",
                                                     "gyro 0: Seq => " << gyro0.sequence << std::endl
                                                                       << "       timeStamp => " << (gyro0.timestamp.get() - _steadyBaseTime).count();
                            LOG(WARNING) << "IMU INTERPOLATION: ",
                                                     "currAccel 0: Seq => " << currAccel.sequence << std::endl
                                                                            << "       timeStamp => " << (currAccel.timestamp.get() - _steadyBaseTime).count();
                            LOG(WARNING)  << "IMU INTERPOLATION: ",
                                                     "gyro 1: Seq => " << gyro1.sequence << std::endl
                                                                       << "       timeStamp => " << (gyro1.timestamp.get() - _steadyBaseTime).count(); */
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
                                LOG(WARNING) << "IMU INTERPOLATION: Droppinh ACCEL with old timestamps which are below accel10";
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


void OAKDataProvider::sendImuMeasurement(dai::IMUReportAccelerometer accel, dai::IMUReportGyroscope gyro) {

    Vector6 imu_accgyr;
    // imu_accgyr << accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z; // Original order
    
    // IMU position w.r.t baseframe is different. It's rotated such that Z is pointing backwards. 
    // So we need to tweak the measurements axis accel and gyro values to match the baseframe 
    // to match with Kimera-VIO convention. An alternative would be to modify 
    // the IMU usage by plugging in the IMU positions in both frountend and backed of VIO
    imu_accgyr << accel.y, -accel.z, -accel.x, gyro.y, -gyro.z, -gyro.x; // Order of accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z in the order interms of base.
    ImuStamp timestamp;
    LOG(WARNING) << "IMUDta-> x m/s: " << accel.y << " y m/s: " << -accel.z << " z m/s: " << -accel.x << " r rad/s: " <<  gyro.y << " p rad/s: " << -gyro.z << " y rad/s: " << -gyro.x;

    if(syncMode_ == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
        timestamp = timestampAtFrame(gyro.timestamp.get());
    } else if(syncMode_ == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
        timestamp = timestampAtFrame(accel.timestamp.get());
    } else {
        timestamp = timestampAtFrame(accel.timestamp.get());
    }
    // LOG(INFO) << "Calling imu_single_callback_ with timestamp " << timestamp;

    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
}

void OAKDataProvider::imuCallback(std::string name, std::shared_ptr<dai::ADatatype> data){
  // TODO(saching): Maybe I should do this only once ?
    CHECK(imu_single_callback_) << "Did you forget to register the IMU callback to the VIO Pipeline?";

    auto daiIMUDataPtr = std::dynamic_pointer_cast<dai::IMUData>(data);
    if(syncMode_ != ImuSyncMethod::COPY) {
        FillImuDataLinearInterpolation(daiIMUDataPtr->packets);
    } else {
        for(int i = 0; i < daiIMUDataPtr->packets.size(); ++i) {
            auto accel = daiIMUDataPtr->packets[i].acceleroMeter;
            auto gyro = daiIMUDataPtr->packets[i].gyroscope;
            sendImuMeasurement(accel, gyro);
        }
    }
}


void OAKDataProvider::sendImuData() const {
  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
  Timestamp previous_timestamp = -1;
  for (const ImuMeasurement& imu_meas : imu_measurements_) {
    CHECK_GT(imu_meas.timestamp_, previous_timestamp)
        << "OAK IMU data is not in chronological order!";
    previous_timestamp = imu_meas.timestamp_;
    imu_single_callback_(imu_meas);
  }
}


/* -------------------------------------------------------------------------- */
Timestamp OAKDataProvider::timestampAtFrame(const std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>& timestamp) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch())
                                                                          .count();
}


}  // namespace VIO
