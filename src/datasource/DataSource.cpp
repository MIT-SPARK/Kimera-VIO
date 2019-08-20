/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataSource.cpp
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */
#include "datasource/DataSource.h"

DEFINE_string(vio_params_path, "", "Path to vio user-defined parameters.");
DEFINE_string(tracker_params_path, "",
              "Path to tracker user-defined parameters.");
DEFINE_string(lcd_params_path, "",
              "Path to loop-closure-detector user-defined parameters.");
DEFINE_int32(backend_type, 0,
             "Type of vioBackEnd to use:\n"
             "0: VioBackEnd\n"
             "1: RegularVioBackEnd");

DEFINE_int64(initial_k, 50,
             "Initial frame to start processing dataset, "
             "previous frames will not be used.");
DEFINE_int64(final_k, 10000,
             "Final frame to finish processing dataset, "
             "subsequent frames will not be used.");
DEFINE_string(dataset_path, "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");

namespace VIO {

gtNavState::gtNavState(const gtsam::Pose3& pose,
    const gtsam::Vector3& velocity,
    const gtsam::imuBias::ConstantBias& imu_bias)
  : pose_(pose),
    velocity_(velocity),
    imu_bias_(imu_bias) {}

gtNavState::gtNavState(const gtsam::NavState& nav_state,
    const gtsam::imuBias::ConstantBias& imu_bias)
  : pose_(nav_state.pose()),
    velocity_(nav_state.velocity()),
    imu_bias_(imu_bias) {}

void gtNavState::print(const std::string& message) const {
  if (VLOG_IS_ON(10)) {
    LOG(INFO) << "--- " << message << "--- ";
    pose_.print("\n pose: \n");
    LOG(INFO) << "\n velocity: \n" << velocity_.transpose();
    imu_bias_.print("\n imuBias: \n");
  }
}

bool CameraImageLists::parseCamImgList(const std::string& folderpath,
                                       const std::string& filename) {
  image_folder_path_ = folderpath;  // stored, only for debug
  const std::string fullname = folderpath + "/" + filename;
  std::ifstream fin(fullname.c_str());
  LOG_IF(FATAL, !fin.is_open()) << "Cannot open file: " << fullname;

  // Skip the first line, containing the header.
  std::string item;
  std::getline(fin, item);

  // Read/store list of image names.
  while (std::getline(fin, item)) {
    // Print the item!
    int idx = item.find_first_of(',');
    Timestamp timestamp = std::stoll(item.substr(0, idx));
    std::string imageFilename =
        folderpath + "/data/" + item.substr(0, idx) + ".png";
    // Strangely, on mac, it does not work if we use: item.substr(idx + 1);
    // maybe different string termination characters???
    img_lists.push_back(make_pair(timestamp, imageFilename));
  }
  fin.close();
  return true;
}

/* -------------------------------------------------------------------------- */
void CameraImageLists::print() const {
  LOG(INFO) << "------------ CameraImageLists::print -------------\n"
            << "image_folder_path: " << image_folder_path_ << '\n'
            << "img_lists size: " << img_lists.size();
}

void GroundTruthData::print() const {
  LOG(INFO) << "------------ GroundTruthData::print -------------";
  body_Pose_cam_.print("body_Pose_cam_: \n");
  LOG(INFO) << "\n gt_rate: " << gt_rate_ << '\n'
            << "nr of gtStates: " << mapToGt_.size();
}

InitializationPerformance::InitializationPerformance(
    const Timestamp init_timestamp,
    const int init_n_frames,
    const double avg_rotationErrorBA,
    const double avg_tranErrorBA,
    const gtNavState init_nav_state,
    const gtsam::Vector3 init_gravity,
    const gtNavState gt_nav_state,
    const gtsam::Vector3 gt_gravity)
  : init_timestamp_(init_timestamp),
    init_n_frames_(init_n_frames),
    avg_rotationErrorBA_(avg_rotationErrorBA),
    avg_tranErrorBA_(avg_tranErrorBA),
    init_nav_state_(init_nav_state),
    init_gravity_(init_gravity),
    gt_nav_state_(gt_nav_state),
    gt_gravity_(gt_gravity) {}

void InitializationPerformance::print() const {
      // Log everything
      LOG(INFO) << "BUNDLE ADJUSTMENT\n"
              << "Average relative errors: (BA vs. GT)\n"
              << "(avg. relative rot error)\n"
              << avg_rotationErrorBA_
              << "\n(avg. relative tran error)\n"
              << avg_tranErrorBA_
              << "\nONLINE GRAVITY ALIGNMENT\n"
              << "Initialization state:\n"
              << "(timestamp)\n"
              << init_timestamp_
              << "\n(pitch estimate)\n"
              << init_nav_state_.pose().rotation().pitch()*180.0/M_PI
              << "\n(pitch GT)\n"
              << gt_nav_state_.pose().rotation().pitch()*180.0/M_PI
              << "\n(roll estimate)\n"
              << init_nav_state_.pose().rotation().roll()*180.0/M_PI
              << "\n(roll GT)\n"
              << gt_nav_state_.pose().rotation().roll()*180.0/M_PI
              << "\n(gyroscope bias estimate)\n"
              << init_nav_state_.imu_bias_.gyroscope()
              << "\n(gyroscope bias GT)\n"
              << gt_nav_state_.imu_bias_.gyroscope()
              << "\n(initial body frame velocity estimate)\n"
              << init_nav_state_.velocity_
              << "\n(initial body frame velocity GT)\n"
              << gt_nav_state_.velocity_
              << "\n(initial body frame gravity estimate)\n"
              << init_gravity_
              << "\n(initial body frame gravity GT)\n"
              << gt_gravity_;
}

DataProvider::DataProvider() :
    initial_k_(FLAGS_initial_k),
    final_k_(FLAGS_final_k),
    dataset_path_(FLAGS_dataset_path) {

  CHECK(final_k_ > initial_k_)
      << "Value for final_k (" << final_k_
      << ") is smaller than value for"
      << " initial_k (" << initial_k_ << ").";

  LOG(INFO) << "Running dataset between frame " << initial_k_
          << " and frame " << final_k_;
}

DataProvider::~DataProvider() {
  LOG(INFO) << "Data provider destructor called.";
}

void DataProvider::registerVioCallback(
    std::function<void(const StereoImuSyncPacket&)> callback) {
  vio_callback_ = std::move(callback);
}

bool DataProvider::spin() {
  // Dummy example:
  // 1) Check that the vio_callback_ has been registered, aka that the user has
  // called the function registerVioCallback, in order to store the callback
  // function.
  CHECK(vio_callback_);

  // 2) Loop over the dataset and:
  //  a) Create StereoImuSyncPacket packets out of the data.
  //  This one is dummy since it is filled with empty images, parameters,
  //  imu data, etc.
  StereoImuSyncPacket vio_packet(
      StereoFrame(1, 1, cv::Mat(), CameraParams(), cv::Mat(), CameraParams(),
                  gtsam::Pose3(), StereoMatchingParams()),
      ImuStampS(), ImuAccGyrS());
  //  b) Call the vio callback in order to start processing the packet.
  vio_callback_(vio_packet);

  // 3) Once the dataset spin has finished, exit.
  // You can return false if something went wrong.
  return true;
}

/* -------------------------------------------------------------------------- */
void DataProvider::parseBackendParams() {
  switch (FLAGS_backend_type) {
    case 0: {
      pipeline_params_.backend_params_ = std::make_shared<VioBackEndParams>();
      break;
    }
    case 1: {
      pipeline_params_.backend_params_ = std::make_shared<RegularVioBackEndParams>();
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized backend type: " << FLAGS_backend_type << "."
                   << " 0: normalVio, 1: RegularVio.";
    }
  }

  pipeline_params_.backend_type_ = FLAGS_backend_type;

  // Read/define vio params.
  if (FLAGS_vio_params_path.empty()) {
    LOG(WARNING) << "No vio parameters specified, using default.";
    // Default params with IMU stats from dataset.
    pipeline_params_.backend_params_->gyroNoiseDensity_ = pipeline_params_.imu_params_.gyro_noise_;
    pipeline_params_.backend_params_->accNoiseDensity_ = pipeline_params_.imu_params_.acc_noise_;
    pipeline_params_.backend_params_->gyroBiasSigma_ = pipeline_params_.imu_params_.gyro_walk_;
    pipeline_params_.backend_params_->accBiasSigma_ = pipeline_params_.imu_params_.acc_walk_;
  } else {
    VLOG(100) << "Using user-specified VIO parameters: "
              << FLAGS_vio_params_path;
    pipeline_params_.backend_params_->parseYAML(FLAGS_vio_params_path);
  }
  // TODO(Toni) make this cleaner! pipeline_params_.imu_params_ are parsed all around, it's a
  // mess!! They are basically parsed from backend params... but they should be
  // on their own mostly.
  pipeline_params_.imu_params_.imu_integration_sigma_ = pipeline_params_.backend_params_->imuIntegrationSigma_;
  pipeline_params_.imu_params_.n_gravity_ = pipeline_params_.backend_params_->n_gravity_;
}

/* -------------------------------------------------------------------------- */
void DataProvider::parseFrontendParams() {

  // Read/define tracker params.
  if (FLAGS_tracker_params_path.empty()) {
    LOG(WARNING) << "No tracker parameters specified, using default";
    pipeline_params_.frontend_params_ = VioFrontEndParams();  // default params
  } else {
    VLOG(100) << "Using user-specified tracker parameters: "
              << FLAGS_tracker_params_path;
    pipeline_params_.frontend_params_.parseYAML(FLAGS_tracker_params_path);
  }

  // Read/define LCD params.
  if (FLAGS_lcd_params_path.empty()) {
    VLOG(100) << "No LoopClosureDetector parameters specified, using default";
    pipeline_params_.lcd_params_ = LoopClosureDetectorParams();
  } else {
    VLOG(100) << "Using user-specified LoopClosureDetector parameters: "
              << FLAGS_lcd_params_path;
    pipeline_params_.lcd_params_.parseYAML(FLAGS_lcd_params_path);
  }

  CHECK(pipeline_params_.backend_params_);
  CHECK_NOTNULL(&pipeline_params_.frontend_params_);
  CHECK_NOTNULL(&pipeline_params_.lcd_params_);
}

}  // namespace VIO
