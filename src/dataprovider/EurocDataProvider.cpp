/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   EurocDataProvider.h
 * @brief  Parse EUROC dataset.
 * @author Antoni Rosinol
 * @author Yun Chang
 * @author Luca Carlone
 */

#include "kimera-vio/dataprovider/EurocDataProvider.h"

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

DEFINE_string(dataset_path,
              "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");
DEFINE_int64(initial_k,
             0,
             "Initial frame to start processing dataset, "
             "previous frames will not be used.");
DEFINE_int64(final_k,
             100000,
             "Final frame to finish processing dataset, "
             "subsequent frames will not be used.");
DEFINE_bool(log_euroc_gt_data,
            false,
            "Log Euroc ground-truth data to file for later evaluation.");

namespace VIO {

/* -------------------------------------------------------------------------- */
EurocDataProvider::EurocDataProvider(const std::string& dataset_path,
                                     const int& initial_k,
                                     const int& final_k,
                                     const VioParams& vio_params)
    : DataProviderInterface(),
      dataset_path_(dataset_path),
      current_k_(std::numeric_limits<FrameId>::max()),
      initial_k_(initial_k),
      final_k_(final_k),
      vio_params_(vio_params),
      imu_measurements_(),
      logger_(FLAGS_log_euroc_gt_data ? VIO::make_unique<EurocGtLogger>()
                                      : nullptr) {
  CHECK(!dataset_path_.empty())
      << "Dataset path for EurocDataProvider is empty.";

  // Start processing dataset from frame initial_k.
  // Useful to skip a bunch of images at the beginning (imu calibration).
  CHECK_GE(initial_k_, 0);

  // Finish processing dataset at frame final_k.
  // Last frame to process (to avoid processing the entire dataset),
  // skip last frames.
  CHECK_GT(final_k_, 0);

  CHECK_GT(final_k_, initial_k_) << "Value for final_k (" << final_k_
                                 << ") is smaller than value for"
                                 << " initial_k (" << initial_k_ << ").";
  current_k_ = initial_k_;

  // Parse the actual dataset first, then run it.
  if (!shutdown_ && !dataset_parsed_) {
    LOG(INFO) << "Parsing Euroc dataset...";
    parse();
    CHECK_GT(imu_measurements_.size(), 0u);
    dataset_parsed_ = true;
  }
}

/* -------------------------------------------------------------------------- */
EurocDataProvider::EurocDataProvider(const VioParams& vio_params)
    : EurocDataProvider(FLAGS_dataset_path,
                        FLAGS_initial_k,
                        FLAGS_final_k,
                        vio_params) {}

/* -------------------------------------------------------------------------- */
EurocDataProvider::~EurocDataProvider() {
  LOG(INFO) << "ETHDatasetParser destructor called.";
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::spin() {
  if (dataset_parsed_) {
    if (!is_imu_data_sent_) {
      // First, send all the IMU data. The flag is to avoid sending it several
      // times if we are running in sequential mode.
      if (imu_single_callback_) {
        sendImuData();
      } else {
        LOG(ERROR) << "Imu callback not registered! Not sending IMU data.";
      }
      is_imu_data_sent_ = true;
    }

    // Spin.
    CHECK_EQ(vio_params_.camera_params_.size(), 2u);
    CHECK_GT(final_k_, initial_k_);
    // We log only the first one, because we may be running in sequential mode.
    LOG_FIRST_N(INFO, 1) << "Running dataset between frame " << initial_k_
                         << " and frame " << final_k_;
    while (!shutdown_ && spinOnce()) {
      if (!vio_params_.parallel_run_) {
        // Return, instead of blocking, when running in sequential mode.
        return true;
      }
    }
  } else {
    LOG(ERROR) << "Euroc dataset was not parsed.";
  }
  LOG_IF(INFO, shutdown_) << "EurocDataProvider shutdown requested.";
  return false;
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::spinOnce() {
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

  const Timestamp& timestamp_frame_k = timestampAtFrame(current_k_);
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
}

void EurocDataProvider::sendImuData() const {
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
void EurocDataProvider::parse() {
  VLOG(100) << "Using dataset path: " << dataset_path_;
  // Parse the dataset (ETH format).
  parseDataset();
  if (VLOG_IS_ON(1)) print();

  // Send first ground-truth pose to VIO for initialization if requested.
  if (vio_params_.backend_params_->autoInitialize_ == 0) {
    // We want to initialize from ground-truth.
    vio_params_.backend_params_->initial_ground_truth_state_ =
        getGroundTruthState(timestampAtFrame(initial_k_));
  }
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::parseImuData(const std::string& input_dataset_path,
                                     const std::string& imuName) {
  ///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
  //#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
  // a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
  std::string filename_data =
      input_dataset_path + "/mav0/" + imuName + "/data.csv";
  std::ifstream fin(filename_data.c_str());
  LOG_IF(FATAL, !fin.is_open()) << "Cannot open file: " << filename_data;

  // Skip the first line, containing the header.
  std::string line;
  std::getline(fin, line);

  size_t deltaCount = 0u;
  Timestamp sumOfDelta = 0;
  double stdDelta = 0;
  double imu_rate_maxMismatch = 0;
  double maxNormAcc = 0, maxNormRotRate = 0;  // only for debugging
  Timestamp previous_timestamp = -1;

  // Read/store imu measurements, line by line.
  while (std::getline(fin, line)) {
    Timestamp timestamp = 0;
    gtsam::Vector6 gyr_acc_data;
    for (size_t i = 0u; i < gyr_acc_data.size() + 1u; i++) {
      int idx = line.find_first_of(',');
      if (i == 0) {
        timestamp = std::stoll(line.substr(0, idx));
      } else {
        gyr_acc_data(i - 1) = std::stod(line.substr(0, idx));
      }
      line = line.substr(idx + 1);
    }
    CHECK_GT(timestamp, previous_timestamp)
        << "Euroc IMU data is not in chronological order!";
    Vector6 imu_accgyr;
    // Acceleration first!
    imu_accgyr << gyr_acc_data.tail(3), gyr_acc_data.head(3);

    double normAcc = gyr_acc_data.tail(3).norm();
    if (normAcc > maxNormAcc) maxNormAcc = normAcc;

    double normRotRate = gyr_acc_data.head(3).norm();
    if (normRotRate > maxNormRotRate) maxNormRotRate = normRotRate;

    //! Store imu measurements
    imu_measurements_.push_back(ImuMeasurement(timestamp, imu_accgyr));

    if (previous_timestamp != -1) {
      sumOfDelta += (timestamp - previous_timestamp);
      double deltaMismatch =
          std::fabs(static_cast<double>(
                        timestamp - previous_timestamp -
                        vio_params_.imu_params_.nominal_sampling_time_s_) *
                    1e-9);
      stdDelta += std::pow(deltaMismatch, 2);
      imu_rate_maxMismatch = std::max(imu_rate_maxMismatch, deltaMismatch);
      deltaCount += 1u;
    }
    previous_timestamp = timestamp;
  }

  // Converted to seconds.
  VLOG(10) << "IMU rate: "
           << (static_cast<double>(sumOfDelta) /
               static_cast<double>(deltaCount)) *
                  1e-9
           << '\n'
           << "IMU rate std: "
           << std::sqrt(stdDelta / static_cast<double>(deltaCount - 1u)) << '\n'
           << "IMU rate max mismatch: " << imu_rate_maxMismatch << '\n'
           << "Maximum measured rotation rate (norm):" << maxNormRotRate << '\n'
           << "Maximum measured acceleration (norm): " << maxNormAcc;
  fin.close();

  return true;
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::parseGtData(const std::string& input_dataset_path,
                                    const std::string& gt_sensor_name) {
  CHECK(!input_dataset_path.empty());
  CHECK(!gt_sensor_name.empty());

  std::string filename_sensor =
      input_dataset_path + "/mav0/" + gt_sensor_name + "/sensor.yaml";
  YamlParser yaml_parser(filename_sensor);

  // Rows and cols are redundant info, since the pose 4x4, but we parse just
  // to check we are all on the same page.
  int n_rows = 0;
  yaml_parser.getNestedYamlParam("T_BS", "rows", &n_rows);
  CHECK_EQ(n_rows, 4u);
  int n_cols = 0;
  yaml_parser.getNestedYamlParam("T_BS", "cols", &n_cols);
  CHECK_EQ(n_cols, 4u);
  std::vector<double> vector_pose;
  yaml_parser.getNestedYamlParam("T_BS", "data", &vector_pose);
  gt_data_.body_Pose_prism_ = UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);

  // Sanity check: usually this is the identity matrix as the GT "sensor"
  // is at the body frame: aka body_Pose_prism_ == body_Pose_cam_
  CHECK(gt_data_.body_Pose_prism_.equals(gtsam::Pose3::identity()))
      << "parseGTdata: we expected identity body_Pose_prism_: is everything "
         "ok?";

  ///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
  //#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
  // q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],
  // v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],
  // b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],
  // b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]
  std::string filename_data =
      input_dataset_path + "/mav0/" + gt_sensor_name + "/data.csv";
  std::ifstream fin(filename_data.c_str());
  CHECK(fin.is_open()) << "Cannot open file: " << filename_data << '\n'
                       << "Assuming dataset has no ground truth...";

  // Skip the first line, containing the header.
  std::string line;
  std::getline(fin, line);

  size_t delta_count = 0u;
  Timestamp sumOfDelta = 0;
  Timestamp previous_timestamp = -1;

  // Read/store gt, line by line.
  double max_gt_vel = 0;
  while (std::getline(fin, line)) {
    Timestamp timestamp = 0;
    std::vector<double> gt_data_raw;
    for (size_t i = 0u; i < 17; i++) {
      int idx = line.find_first_of(',');
      if (i == 0u) {
        timestamp = std::stoll(line.substr(0, idx));
      } else {
        gt_data_raw.push_back(std::stod(line.substr(0, idx)));
      }
      line = line.substr(idx + 1);
    }
    if (previous_timestamp == -1) {
      previous_timestamp = timestamp;  // do nothing
    } else {
      sumOfDelta += (timestamp - previous_timestamp);
      // std::cout << "time diff (sec): " << (timestamp - previous_timestamp) *
      // 1e-9 << std::endl;
      delta_count += 1u;
      previous_timestamp = timestamp;
    }

    VioNavState gt_curr;
    gtsam::Point3 position(gt_data_raw[0], gt_data_raw[1], gt_data_raw[2]);
    // Quaternion w x y z.
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
        gt_data_raw[3], gt_data_raw[4], gt_data_raw[5], gt_data_raw[6]);

    // Sanity check.
    gtsam::Vector q = rot.quaternion();
    // Figure out sign for quaternion.
    if (std::fabs(q(0) + gt_data_raw[3]) < std::fabs(q(0) - gt_data_raw[3])) {
      q = -q;
    }

    LOG_IF(FATAL,
           (fabs(q(0) - gt_data_raw[3]) > 1e-3) ||
               (fabs(q(1) - gt_data_raw[4]) > 1e-3) ||
               (fabs(q(2) - gt_data_raw[5]) > 1e-3) ||
               (fabs(q(3) - gt_data_raw[6]) > 1e-3))
        << "parseGTdata: wrong quaternion conversion"
        << "(" << q(0) << "," << gt_data_raw[3] << ") "
        << "(" << q(1) << "," << gt_data_raw[4] << ") "
        << "(" << q(2) << "," << gt_data_raw[5] << ") "
        << "(" << q(3) << "," << gt_data_raw[6] << ").";

    gt_curr.pose_ = gtsam::Pose3(rot, position);
    gt_curr.velocity_ =
        gtsam::Vector3(gt_data_raw[7], gt_data_raw[8], gt_data_raw[9]);
    gtsam::Vector3 gyroBias =
        gtsam::Vector3(gt_data_raw[10], gt_data_raw[11], gt_data_raw[12]);
    gtsam::Vector3 accBias =
        gtsam::Vector3(gt_data_raw[13], gt_data_raw[14], gt_data_raw[15]);
    gt_curr.imu_bias_ = gtsam::imuBias::ConstantBias(accBias, gyroBias);

    gt_data_.map_to_gt_.insert(
        std::pair<Timestamp, VioNavState>(timestamp, gt_curr));

    double norm_vel = gt_curr.velocity_.norm();
    if (norm_vel > max_gt_vel) max_gt_vel = norm_vel;
  }  // End of while loop.

  CHECK_EQ(delta_count, gt_data_.map_to_gt_.size() - 1u)
      << "parseGTdata: wrong nr of deltaCount: deltaCount " << delta_count
      << " nrPoses " << gt_data_.map_to_gt_.size();

  CHECK_NE(delta_count, 0u);
  // Converted in seconds.
  // TODO(TONI): this looks horrible.
  gt_data_.gt_rate_ =
      (static_cast<double>(sumOfDelta) / static_cast<double>(delta_count)) *
      1e-9;
  fin.close();

  VLOG(1) << "Maximum ground truth velocity: " << max_gt_vel;
  return true;
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::parseDataset() {
  // Parse IMU data.
  CHECK(parseImuData(dataset_path_, kImuName));

  // Parse Camera data.
  CameraImageLists left_cam_image_list;
  CameraImageLists right_cam_image_list;
  parseCameraData(kLeftCamName, &left_cam_image_list);
  if (VLOG_IS_ON(1)) left_cam_image_list.print();
  parseCameraData(kRightCamName, &right_cam_image_list);
  if (VLOG_IS_ON(1)) right_cam_image_list.print();
  // TODO(Toni): remove camera_names_ and camera_image_lists_...
  camera_names_.push_back(kLeftCamName);
  camera_names_.push_back(kRightCamName);
  // WARNING Use [x] not .at() because we are adding entries that do not exist.
  camera_image_lists_[kLeftCamName] = left_cam_image_list;
  camera_image_lists_[kRightCamName] = right_cam_image_list;
  // CHECK(sanityCheckCameraData(camera_names_, &camera_image_lists_));

  // Parse Ground-Truth data.
  static const std::string ground_truth_name = "state_groundtruth_estimate0";
  is_gt_available_ = parseGtData(dataset_path_, ground_truth_name);

  clipFinalFrame();

  // Log Ground-Truth data.
  if (logger_) {
    if (is_gt_available_) {
      logger_->logGtData(dataset_path_ + "/mav0/" + ground_truth_name +
                         "/data.csv");
    } else {
      LOG(ERROR) << "Requested ground-truth data logging but no ground-truth "
                    "data available.";
    }
  }

  return true;
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::parseCameraData(const std::string& cam_name,
                                        CameraImageLists* cam_list_i) {
  CHECK_NOTNULL(cam_list_i)
      ->parseCamImgList(dataset_path_ + "/mav0/" + cam_name, "data.csv");
  return true;
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::sanityCheckCameraData(
    const std::vector<std::string>& camera_names,
    std::map<std::string, CameraImageLists>* camera_image_lists) const {
  CHECK_NOTNULL(camera_image_lists);
  CHECK_GT(vio_params_.camera_params_.size(), 0u);
  CHECK_EQ(vio_params_.camera_params_.size(), 2u);
  const auto& left_cam_info = vio_params_.camera_params_.at(0);
  auto& left_img_lists = camera_image_lists->at(camera_names.at(0)).img_lists_;
  auto& right_img_lists = camera_image_lists->at(camera_names.at(1)).img_lists_;
  return sanityCheckCamSize(&left_img_lists, &right_img_lists) &&
         sanityCheckCamTimestamps(
             left_img_lists, right_img_lists, left_cam_info);
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::sanityCheckCamSize(
    CameraImageLists::ImgLists* left_img_lists,
    CameraImageLists::ImgLists* right_img_lists) const {
  CHECK_NOTNULL(left_img_lists);
  CHECK_NOTNULL(right_img_lists);
  size_t nr_left_cam_imgs = left_img_lists->size();
  size_t nr_right_cam_imgs = right_img_lists->size();
  if (nr_left_cam_imgs != nr_right_cam_imgs) {
    LOG(WARNING) << "Different number of images in left and right camera!\n"
                 << "Left: " << nr_left_cam_imgs << "\n"
                 << "Right: " << nr_right_cam_imgs;
    size_t nrCommonImages = std::min(nr_left_cam_imgs, nr_right_cam_imgs);
    left_img_lists->resize(nrCommonImages);
    right_img_lists->resize(nrCommonImages);
  }
  return true;
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::sanityCheckCamTimestamps(
    const CameraImageLists::ImgLists& left_img_lists,
    const CameraImageLists::ImgLists& right_img_lists,
    const CameraParams& left_cam_info) const {
  double stdDelta = 0.0;
  double frame_rate_maxMismatch = 0.0;
  size_t deltaCount = 0u;
  for (size_t i = 0; i < left_img_lists.size(); i++) {
    if (i > 0) {
      deltaCount++;
      const Timestamp& timestamp = left_img_lists.at(i).first;
      const Timestamp& previous_timestamp = left_img_lists.at(i - 1).first;
      // TODO(TONI): this looks horrible.
      double deltaMismatch =
          std::fabs(static_cast<double>(timestamp - previous_timestamp -
                                        left_cam_info.frame_rate_) *
                    1e-9);
      stdDelta += pow(deltaMismatch, 2);
      frame_rate_maxMismatch = std::max(frame_rate_maxMismatch, deltaMismatch);
    }

    LOG_IF(FATAL, left_img_lists.at(i).first != right_img_lists.at(i).first)
        << "Different timestamp for left and right image!\n"
        << "left: " << left_img_lists.at(i).first << '\n'
        << "right: " << right_img_lists.at(i).first << '\n'
        << " for image " << i << " of " << left_img_lists.size();
  }

  CHECK_NE(deltaCount - 1, 0);
  LOG(INFO) << "nominal frame rate: " << left_cam_info.frame_rate_ << '\n'
            << "frame rate std: "
            << std::sqrt(stdDelta / static_cast<double>(deltaCount - 1u))
            << '\n'
            << "frame rate maxMismatch: " << frame_rate_maxMismatch;
  return true;
}

std::string EurocDataProvider::getDatasetName() {
  if (dataset_name_.empty()) {
    // Find and store actual name (rather than path) of the dataset.
    size_t found_last_slash = dataset_path_.find_last_of("/\\");
    std::string dataset_path_tmp = dataset_path_;
    dataset_name_ = dataset_path_tmp.substr(found_last_slash + 1);
    // The dataset name has a slash at the very end
    if (found_last_slash >= dataset_path_tmp.size() - 1) {
      // Cut the last slash.
      dataset_path_tmp = dataset_path_tmp.substr(0, found_last_slash);
      // Repeat the search.
      found_last_slash = dataset_path_tmp.find_last_of("/\\");
      // Try to pick right name.
      dataset_name_ = dataset_path_tmp.substr(found_last_slash + 1);
    }
    LOG(INFO) << "Dataset name: " << dataset_name_;
  }
  return dataset_name_;
}

/* -------------------------------------------------------------------------- */
gtsam::Pose3 EurocDataProvider::getGroundTruthRelativePose(
    const Timestamp& previousTimestamp,
    const Timestamp& currentTimestamp) const {
  gtsam::Pose3 previousPose = getGroundTruthPose(previousTimestamp);
  gtsam::Pose3 currentPose = getGroundTruthPose(currentTimestamp);
  return previousPose.between(currentPose);
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::isGroundTruthAvailable(
    const Timestamp& timestamp) const {
  return timestamp > gt_data_.map_to_gt_.begin()->first;
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::isGroundTruthAvailable() const {
  return is_gt_available_;
}

/* -------------------------------------------------------------------------- */
VioNavState EurocDataProvider::getGroundTruthState(
    const Timestamp& timestamp) const {
  const auto& it_low =
      gt_data_.map_to_gt_.equal_range(timestamp).first;  // closest, non-lesser

  // Sanity check.
  double delta_low = static_cast<double>(it_low->first - timestamp) * 1e-9;
  const auto& it_begin = gt_data_.map_to_gt_.begin();
  LOG_IF(FATAL, timestamp > it_begin->first && delta_low > 0.01)
      << "getGroundTruthState: something wrong " << delta_low;
  return it_low->second;
}

/* -------------------------------------------------------------------------- */
// Compute initialization errors and stats.
// [in]: timestamp vector for poses in bundle-adjustment
// [in]: vector of poses from bundle-adjustment. the initial
// pose is identity (we are interested in relative poses!)
// [in]: initial nav state with pose, velocity in body frame,
// [in]: gravity vector estimate in body frame.
const InitializationPerformance EurocDataProvider::getInitializationPerformance(
    const std::vector<Timestamp>& timestamps,
    const std::vector<gtsam::Pose3>& poses_ba,
    const VioNavState& init_nav_state,
    const gtsam::Vector3& init_gravity) {
  CHECK(isGroundTruthAvailable());
  CHECK_EQ(timestamps.size(), poses_ba.size());
  // The first BA pose must be identity and can be discarded

  // GT and performance variables to fill
  int init_n_frames = timestamps.size() - 1;
  double avg_relativeRotError = 0.0;
  double avg_relativeTranError = 0.0;
  gtsam::Pose3 gt_relative;
  VioNavState gt_state = getGroundTruthState(timestamps.at(1));
  gtsam::Vector3 gt_gravity = gtsam::Vector3(0.0, 0.0, -9.81);
  // Assumes gravity vector is downwards

  // Loop through bundle adjustment poses and get GT
  for (int i = 1; i < timestamps.size(); i++) {
    double relativeRotError = -1;
    double relativeTranError = -1;
    // Fill relative poses from GT
    // Check that GT is available
    if (!isGroundTruthAvailable(timestamps.at(i - 1)) ||
        !isGroundTruthAvailable(timestamps.at(i))) {
      LOG(FATAL) << "GT Timestamp not available!";
    }
    gt_relative =
        getGroundTruthRelativePose(timestamps.at(i - 1), timestamps.at(i));
    // Get relative pose from BA
    gtsam::Pose3 ba_relative = poses_ba.at(i - 1).between(poses_ba.at(i));
    // Compute errors between BA and GT
    std::tie(relativeRotError, relativeTranError) =
        UtilsOpenCV::ComputeRotationAndTranslationErrors(
            gt_relative, ba_relative, false);
    avg_relativeRotError += fabs(relativeRotError);
    avg_relativeTranError += fabs(relativeTranError);
  }
  // Compute average logmap error and translation error
  CHECK_NE(init_n_frames, 0);
  avg_relativeRotError =
      avg_relativeRotError / static_cast<double>(init_n_frames);
  avg_relativeTranError =
      avg_relativeTranError / static_cast<double>(init_n_frames);

  LOG(INFO) << "avg. rot. error\n"
            << avg_relativeRotError << "\navg. tran. error\n"
            << avg_relativeTranError;

  // Convert velocities and gravity vector in initial body frame.
  // This is already the case for the init nav state passed.
  gt_state.velocity_ =
      gt_state.pose_.rotation().transpose() * gt_state.velocity_;
  gt_gravity = gt_state.pose_.rotation().transpose() * gt_gravity;

  LOG(INFO) << "GT: init pose\n"
            << gt_state.pose_.rotation() << "\n init velocity\n"
            << gt_state.velocity_ << "\n init bias\n"
            << gt_state.imu_bias_.gyroscope() << "\n init gravity\n"
            << gt_gravity;

  // Create performance overview
  const InitializationPerformance init_performance(timestamps.at(1),
                                                   init_n_frames,
                                                   avg_relativeRotError,
                                                   avg_relativeTranError,
                                                   init_nav_state,
                                                   init_gravity,
                                                   gt_state,
                                                   gt_gravity);
  // Log performance
  init_performance.print();
  // Return
  return init_performance;
}

/* -------------------------------------------------------------------------- */
size_t EurocDataProvider::getNumImages() const {
  CHECK_GT(camera_names_.size(), 0u);
  const std::string& left_cam_name = camera_names_.at(0);
  const std::string& right_cam_name = camera_names_.at(0);
  size_t n_left_images = getNumImagesForCamera(left_cam_name);
  size_t n_right_images = getNumImagesForCamera(right_cam_name);
  CHECK_EQ(n_left_images, n_right_images);
  return n_left_images;
}

/* -------------------------------------------------------------------------- */
size_t EurocDataProvider::getNumImagesForCamera(
    const std::string& camera_name) const {
  const auto& iter = camera_image_lists_.find(camera_name);
  CHECK(iter != camera_image_lists_.end());
  return iter->second.getNumImages();
}

/* -------------------------------------------------------------------------- */
bool EurocDataProvider::getImgName(const std::string& camera_name,
                                   const size_t& k,
                                   std::string* img_filename) const {
  CHECK_NOTNULL(img_filename);
  const auto& iter = camera_image_lists_.find(camera_name);
  CHECK(iter != camera_image_lists_.end());
  const auto& img_lists = iter->second.img_lists_;
  if (k < img_lists.size()) {
    *img_filename = img_lists.at(k).second;
    return true;
  } else {
    LOG(ERROR) << "Requested image #: " << k << " but we only have "
               << img_lists.size() << " images.";
  }
  return false;
}

/* -------------------------------------------------------------------------- */
Timestamp EurocDataProvider::timestampAtFrame(const FrameId& frame_number) {
  CHECK_GT(camera_names_.size(), 0);
  CHECK_LT(frame_number,
           camera_image_lists_.at(camera_names_[0]).img_lists_.size());
  return camera_image_lists_.at(camera_names_[0])
      .img_lists_[frame_number]
      .first;
}

void EurocDataProvider::clipFinalFrame() {
  // Clip final_k_ to the total number of images.
  const size_t& nr_images = getNumImages();
  if (final_k_ > nr_images) {
    LOG(WARNING) << "Value for final_k, " << final_k_ << " is larger than total"
                 << " number of frames in dataset " << nr_images;
    final_k_ = nr_images;
    LOG(WARNING) << "Using final_k = " << final_k_;
  }
  CHECK_LE(final_k_, nr_images);
}
/* -------------------------------------------------------------------------- */
void EurocDataProvider::print() const {
  LOG(INFO) << "------------------ ETHDatasetParser::print ------------------\n"
            << "Displaying info for dataset: " << dataset_path_;
  // For each of the 2 cameras.
  CHECK_EQ(vio_params_.camera_params_.size(), camera_names_.size());
  for (size_t i = 0; i < camera_names_.size(); i++) {
    LOG(INFO) << "\n"
              << (i == 0 ? "Left" : "Right")
              << " camera name: " << camera_names_[i] << ", with params:\n";
    vio_params_.camera_params_.at(i).print();
    camera_image_lists_.at(camera_names_[i]).print();
  }
  if (FLAGS_minloglevel < 1) {
    gt_data_.print();
  }
  LOG(INFO) << "-------------------------------------------------------------";
}

//////////////////////////////////////////////////////////////////////////////

/* -------------------------------------------------------------------------- */
MonoEurocDataProvider::MonoEurocDataProvider(const std::string& dataset_path,
                                             const int& initial_k,
                                             const int& final_k,
                                             const VioParams& vio_params)
    : EurocDataProvider(dataset_path, initial_k, final_k, vio_params) {}

/* -------------------------------------------------------------------------- */
MonoEurocDataProvider::MonoEurocDataProvider(const VioParams& vio_params)
    : EurocDataProvider(vio_params) {}

/* -------------------------------------------------------------------------- */
MonoEurocDataProvider::~MonoEurocDataProvider() {
  LOG(INFO) << "Mono ETHDataParser destructor called.";
}

/* -------------------------------------------------------------------------- */
bool MonoEurocDataProvider::spin() {
  if (dataset_parsed_) {
    if (!is_imu_data_sent_) {
      // First, send all the IMU data. The flag is to avoid sending it several
      // times if we are running in sequential mode.
      if (imu_single_callback_) {
        sendImuData();
      } else {
        LOG(ERROR) << "Imu callback not registered! Not sending IMU data.";
      }
      is_imu_data_sent_ = true;
    }

    // Spin.
    CHECK_EQ(vio_params_.camera_params_.size(), 2u);
    CHECK_GT(final_k_, initial_k_);
    // We log only the first one, because we may be running in sequential mode.
    LOG_FIRST_N(INFO, 1) << "Running dataset between frame " << initial_k_
                         << " and frame " << final_k_;
    while (!shutdown_ && spinOnce()) {
      if (!vio_params_.parallel_run_) {
        // Return, instead of blocking, when running in sequential mode.
        return true;
      }
    }
  } else {
    LOG(ERROR) << "Euroc dataset was not parsed.";
  }
  LOG_IF(INFO, shutdown_) << "EurocDataProvider shutdown requested.";
  return false;
}

/* -------------------------------------------------------------------------- */
bool MonoEurocDataProvider::spinOnce() {
  CHECK_LT(current_k_, std::numeric_limits<FrameId>::max())
      << "Are you sure you've initialized current_k_?";
  if (current_k_ >= final_k_) {
    LOG(INFO) << "Finished spinning Euroc dataset.";
    return false;
  }

  const CameraParams& left_cam_info = vio_params_.camera_params_.at(0);
  const bool& equalize_image =
      vio_params_.frontend_params_.stereo_matching_params_.equalize_image_;

  const Timestamp& timestamp_frame_k = timestampAtFrame(current_k_);
  VLOG(10) << "Sending left frame k= " << current_k_
           << " with timestamp: " << timestamp_frame_k;

  // TODO(Toni): ideally only send cv::Mat raw images...:
  // - pass params to vio_pipeline ctor
  // - make vio_pipeline actually equalize or transform images as necessary.
  std::string left_img_filename;
  bool available_left_img = getLeftImgName(current_k_, &left_img_filename);
  if (available_left_img) {
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
  } else {
    LOG(ERROR) << "Missing left image, proceeding to the next one.";
  }

  // This is done directly when parsing the Imu data.
  // imu_single_callback_(imu_meas);

  VLOG(10) << "Finished VIO processing for frame k = " << current_k_;
  current_k_++;
  return true;
}

}  // namespace VIO
