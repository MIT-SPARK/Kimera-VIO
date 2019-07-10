/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ETH_parser.h
 * @brief  Reads ETH's Euroc dataset.
 * @author Antoni Rosinol, Luca Carlone, Yun Chang
 */

#include "datasource/ETH_parser.h"

#include "ImuFrontEnd-definitions.h"
#include "StereoFrame.h"

DEFINE_int32(skip_n_start_frames, 10, "Number of initial frames to skip.");
DEFINE_int32(skip_n_end_frames, 100, "Number of final frames to skip.");

namespace VIO {

/* -------------------------------------------------------------------------- */
ETHDatasetParser::ETHDatasetParser() : DataProvider(), imuData_() {
  parse();

  // Check that final_k_ is smaller than the number of images.
  // And remove final frames.
  const size_t& nr_images = getNumImages();
  if (final_k_ >= nr_images) {
    LOG(WARNING) << "Value for final_k, " << final_k_
                 << " is larger or equal to the total"
                 << " number of frames in dataset " << nr_images;
    // Skip last frames which are typically problematic
    // (IMU bumps, FOV occluded)...
    CHECK_GT(FLAGS_skip_n_end_frames, 0);
    CHECK_LT(FLAGS_skip_n_end_frames, nr_images);
    final_k_ = nr_images - FLAGS_skip_n_end_frames;
    LOG(WARNING) << "Using final_k = " << final_k_ << ", where we removed "
                 << FLAGS_skip_n_end_frames
                 << " frames to avoid bad IMU readings.";
  }
  CHECK_GT(final_k_, 0);
  CHECK_LT(final_k_, nr_images);
}

/* -------------------------------------------------------------------------- */
ETHDatasetParser::~ETHDatasetParser() {
  LOG(INFO) << "ETHDatasetParser destructor called.";
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::spin() {
  // Check that the user correctly registered a callback function for the
  // pipeline.
  CHECK(vio_callback_) << "Missing VIO callback registration. Call "
                          " registerVioCallback before spinning the dataset.";
  // Check also that the dataset has been correctly parsed.

  // Timestamp 10 frames before the first (for imu calibration)
  CHECK_GE(initial_k_, FLAGS_skip_n_start_frames)
      << "Initial frame " << initial_k_ << " has to be larger than "
      << FLAGS_skip_n_start_frames << " (needed for IMU calibration)";
  Timestamp timestamp_last_frame =
      timestampAtFrame(initial_k_ - FLAGS_skip_n_start_frames);

  // Spin.
  const StereoMatchingParams& stereo_matching_params =
      frontend_params_.getStereoMatchingParams();
  const bool equalize_image = stereo_matching_params.equalize_image_;
  const CameraParams& left_cam_info = getLeftCamInfo();
  const CameraParams& right_cam_info = getRightCamInfo();
  const gtsam::Pose3& camL_pose_camR = getCamLPoseCamR();
  for (FrameId k = initial_k_; k < final_k_; k++) {
    spinOnce(k, timestamp_last_frame, stereo_matching_params, equalize_image,
             left_cam_info, right_cam_info, camL_pose_camR);
  }
  return true;
}

void ETHDatasetParser::spinOnce(
    const FrameId& k, Timestamp& timestamp_last_frame,
    const StereoMatchingParams& stereo_matching_params,
    const bool equalize_image, const CameraParams& left_cam_info,
    const CameraParams& right_cam_info, const gtsam::Pose3& camL_pose_camR) {
  Timestamp timestamp_frame_k = timestampAtFrame(k);
  ImuMeasurements imu_meas;
  CHECK_LT(timestamp_last_frame, timestamp_frame_k);
  CHECK(utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable ==
        imuData_.imu_buffer_.getImuDataInterpolatedUpperBorder(
            timestamp_last_frame, timestamp_frame_k, &imu_meas.timestamps_,
            &imu_meas.measurements_));

  VLOG(10) << "////////////////////////////////////////// Creating packet!\n"
           << "STAMPS IMU rows : \n"
           << imu_meas.timestamps_.rows() << '\n'
           << "STAMPS IMU cols : \n"
           << imu_meas.timestamps_.cols() << '\n'
           << "STAMPS IMU: \n"
           << imu_meas.timestamps_ << '\n'
           << "ACCGYR IMU rows : \n"
           << imu_meas.measurements_.rows() << '\n'
           << "ACCGYR IMU cols : \n"
           << imu_meas.measurements_.cols() << '\n'
           << "ACCGYR IMU: \n"
           << imu_meas.measurements_;

  timestamp_last_frame = timestamp_frame_k;

  // TODO(Toni) remove this, it's just because the logging in the pipeline needs
  // it, but totally useless...
  static bool do_once = true;
  if (do_once) {
    timestamp_first_lkf_ = timestamp_last_frame;
    do_once = false;
  }

  // TODO(Toni) alternatively push here to a queue, and give that queue to the
  // VIO pipeline so it can pull from it.
  // Call VIO Pipeline.
  VLOG(10) << "Call VIO processing for frame k: " << k
           << " with timestamp: " << timestamp_frame_k;
  vio_callback_(StereoImuSyncPacket(
      StereoFrame(k, timestamp_frame_k,
                  UtilsOpenCV::ReadAndConvertToGrayScale(getLeftImgName(k),
                                                         equalize_image),
                  left_cam_info,
                  UtilsOpenCV::ReadAndConvertToGrayScale(getRightImgName(k),
                                                         equalize_image),
                  right_cam_info, camL_pose_camR, stereo_matching_params),
      imu_meas.timestamps_, imu_meas.measurements_));
  VLOG(10) << "Finished VIO processing for frame k = " << k;
}

/* -------------------------------------------------------------------------- */
void ETHDatasetParser::parse() {
  VLOG(100) << "Using dataset path: " << dataset_path_;
  // Parse the dataset (ETH format).
  static const std::string left_cam_name = "cam0";
  static const std::string right_cam_name = "cam1";
  static const std::string imu_name = "imu0";
  static const std::string ground_truth_name = "state_groundtruth_estimate0";
  parseDataset(dataset_path_, left_cam_name, right_cam_name, imu_name,
               ground_truth_name);
  print();
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::parseImuParams(const std::string& input_dataset_path,
                                      const std::string& imuName) {
  std::string filename_sensor =
      input_dataset_path + "/mav0/" + imuName + "/sensor.yaml";
  // parse sensor parameters
  // make sure that each YAML file has %YAML:1.0 as first line
  cv::FileStorage fs;
  UtilsOpenCV::safeOpenCVFileStorage(&fs, filename_sensor);

  // body_Pose_cam_: sensor to body transformation
  int n_rows(fs["T_BS"]["rows"]);
  int n_cols(fs["T_BS"]["cols"]);
  std::vector<double> vec;
  fs["T_BS"]["data"] >> vec;
  auto body_Pose_cam = UtilsOpenCV::Vec2pose(vec, n_rows, n_cols);

  // sanity check: IMU is usually chosen as the body frame
  gtsam::Pose3 identityPose;
  LOG_IF(FATAL, !body_Pose_cam.equals(gtData_.body_Pose_cam_))
      << "parseImuData: we expected identity body_Pose_cam_: is everything ok?";

  // TODO(Toni) REMOVE THIS PARSING.
  int rate = fs["rate_hz"];
  CHECK_NE(rate, 0);
  imuData_.nominal_imu_rate_ = 1 / static_cast<double>(rate);

  imu_params_.gyro_noise_ = fs["gyroscope_noise_density"];
  imu_params_.gyro_walk_ = fs["gyroscope_random_walk"];
  imu_params_.acc_noise_ = fs["accelerometer_noise_density"];
  imu_params_.acc_walk_ = fs["accelerometer_random_walk"];

  fs.release();
  return true;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::parseImuData(const std::string& input_dataset_path,
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
    gtsam::Vector6 gyroAccData;
    for (size_t i = 0; i < 7; i++) {
      int idx = line.find_first_of(',');
      if (i == 0) {
        timestamp = std::stoll(line.substr(0, idx));
      } else {
        gyroAccData(i - 1) = std::stod(line.substr(0, idx));
      }
      line = line.substr(idx + 1);
    }
    Vector6 imu_accgyr;
    // Acceleration first!
    imu_accgyr << gyroAccData.tail(3), gyroAccData.head(3);

    double normAcc = gyroAccData.tail(3).norm();
    if (normAcc > maxNormAcc) maxNormAcc = normAcc;

    double normRotRate = gyroAccData.head(3).norm();
    if (normRotRate > maxNormRotRate) maxNormRotRate = normRotRate;

    imuData_.imu_buffer_.addMeasurement(timestamp, imu_accgyr);
    if (previous_timestamp == -1) {
      // Do nothing.
      previous_timestamp = timestamp;
    } else {
      sumOfDelta += (timestamp - previous_timestamp);
      double deltaMismatch = std::fabs(
          double(timestamp - previous_timestamp - imuData_.nominal_imu_rate_) *
          1e-9);
      stdDelta += std::pow(deltaMismatch, 2);
      imu_rate_maxMismatch = std::max(imu_rate_maxMismatch, deltaMismatch);
      deltaCount += 1u;
      previous_timestamp = timestamp;
    }
  }

  LOG_IF(FATAL, deltaCount != imuData_.imu_buffer_.size() - 1u)
      << "parseImuData: wrong nr of deltaCount: deltaCount " << deltaCount
      << " nr lines " << imuData_.imu_buffer_.size();

  // Converted to seconds.
  imuData_.imu_rate_ =
      (static_cast<double>(sumOfDelta) / static_cast<double>(deltaCount)) *
      1e-9;
  imuData_.imu_rate_std_ =
      std::sqrt(stdDelta / static_cast<double>(deltaCount - 1u));
  imuData_.imu_rate_maxMismatch_ = imu_rate_maxMismatch;
  fin.close();

  LOG(INFO) << "Maximum measured rotation rate (norm):" << maxNormRotRate << '-'
            << "Maximum measured acceleration (norm): " << maxNormAcc;
  return true;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::parseGTdata(const std::string& input_dataset_path,
                                   const std::string& gtSensorName) {
  ///////////////// PARSE SENSOR FILE //////////////////////////////////////////
  std::string filename_sensor =
      input_dataset_path + "/mav0/" + gtSensorName + "/sensor.yaml";

  // Make sure that each YAML file has %YAML:1.0 as first line.
  cv::FileStorage fs;
  UtilsOpenCV::safeOpenCVFileStorage(&fs, filename_sensor, false);
  if (!fs.isOpened()) {
    LOG(WARNING) << "Cannot open file in parseGTYAML: " << filename_sensor
                 << "\nAssuming dataset has no ground truth...";
    return false;
  }

  // body_Pose_cam_: usually this is the identity matrix as the GT "sensor" is
  // at the body frame
  int n_rows(fs["T_BS"]["rows"]);
  int n_cols(fs["T_BS"]["cols"]);

  std::vector<double> vec;
  fs["T_BS"]["data"] >> vec;
  gtData_.body_Pose_cam_ = UtilsOpenCV::Vec2pose(vec, n_rows, n_cols);

  // sanity check: usually this is the identity matrix as the GT "sensor"
  // is at the body frame
  gtsam::Pose3 identityPose;

  LOG_IF(FATAL, !gtData_.body_Pose_cam_.equals(identityPose))
      << "parseGTdata: we expected identity body_Pose_cam_: is everything ok?";

  fs.release();

  ///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
  //#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
  // q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],
  // v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],
  // b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],
  // b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]
  std::string filename_data =
      input_dataset_path + "/mav0/" + gtSensorName + "/data.csv";
  std::ifstream fin(filename_data.c_str());
  LOG_IF(FATAL, !fin.is_open()) << "Cannot open file: " << filename_data << '\n'
                                << "Assuming dataset has no ground truth...";

  // Skip the first line, containing the header.
  std::string line;
  std::getline(fin, line);

  size_t deltaCount = 0u;
  Timestamp sumOfDelta = 0;
  Timestamp previous_timestamp = -1;

  // Read/store gt, line by line.
  double maxGTvel = 0;
  while (std::getline(fin, line)) {
    Timestamp timestamp = 0;
    std::vector<double> gtDataRaw;
    for (size_t i = 0; i < 17; i++) {
      int idx = line.find_first_of(',');
      if (i == 0)
        timestamp = std::stoll(line.substr(0, idx));
      else
        gtDataRaw.push_back(std::stod(line.substr(0, idx)));
      line = line.substr(idx + 1);
    }
    if (previous_timestamp == -1) {
      previous_timestamp = timestamp;  // do nothing
    } else {
      sumOfDelta += (timestamp - previous_timestamp);
      // std::cout << "time diff (sec): " << (timestamp - previous_timestamp) *
      // 1e-9 << std::endl;
      deltaCount += 1u;
      previous_timestamp = timestamp;
    }

    gtNavState gt_curr;
    gtsam::Point3 position(gtDataRaw[0], gtDataRaw[1], gtDataRaw[2]);
    // Quaternion w x y z.
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(gtDataRaw[3], gtDataRaw[4],
                                              gtDataRaw[5], gtDataRaw[6]);

    // Sanity check.
    gtsam::Vector q = rot.quaternion();
    // Figure out sign for quaternion.
    if (std::fabs(q(0) + gtDataRaw[3]) < std::fabs(q(0) - gtDataRaw[3])) {
      q = -q;
    }

    LOG_IF(FATAL, (fabs(q(0) - gtDataRaw[3]) > 1e-3) ||
                      (fabs(q(1) - gtDataRaw[4]) > 1e-3) ||
                      (fabs(q(2) - gtDataRaw[5]) > 1e-3) ||
                      (fabs(q(3) - gtDataRaw[6]) > 1e-3))
        << "parseGTdata: wrong quaternion conversion"
        << "(" << q(0) << "," << gtDataRaw[3] << ") "
        << "(" << q(1) << "," << gtDataRaw[4] << ") "
        << "(" << q(2) << "," << gtDataRaw[5] << ") "
        << "(" << q(3) << "," << gtDataRaw[6] << ").";

    gt_curr.pose_ = gtsam::Pose3(rot, position).compose(gtData_.body_Pose_cam_);
    gt_curr.velocity_ =
        gtsam::Vector3(gtDataRaw[7], gtDataRaw[8], gtDataRaw[9]);
    gtsam::Vector3 gyroBias =
        gtsam::Vector3(gtDataRaw[10], gtDataRaw[11], gtDataRaw[12]);
    gtsam::Vector3 accBias =
        gtsam::Vector3(gtDataRaw[13], gtDataRaw[14], gtDataRaw[15]);
    gt_curr.imu_bias_ = gtsam::imuBias::ConstantBias(accBias, gyroBias);

    gtData_.mapToGt_.insert(
        std::pair<Timestamp, gtNavState>(timestamp, gt_curr));

    double normVel = gt_curr.velocity_.norm();
    if (normVel > maxGTvel) maxGTvel = normVel;
  }  // End of while loop.

  LOG_IF(FATAL, deltaCount != gtData_.mapToGt_.size() - 1u)
      << "parseGTdata: wrong nr of deltaCount: deltaCount " << deltaCount
      << " nrPoses " << gtData_.mapToGt_.size();

  CHECK_NE(deltaCount, 0u);
  // Converted in seconds.
  gtData_.gt_rate_ = (double(sumOfDelta) / double(deltaCount)) * 1e-9;
  fin.close();

  LOG(INFO) << "Maximum ground truth velocity: " << maxGTvel;
  return true;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::parseDataset(const std::string& input_dataset_path,
                                    const std::string& leftCameraName,
                                    const std::string& rightCameraName,
                                    const std::string& imuName,
                                    const std::string& gtSensorName,
                                    bool doParseImages) {
  parseCameraData(input_dataset_path, leftCameraName, rightCameraName,
                  doParseImages);
  CHECK(parseImuParams(input_dataset_path, imuName));
  parseImuData(input_dataset_path, imuName);
  is_gt_available_ = parseGTdata(input_dataset_path, gtSensorName);

  // Find and store actual name (rather than path) of the dataset.
  std::size_t foundLastSlash = input_dataset_path.find_last_of("/\\");
  std::string dataset_path_tmp = input_dataset_path;
  dataset_name_ = dataset_path_tmp.substr(foundLastSlash + 1);
  if (foundLastSlash >=
      dataset_path_tmp.size() -
          1) {  // the dataset name has a slash at the very end
    dataset_path_tmp =
        dataset_path_tmp.substr(0, foundLastSlash);  // cut the last slash
    foundLastSlash = dataset_path_tmp.find_last_of("/\\");  // repeat the search
    dataset_name_ =
        dataset_path_tmp.substr(foundLastSlash + 1);  // try to pick right name
  }
  LOG(INFO) << "Dataset name: " << dataset_name_;
  return true;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::parseCameraData(const std::string& input_dataset_path,
                                       const std::string& left_cam_name,
                                       const std::string& right_cam_name,
                                       const bool parse_imgs) {
  // Default names: match names of the corresponding folders.
  camera_names_.resize(2);
  camera_names_[0] = left_cam_name;
  camera_names_[1] = right_cam_name;

  // Read camera info and list of images.
  camera_info_.clear();
  camera_image_lists_.clear();
  for (const std::string& cam_name : camera_names_) {
    LOG(INFO) << "reading camera: " << cam_name;
    CameraParams cam_info_i;
    cam_info_i.parseYAML(input_dataset_path + "/mav0/" + cam_name +
                         "/sensor.yaml");
    camera_info_[cam_name] = cam_info_i;

    CameraImageLists cam_list_i;
    if (parse_imgs) {
      cam_list_i.parseCamImgList(input_dataset_path + "/mav0/" + cam_name,
                                 "data.csv");
    }
    camera_image_lists_[cam_name] = cam_list_i;
  }

  CHECK(
      sanityCheckCameraData(camera_names_, camera_info_, &camera_image_lists_));

  // Set extrinsic for the stereo.
  const CameraParams& left_camera_info = camera_info_.at(camera_names_.at(0));
  const CameraParams& right_camera_info = camera_info_.at(camera_names_.at(1));

  // Extrinsics of the stereo (not rectified)
  // relative pose between cameras
  camL_Pose_camR_ = (left_camera_info.body_Pose_cam_)
                        .between(right_camera_info.body_Pose_cam_);
  return true;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::sanityCheckCameraData(
    const std::vector<std::string>& camera_names,
    const std::map<std::string, CameraParams>& camera_info,
    std::map<std::string, CameraImageLists>* camera_image_lists) const {
  CHECK_NOTNULL(camera_image_lists);
  const auto& left_cam_info = camera_info.at(camera_names.at(0));
  auto& left_img_lists = camera_image_lists->at(camera_names.at(0)).img_lists;
  auto& right_img_lists = camera_image_lists->at(camera_names.at(1)).img_lists;
  return sanityCheckCamSize(&left_img_lists, &right_img_lists) &&
         sanityCheckCamTimestamps(left_img_lists, right_img_lists,
                                  left_cam_info);
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::sanityCheckCamSize(
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
bool ETHDatasetParser::sanityCheckCamTimestamps(
    const CameraImageLists::ImgLists& left_img_lists,
    const CameraImageLists::ImgLists& right_img_lists,
    const CameraParams& left_cam_info) const {
  double stdDelta = 0;
  double frame_rate_maxMismatch = 0;
  size_t deltaCount = 0u;
  for (size_t i = 0; i < left_img_lists.size(); i++) {
    if (i > 0) {
      deltaCount++;
      const Timestamp& timestamp = left_img_lists.at(i).first;
      const Timestamp& previous_timestamp = left_img_lists.at(i - 1).first;
      double deltaMismatch = fabs(
          double(timestamp - previous_timestamp - left_cam_info.frame_rate_) *
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

  LOG(INFO) << "nominal frame rate: " << left_cam_info.frame_rate_ << '\n'
            << "frame rate std: "
            << std::sqrt(stdDelta / double(deltaCount - 1u)) << '\n'
            << "frame rate maxMismatch: " << frame_rate_maxMismatch;
  return true;
}

/* -------------------------------------------------------------------------- */
gtsam::Pose3 ETHDatasetParser::getGroundTruthRelativePose(
    const Timestamp& previousTimestamp,
    const Timestamp& currentTimestamp) const {
  gtsam::Pose3 previousPose = getGroundTruthPose(previousTimestamp);
  gtsam::Pose3 currentPose = getGroundTruthPose(currentTimestamp);
  return previousPose.between(currentPose);
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::isGroundTruthAvailable(
    const Timestamp& timestamp) const {
  auto it_begin = gtData_.mapToGt_.begin();
  return timestamp > it_begin->first;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::isGroundTruthAvailable() const {
  return is_gt_available_;
}

/* -------------------------------------------------------------------------- */
gtNavState ETHDatasetParser::getGroundTruthState(
    const Timestamp& timestamp) const {
  auto it_low_up = gtData_.mapToGt_.equal_range(timestamp);
  auto it_low = it_low_up.first;  // closest, non-lesser

  // Sanity check.
  double delta_low = double(it_low->first - timestamp) * 1e-9;
  auto it_begin = gtData_.mapToGt_.begin();
  LOG_IF(FATAL, timestamp > it_begin->first && delta_low > 0.01)
      << "\n getGroundTruthState: something wrong " << delta_low;
  return it_low->second;
}

/* -------------------------------------------------------------------------- */
std::pair<double, double> ETHDatasetParser::computePoseErrors(
    const gtsam::Pose3& lkf_T_k_body, const bool isTrackingValid,
    const Timestamp& previousTimestamp, const Timestamp& currentTimestamp,
    const bool upToScale) const {
  double relativeRotError = -1.0;
  double relativeTranError = -1.0;
  if (isTrackingValid && isGroundTruthAvailable(previousTimestamp)) {
    gtsam::Pose3 lkf_T_k_gt =
        getGroundTruthRelativePose(previousTimestamp, currentTimestamp);
    // Compute errors.
    std::tie(relativeRotError, relativeTranError) =
        UtilsOpenCV::ComputeRotationAndTranslationErrors(
            lkf_T_k_gt, lkf_T_k_body, upToScale);
  }
  return std::make_pair(relativeRotError, relativeTranError);
}

/* -------------------------------------------------------------------------- */
Timestamp ETHDatasetParser::timestampAtFrame(const FrameId& frame_number) {
  DCHECK_LT(frame_number,
            camera_image_lists_[camera_names_[0]].img_lists.size());
  return camera_image_lists_[camera_names_[0]].img_lists[frame_number].first;
}

/* -------------------------------------------------------------------------- */
void ETHDatasetParser::print() const {
  LOG(INFO) << "-------------------------------------------------------------\n"
            << "------------------ ETHDatasetParser::print ------------------\n"
            << "-------------------------------------------------------------\n"
            << "Displaying info for dataset: " << dataset_path_;
  camL_Pose_camR_.print("camL_Pose_calR \n");
  // For each of the 2 cameras.
  for (size_t i = 0; i < camera_names_.size(); i++) {
    LOG(INFO) << "\n"
              << (i == 0 ? "Left" : "Right")
              << " camera name: " << camera_names_[i] << ", with params:\n";
    camera_info_.at(camera_names_[i]).print();
    camera_image_lists_.at(camera_names_[i]).print();
  }
  gtData_.print();
  imuData_.print();
  LOG(INFO) << "-------------------------------------------------------------\n"
            << "-------------------------------------------------------------\n"
            << "-------------------------------------------------------------";
}

};  // namespace VIO
