#include <functional>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <eigen3/Eigen/Dense>

#include "kimera-vio/pipeline/Pipeline.h"

#include "../common/plugin.hpp"
#include "../common/switchboard.hpp"
#include "../common/data_format.hpp"
#include "../common/phonebook.hpp"

using namespace ILLIXR;

std::string get_path() {
    const char* KIMERA_ROOT_c_str = std::getenv("KIMERA_ROOT");
	if (!KIMERA_ROOT_c_str) {
		std::cerr << "Please define KIMERA_ROOT" << std::endl;
		abort();
	}
	std::string KIMERA_ROOT = std::string{KIMERA_ROOT_c_str};
    return KIMERA_ROOT + "params/ILLIXR";
}

class kimera_vio : public plugin {
public:
	/* Provide handles to kimera_vio */
	kimera_vio(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, kimera_current_frame_id(0)
		, kimera_pipeline_params(get_path())
		, kimera_pipeline(kimera_pipeline_params)
    	, _m_pose{sb->publish<pose_type>("slow_pose")}
    	, _m_imu_integrator_input{sb->publish<imu_integrator_input>("imu_integrator_input")}
	{
		_m_begin = std::chrono::system_clock::now();
		imu_cam_buffer = NULL;
    
		// TODO: read Kimera flag file path from runner and find a better way of passing it to gflag
		kimera_pipeline.registerBackendOutputCallback(
		std::bind(
			&kimera_vio::pose_callback,
			this,
			std::placeholders::_1
		)
		);

		_m_pose->put(
			new pose_type{
				.sensor_time = std::chrono::time_point<std::chrono::system_clock>{},
				.position = Eigen::Vector3f{0, 0, 0},
				.orientation = Eigen::Quaternionf{1, 0, 0, 0}
			}
		);

#ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{"-1"});
#endif
	}


	virtual void start() override {
		plugin::start();
		sb->schedule<imu_cam_type>(id, "imu_cam", [&](const imu_cam_type *datum) {
			this->feed_imu_cam(datum);
		});
	}


	std::size_t iteration_no = 0;
	void feed_imu_cam(const imu_cam_type *datum) {
		// Ensures that slam doesnt start before valid IMU readings come in
		if (datum == NULL) {
			assert(previous_timestamp == 0);
			return;
		}

		// This ensures that every data point is coming in chronological order If youre failing this assert, 
		// make sure that your data folder matches the name in offline_imu_cam/plugin.cc
		assert(datum->dataset_time > previous_timestamp);
		previous_timestamp = datum->dataset_time;

		imu_cam_buffer = datum;

		VIO::Vector6 imu_raw_vals;
		imu_raw_vals << datum->linear_a.cast<double>(), datum->angular_v.cast<double>();

		// Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
		assert((datum->img0.has_value() && datum->img1.has_value()) || (!datum->img0.has_value() && !datum->img1.has_value()));
		kimera_pipeline.fillSingleImuQueue(VIO::ImuMeasurement(datum->dataset_time, imu_raw_vals));

		// If there is not cam data this func call, break early
		if (!datum->img0.has_value() && !datum->img1.has_value()) {
			return;
		}

#ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{std::to_string(iteration_no)});
		iteration_no++;
		if (iteration_no % 20 == 0) {
			cv::metrics::dump();
		}
#else
#warning "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
#endif

		cv::Mat img0{*imu_cam_buffer->img0.value()};
		cv::Mat img1{*imu_cam_buffer->img1.value()};

		// VIOParams
		VIO::CameraParams left_cam_info = kimera_pipeline_params.camera_params_.at(0);
		VIO::CameraParams right_cam_info = kimera_pipeline_params.camera_params_.at(1);
		kimera_pipeline.fillLeftFrameQueue(VIO::make_unique<VIO::Frame>(kimera_current_frame_id,
																	datum->dataset_time,
																	left_cam_info, img0));
		kimera_pipeline.fillRightFrameQueue(VIO::make_unique<VIO::Frame>(kimera_current_frame_id,
																	datum->dataset_time,
																	right_cam_info, img1));

		kimera_pipeline.spin();
#ifndef NDEBUG
    	std::cout << "SPIN FULL\n";
#endif
	}

  	void pose_callback(const std::shared_ptr<VIO::BackendOutput>& vio_output) {

		const auto& cached_state = vio_output->W_State_Blkf_;
		const auto& w_pose_blkf_trans = cached_state.pose_.translation().transpose();
		const auto& w_pose_blkf_rot = cached_state.pose_.rotation().quaternion();
		const auto& w_vel_blkf = cached_state.velocity_.transpose();
		const auto& imu_bias_gyro = cached_state.imu_bias_.gyroscope().transpose();
		const auto& imu_bias_acc = cached_state.imu_bias_.accelerometer().transpose();
		// Get the pose returned from SLAM
		Eigen::Quaternionf quat = Eigen::Quaternionf{w_pose_blkf_rot(0), w_pose_blkf_rot(1), w_pose_blkf_rot(2), w_pose_blkf_rot(3)};
		Eigen::Quaterniond doub_quat = Eigen::Quaterniond{w_pose_blkf_rot(0), w_pose_blkf_rot(1), w_pose_blkf_rot(2), w_pose_blkf_rot(3)};
		Eigen::Vector3f pos  = w_pose_blkf_trans.cast<float>();

		assert(isfinite(quat.w()));
		assert(isfinite(quat.x()));
		assert(isfinite(quat.y()));
		assert(isfinite(quat.z()));
		assert(isfinite(pos[0]));
		assert(isfinite(pos[1]));
		assert(isfinite(pos[2]));

		_m_pose->put(new pose_type{
		.sensor_time = imu_cam_buffer->time,
		.position = pos,
		.orientation = quat,
		});

		_m_imu_integrator_input->put(new imu_integrator_input{
			.last_cam_integration_time = (double(imu_cam_buffer->dataset_time) / NANO_SEC),
			.t_offset = -0.05,

			.params = {
				.gyro_noise = kimera_pipeline_params.imu_params_.gyro_noise_,
				.acc_noise = kimera_pipeline_params.imu_params_.acc_noise_,
				.gyro_walk = kimera_pipeline_params.imu_params_.gyro_walk_,
				.acc_walk = kimera_pipeline_params.imu_params_.acc_walk_,
				.n_gravity = kimera_pipeline_params.imu_params_.n_gravity_,
				.imu_integration_sigma = kimera_pipeline_params.imu_params_.imu_integration_sigma_,
				.nominal_rate = kimera_pipeline_params.imu_params_.nominal_rate_,
			},

			.biasAcc =imu_bias_acc,
			.biasGyro = imu_bias_gyro,
			.position = w_pose_blkf_trans,
			.velocity = w_vel_blkf,
			.quat = doub_quat,
		});
	}

	virtual ~kimera_vio() override {}

private:
	const std::shared_ptr<switchboard> sb;
	std::unique_ptr<writer<pose_type>> _m_pose;
	std::unique_ptr<writer<imu_integrator_input>> _m_imu_integrator_input;
	time_type _m_begin;

	VIO::FrameId kimera_current_frame_id;
	VIO::VioParams kimera_pipeline_params;
	VIO::Pipeline kimera_pipeline;
	
	const imu_cam_type* imu_cam_buffer;
	double previous_timestamp = 0.0;
};

PLUGIN_MAIN(kimera_vio)
