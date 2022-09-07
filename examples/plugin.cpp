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
#include "../common/error_util.hpp"

using namespace ILLIXR;

std::string get_path() {
    const char* KIMERA_ROOT_c_str = std::getenv("KIMERA_ROOT");
	if (!KIMERA_ROOT_c_str) {
        ILLIXR::abort("Please define KIMERA_ROOT");
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
        , _m_pose{sb->get_writer<pose_type>("slow_pose")}
        , _m_imu_integrator_input{sb->get_writer<imu_integrator_input>("imu_integrator_input")}
        , cam{nullptr}
        , _m_cam{sb->get_buffered_reader<cam_type>("cam")}
    {
        // TODO: read Kimera flag file path from runner and find a better way of passing it to gflag
        kimera_pipeline.registerBackendOutputCallback(
            std::bind(
                &kimera_vio::pose_callback,
                this,
                std::placeholders::_1
            )
        );

        _m_pose.put(_m_pose.allocate<pose_type>(pose_type{
            time_point{},
            Eigen::Vector3f{0, 0, 0},
            Eigen::Quaternionf{1, 0, 0, 0}
        }));

#ifdef CV_HAS_METRICS
        cv::metrics::setAccount(new std::string{"-1"});
#endif
    }


    virtual void start() override {
        plugin::start();
        sb->schedule<imu_type>(id, "imu", [this](switchboard::ptr<const imu_type> datum, std::size_t) {
            this->feed_imu_cam(datum);
        });
    }


    std::size_t iteration_no = 0;
    void feed_imu_cam(switchboard::ptr<const imu_type> datum) {
        // Ensures that slam doesnt start before valid IMU readings come in
        if (datum == nullptr) {
            return;
        }

        // This ensures that every data point is coming in chronological order If youre failing this assert, 
        // make sure that your data folder matches the name in offline_imu_cam/plugin.cc
        assert(datum->time > previous_timestamp);
        previous_timestamp = datum->time;

        VIO::Vector6 imu_raw_vals;
        imu_raw_vals << datum->linear_a.cast<double>(), datum->angular_v.cast<double>();

        // Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
        // assert((datum->img0.has_value() && datum->img1.has_value()) || (!datum->img0.has_value() && !datum->img1.has_value()));
        kimera_pipeline.fillSingleImuQueue(VIO::ImuMeasurement(datum->time.time_since_epoch().count(), imu_raw_vals));

		// Buffered Async:
		cam = _m_cam.size() == 0 ? nullptr : _m_cam.dequeue();
		// If there is not cam data this func call, break early
		if (!cam) {
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

        cv::Mat img0{cam->img0};
        cv::Mat img1{cam->img1};

        // VIOParams
        VIO::CameraParams left_cam_info = kimera_pipeline_params.camera_params_.at(0);
        VIO::CameraParams right_cam_info = kimera_pipeline_params.camera_params_.at(1);
        kimera_pipeline.fillLeftFrameQueue(VIO::make_unique<VIO::Frame>(kimera_current_frame_id,
																	datum->time.time_since_epoch().count(),
                                                                    left_cam_info, img0));
        kimera_pipeline.fillRightFrameQueue(VIO::make_unique<VIO::Frame>(kimera_current_frame_id,
																	datum->time.time_since_epoch().count(),
                                                                    right_cam_info, img1));

		// Ok this needs to be filed in a later task to dig up what sets errno in Kimera
		assert(errno == 0);
		kimera_pipeline.spin();
		errno = 0;

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
        Eigen::Quaternionf quat = Eigen::Quaternionf{
            static_cast<float>(w_pose_blkf_rot(0)),
            static_cast<float>(w_pose_blkf_rot(1)),
            static_cast<float>(w_pose_blkf_rot(2)),
            static_cast<float>(w_pose_blkf_rot(3))
        };
        Eigen::Quaterniond doub_quat = Eigen::Quaterniond{
            w_pose_blkf_rot(0),
            w_pose_blkf_rot(1),
            w_pose_blkf_rot(2),
            w_pose_blkf_rot(3)
        };
        Eigen::Vector3f pos  = w_pose_blkf_trans.cast<float>();

        assert(isfinite(quat.w()));
        assert(isfinite(quat.x()));
        assert(isfinite(quat.y()));
        assert(isfinite(quat.z()));
        assert(isfinite(pos[0]));
        assert(isfinite(pos[1]));
        assert(isfinite(pos[2]));

        _m_pose.put(_m_pose.allocate<pose_type>(pose_type{
            cam->time,
            pos,
            quat
        }));

        _m_imu_integrator_input.put(_m_imu_integrator_input.allocate<imu_integrator_input>(imu_integrator_input{
			cam->time,
			duration(std::chrono::milliseconds{-50}),
            imu_params{
                kimera_pipeline_params.imu_params_.gyro_noise_,
                kimera_pipeline_params.imu_params_.acc_noise_,
                kimera_pipeline_params.imu_params_.gyro_walk_,
                kimera_pipeline_params.imu_params_.acc_walk_,
                kimera_pipeline_params.imu_params_.n_gravity_,
                kimera_pipeline_params.imu_params_.imu_integration_sigma_,
                kimera_pipeline_params.imu_params_.nominal_rate_,
            },
            imu_bias_acc,
            imu_bias_gyro,
            w_pose_blkf_trans,
            w_vel_blkf,
            doub_quat
        }));
    }

    virtual ~kimera_vio() override {}

private:
    const std::shared_ptr<switchboard> sb;
    switchboard::writer<pose_type> _m_pose;
    switchboard::writer<imu_integrator_input> _m_imu_integrator_input;

    VIO::FrameId kimera_current_frame_id;
    VIO::VioParams kimera_pipeline_params;
    VIO::Pipeline kimera_pipeline;
    
    switchboard::ptr<const cam_type> cam;
	switchboard::buffered_reader<cam_type> _m_cam;
    time_point previous_timestamp = time_point{duration{-1}};
};

PLUGIN_MAIN(kimera_vio)
