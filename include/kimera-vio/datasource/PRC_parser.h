#pragma once

// STL
#include <string>
#include <fstream>
#include <algorithm> // for max 
#include <map>
#include <utility>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/datasource/DataSource-definitions.h"
#include "kimera-vio/datasource/DataSource.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"


namespace VIO{
	class PRCDataParser : public DataProvide {
	public:
		PRCDataParser(int initial_k,
					  int final_k,
					  int skip_n_start_frame,
					  int skip_n_end_frame);
		PRCDataParser();
		virtual ~PRCDataParser();
	public:
		virtual bool spin() override;
		
		void spinOnce(const FrameId& k,
                const StereoMatchingParams& stereo_matchiong_params,
                const bool equalize_image,
                const CameraParams& left_cam_info,
                const CameraParams& right_cam_info,
                const gtsam::Pose3& camL_pose_camR,
                Timestamp* timestamp_last_frame);
		// Parses ZED data as well as the frontend and backend params 
		void parse();

		// Compute error on the relative pose between two time stamps,
  		// compared with the relative pose from ground truth.
  	   std::pair<double, double> computePoseErrors(
	      const gtsam::Pose3& lkf_T_k_body,
	      const bool isTrackingValid,
	      const Timestamp& previousTimestamp,
	      const Timestamp& currentTimestamp,	
	      const bool upToScale = false) const;
	};
}

