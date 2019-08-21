#!/usr/bin/env python
# * --------------------------------------------------------------------------
# * Copyright 2019, Massachusetts Institute of Technology,
# * Cambridge, MA 02139
# * All Rights Reserved
# * Authors: Luca Carlone, et al. (see THANKS for the full author list)
# * See LICENSE for the license information
# * --------------------------------------------------------------------------
# * @file   kalibr2sparkvio_stereo_pinhole-radtan.py
# * @brief  create calibration.yaml config from Kalibr files (pinhole radtan)
# * @author Sandro Berchier
# * --------------------------------------------------------------------------
# * Instructions: python config2sparkvio.py --flags
# The script creates yaml config files for SparkVIO from Kalibr or
# ROS info topics. Options: stereo-radtan, stereo-equidistant and RGBD.
# 	-config_option 		--> VIO config option, default='stereo-radtan'
# 	-output 		--> path for output directory, default='./example'
#	-responsible		--> person who performed the calibration,
# 	default='unknown'
#	-date			--> date when calibration was performed, default='unknown'
#	-camera			--> camera hardware used in calibration, default='unknown'
#	-IMU			--> IMU hardware used in calibration, default='unknown'
#	-cam_rate		--> camera rate in hz, default=20
# 	-input_cam 		--> path to directory with input file for cameras,
# 	default='./example/camchain.yaml'
#	-input_imu 		--> path to directory with input file for imu,
# 	default='./example/imu.yaml'
#	-baseline		--> baseline in m, default=0.05m
# 	-tf_topic_cam0 		--> tf frame name of cam0,
# 	default='realsense_infra1_optical_frame'
#	-info_topic_cam0 	--> camera info topic name of cam0,
# 	default='realsense/infra1/camera_info'
# 	-tf_topic_cam1 		--> tf frame name of cam1,
# 	default='realsense_infra2_optical_frame'
#	-info_topic_cam1 	--> camera info topic name of cam1,
# 	default='realsense/infra2/camera_info'\
#	-tf_topic_imu0	 	--> tf frame name of imu0,
# 	default='realsense_accel_frame'	
# * Example:
# 		python config2sparkvio.py 
# 		-config_option 'stereo-radtan'
# 		-input_cam /home/sb/ETH/example/camchain.yaml 
# 		-input_imu /home/sb/ETH/example/imu.yaml 
# 		-output /home/sb/ETH/example 
# 		-responsible 'Sandro Berchier' 
# 		-date '05.06.2019' 
# 		-camera 'MyntEye S' 
# 		-IMU 'MyntEye S'
# * --------------------------------------------------------------------------
################################# DEPENDENCIES #################################
import argparse, os, config_library

################################# START MAIN ###################################

if __name__ == "__main__":

    	# Parse flags and arguments.
    	parser = argparse.ArgumentParser(
				description='Parse flags and arguments..')
	parser.add_argument('-config_option','--config_option', \
			help=('VIO config options: (stereo-radtan, RGBD,' +
				' stereo-equi from Kalibr and stereo-radtan from rosinfo).'), \
			default='stereo-radtan')

	# General
    	parser.add_argument('-output','--output_directory', \
			 help='Path to output yamls.', default='./example')
    	parser.add_argument('-responsible','--responsible_calibration',
			help='Person who calibrated the VI sensor.', default='unknown')
	parser.add_argument('-date','--date_calibration', \
		help='Date of VI sensor calibration.', default='unknown')
	parser.add_argument('-camera','--camera_hardware', \
		help='Visual hardware of VI sensor.', default='unknown')
	parser.add_argument('-IMU','--IMU_hardware', \
		help='IMU hardware of VI sensor.', default='unknown')
    	parser.add_argument('-cam_rate', '--cam_rate_hz',
			help='Cam rate [hz].', default=20)

	# Specific to Stereo and RGB-D
    	parser.add_argument('-input_cam','--input_directory_cam', \
			help='Path of Kalibr yaml for camchain.', \
			default='./example/camchain.yaml')
    	parser.add_argument('-input_imu','--input_directory_imu', \
			help='Path of Kalibr yaml for imu.', \
			default='./example/imu.yaml')
	parser.add_argument('-baseline','--baseline', \
			help='Baseline in m.',default=0.05)

	# Specific to Intel Calibration
	parser.add_argument('-tf_topic_cam0','--tf_frame_cam0', \
			help='Frame name of cam0.', \
			default='realsense_infra1_optical_frame')
    	parser.add_argument('-tf_topic_cam1','--tf_frame_cam1', \
			help='Frame name of cam1.', \
			default='realsense_infra2_optical_frame')
    	parser.add_argument('-info_topic_cam0','--info_topic_cam0', \
			help='Info topic name of cam0.', \
			default='realsense/infra1/camera_info')
    	parser.add_argument('-info_topic_cam1','--info_topic_cam1', \
			help='Info topic name of cam1.', \
			default='realsense/infra2/camera_info')
    	parser.add_argument('-tf_topic_imu0','--tf_frame_imu0', \
			help='Frame name of imu0.', \
			default='realsense_accel_frame')

    	args = parser.parse_args()

    	################################ CREATE CONFIG OPTIONS

	print ("\n>>>>> "+args.config_option+" option selected.\n")

	if (args.config_option=='stereo-radtan' or \
		args.config_option=='RGBD-radtan' or \
		args.config_option=='stereo-equi'):
			# Create config class
			sparkvio_config = \
					config_library.kalibr_configure(args)
	# Pinhole radtan = plumb bob
	elif (args.config_option=='intel-radtan'):
			# Create config class
			sparkvio_config = \
					config_library.intel_configure_pinhole_radtan(args)
	else:
			print ("\n>>>>> Option not listed.\n")

	################################ PRINT CONFIGURATION FILE

	# Dump config file
	config_name = 'calibration.yaml'
	with open(os.path.join(args.output_directory, config_name), \
		'w+') as outfile:
		outfile.write(sparkvio_config.str)
	print (">>>>> Terminated.\n")

################################### END MAIN ###################################