#!/usr/bin/env python  
#
# /* ----------------------------------------------------------------------------
# * Copyright 2019, Massachusetts Institute of Technology,
# * Cambridge, MA 02139
# * All Rights Reserved
# * Authors: Luca Carlone, et al. (see THANKS for the full author list)
# * See LICENSE for the license information
# * -------------------------------------------------------------------------- */
#
# /**
# * @file   realsense_info2sparkvio_pinhole-radtan.py
# * @brief  create calibration.yaml config from RealSense info topic
# * @author Sandro Berchier
# */
#
##################################################################################################
##################################################################################################
#
# Usage    : python kalibr2sparkvio_stereo.py --flags
# 	-tf_topic_cam0 		--> tf frame name of cam0, default='realsense_infra1_optical_frame'
#	-info_topic_cam0 	--> camera info topic name of cam0, default='realsense/infra1/camera_info'
# 	-tf_topic_cam1 		--> tf frame name of cam1, default='realsense_infra2_optical_frame'
#	-info_topic_cam1 	--> camera info topic name of cam1, default='realsense/infra2/camera_info'\
#	-tf_topic_imu0	 	--> tf frame name of imu0, default='realsense_accel_frame'
# 	-output 		--> path for output directory, default='./example'
#	-responsible		--> person who performed the calibration, default='unknown'
#	-date			--> date when calibration was performed, default='unknown'
#	-stereo			--> stereo hardware used in calibration, default='unknown'
#	-IMU			--> IMU hardware used in calibration, default='unknown'
#	-cam_rate		--> camera rate in hz, default=20	
#
# Example:
# 	python realsense_info2sparkvio_pinhole-radtan.py -tf_topic_cam0 realsense_infra1_optical_frame -tf_topic_cam1 realsense_infra2_optical_frame -info_topic_cam0 realsense/infra1/camera_info -info_topic_cam1 realsense/infra2/camera_info -tf_topic_imu0 realsense_accel_frame -output ~/Desktop -responsible 'factory calibration??' -date 'unknown' -stereo 'RealSense D435i' -IMU 'RealSense D435i'
#
##################################################################################################
##################################################################################################

######################################### DEPENDENCIES #############################################

import argparse, roslib, os, rospy, math, tf
import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import CameraInfo

######################################### GLOBAL VARIABLES #########################################

rospy.init_node('realsense_tf_listener')
listener = tf.TransformListener()

########################################### FUNCTIONS ##############################################

def tf_topic_pose(frame1,frame2):

	# Listen to transform
	(tras,quat) = listener.lookupTransform(frame1, frame2, rospy.Time(0))

	# Get translation and quaternion
	traslation = np.array([[tras[0]],[tras[1]],[tras[2]]])
	quaternion = np.array([quat[0],quat[1],quat[2],quat[3]])

	# Build inverse pose
	pose_inv = np.array(quaternion_matrix(quaternion))
	pose_inv[0,3] = traslation[0]
	pose_inv[1,3] = traslation[1]
	pose_inv[2,3] = traslation[2]
	#pose = np.linalg.inv(pose_inv)
	pose = pose_inv

	# Print pose for frame 1 to frame 2
	np.set_printoptions(precision=3,suppress=True)
	print ("Pose: " + frame1 + " to " + frame2)
	print pose

	# Return pose
	return pose

def topic_calibration(cam_info):

	# Listen to topic once
	cam_intrinsics = rospy.wait_for_message(cam_info,CameraInfo)

	# Print camera info
	print cam_intrinsics

	# Return camera info
	return cam_intrinsics

# Class to output yaml file for SparkVIO	
class spark_config:
	# YAML format for SparkVIO
	def __init__(self,responsible_calibration, date_calibration, \
		stereo_hardware, IMU_hardware, \
		camera_model,camera_distortion_model, \
		camera_rate_hz,camera_resolution, \
		left_camera_intrinsics,left_camera_distortion_coefficients, \
		left_camera_extrinsics, \
		right_camera_intrinsics, right_camera_distortion_coefficients, \
		right_camera_extrinsics, \
    		calibration_to_body_frame, \
		imu_rate_hz, imu_extrinsics, \
		gyroscope_noise_density, gyroscope_random_walk, \
		accelerometer_noise_density, accelerometer_random_walk):
		self.str = """\
# Calibration (ROS - Camera Info - Factory Calibration)
# ----------------------------------------------------------------------------------
# General Info
responsible: %s
date: %s

# Hardware
stereo_camera_hardware: %s
IMU_hardware: %s


# Cameras
# ----------------------------------------------------------------------------------
# Rate:
camera_rate_hz: %i

# Camera Model:
camera_model: %s
distortion_model: %s

# Resolution:
camera_resolution: [%d,%d] # width, height

# Left Camera Parameters:
left_camera_intrinsics: [%.3f,%.3f,%.3f,%.3f] # fu, fv, cu, cv
left_camera_distortion_coefficients: [%.9f,%.9f,%.9f,%.9f] # k1, k2, p1, p2

# Left Camera to IMU Transformation:
left_camera_extrinsics: [%.9f, %.9f, %.9f, %.9f,
                         %.9f, %.9f, %.9f, %.9f,
                         %.9f, %.9f, %.9f, %.9f, 
                         %.9f, %.9f, %.9f, %.9f]

# Right Camera Parameters:
right_camera_intrinsics: [%.3f,%.3f,%.3f,%.3f] # fu, fv, cu, cv
right_camera_distortion_coefficients: [%.9f,%.9f,%.9f,%.9f] # k1, k2, p1, p2

# Right Camera to IMU Transformation:
right_camera_extrinsics: [%.9f, %.9f, %.9f, %.9f,
                          %.9f, %.9f, %.9f, %.9f,
                          %.9f, %.9f, %.9f, %.9f, 
                          %.9f, %.9f, %.9f, %.9f]


# Body
# ----------------------------------------------------------------------------------
# Transformation:
calibration_to_body_frame: [%.9f, %.9f, %.9f, %.9f,
                            %.9f, %.9f, %.9f, %.9f,
                            %.9f, %.9f, %.9f, %.9f, 
                            %.9f, %.9f, %.9f, %.9f]


# IMU
# ----------------------------------------------------------------------------------
# Rate: 
imu_rate_hz: %i

# Noise Model Parameters: (Static)
gyroscope_noise_density: %.9f    # [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
gyroscope_random_walk: %.9f       # [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
accelerometer_noise_density: %.9f  # [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
accelerometer_random_walk: %.9f    # [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )

# IMU to Body Transformation:
imu_extrinsics: [%.9f, %.9f, %.9f, %.9f,
                 %.9f, %.9f, %.9f, %.9f,
                 %.9f, %.9f, %.9f, %.9f, 
                 %.9f, %.9f, %.9f, %.9f]
""" % (responsible_calibration, date_calibration,
	stereo_hardware, IMU_hardware,
	camera_rate_hz,camera_model,camera_distortion_model, \
	camera_resolution[0],camera_resolution[1], \
	left_camera_intrinsics[0],left_camera_intrinsics[1], \
	left_camera_intrinsics[2],left_camera_intrinsics[3], \
	left_camera_distortion_coefficients[0],left_camera_distortion_coefficients[1], \
	left_camera_distortion_coefficients[2],left_camera_distortion_coefficients[3], \
	left_camera_extrinsics[0,0],left_camera_extrinsics[0,1],left_camera_extrinsics[0,2],left_camera_extrinsics[0,3], \
	left_camera_extrinsics[1,0],left_camera_extrinsics[1,1],left_camera_extrinsics[1,2],left_camera_extrinsics[1,3], \
	left_camera_extrinsics[2,0],left_camera_extrinsics[2,1],left_camera_extrinsics[2,2],left_camera_extrinsics[2,3], \
	left_camera_extrinsics[3,0],left_camera_extrinsics[3,1],left_camera_extrinsics[3,2],left_camera_extrinsics[3,3], \
	right_camera_intrinsics[0],right_camera_intrinsics[1], \
	right_camera_intrinsics[2],right_camera_intrinsics[3], \
	right_camera_distortion_coefficients[0],right_camera_distortion_coefficients[1], \
	right_camera_distortion_coefficients[2],right_camera_distortion_coefficients[3], \
	right_camera_extrinsics[0,0],right_camera_extrinsics[0,1],right_camera_extrinsics[0,2],right_camera_extrinsics[0,3], \
	right_camera_extrinsics[1,0],right_camera_extrinsics[1,1],right_camera_extrinsics[1,2],right_camera_extrinsics[1,3], \
	right_camera_extrinsics[2,0],right_camera_extrinsics[2,1],right_camera_extrinsics[2,2],right_camera_extrinsics[2,3], \
	right_camera_extrinsics[3,0],right_camera_extrinsics[3,1],right_camera_extrinsics[3,2],right_camera_extrinsics[3,3], \
	calibration_to_body_frame[0,0],calibration_to_body_frame[0,1],calibration_to_body_frame[0,2],calibration_to_body_frame[0,3], \
	calibration_to_body_frame[1,0],calibration_to_body_frame[1,1],calibration_to_body_frame[1,2],calibration_to_body_frame[1,3], \
	calibration_to_body_frame[2,0],calibration_to_body_frame[2,1],calibration_to_body_frame[2,2],calibration_to_body_frame[2,3], \
	calibration_to_body_frame[3,0],calibration_to_body_frame[3,1],calibration_to_body_frame[3,2],calibration_to_body_frame[3,3], \
	imu_rate_hz, \
	gyroscope_noise_density,gyroscope_random_walk, \
	accelerometer_noise_density,accelerometer_random_walk, \
	imu_extrinsics[0,0],imu_extrinsics[0,1],imu_extrinsics[0,2],imu_extrinsics[0,3], \
	imu_extrinsics[1,0],imu_extrinsics[1,1],imu_extrinsics[1,2],imu_extrinsics[1,3], \
	imu_extrinsics[2,0],imu_extrinsics[2,1],imu_extrinsics[2,2],imu_extrinsics[2,3], \
	imu_extrinsics[3,0],imu_extrinsics[3,1],imu_extrinsics[3,2],imu_extrinsics[3,3])


########################################## START MAIN ############################################

if __name__ == '__main__':

	# Parse flags and arguments.
    	parser = argparse.ArgumentParser(description='Parse flags and arguments..')
    	parser.add_argument('-tf_topic_cam0','--tf_frame_cam0', help='Frame name of cam0.',default='realsense_infra1_optical_frame')
    	parser.add_argument('-tf_topic_cam1','--tf_frame_cam1', help='Frame name of cam1.',default='realsense_infra2_optical_frame')
    	parser.add_argument('-info_topic_cam0','--info_topic_cam0', help='Info topic name of cam0.',default='realsense/infra1/camera_info')
    	parser.add_argument('-info_topic_cam1','--info_topic_cam1', help='Info topic name of cam1.',default='realsense/infra2/camera_info')
    	parser.add_argument('-tf_topic_imu0','--tf_frame_imu0', help='Frame name of imu0.',default='realsense_accel_frame')
    	#parser.add_argument('-info_topic_imu0','--info_topic_imu0', help='Info topic name of imu0.',default='realsense/imu_info') # This doesn't work...
    	parser.add_argument('-output','--output_directory', help='Path to output yamls.', default='./example')
    	parser.add_argument('-responsible','--responsible_calibration', help='Person who calibrated the VI sensor.', default='unknown')
	parser.add_argument('-date','--date_calibration', help='Date of VI sensor calibration.', default='unknown')
	parser.add_argument('-stereo','--stereo_hardware', help='Visual hardware of VI sensor.', default='unknown')
	parser.add_argument('-IMU','--IMU_hardware', help='IMU hardware of VI sensor.', default='unknown')
    	parser.add_argument('-cam_rate', '--cam_rate_hz', help='Cam rate [hz].', default=30)
    	args = parser.parse_args()

	# Get ROS messages
	rate = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		try:
			# Camera 0
			cam0_intr = topic_calibration(args.info_topic_cam0)
			cam0_extr = tf_topic_pose(args.tf_frame_imu0,args.tf_frame_cam0)

			# Camera 1
			cam1_intr = topic_calibration(args.info_topic_cam0)
			cam1_extr = tf_topic_pose(args.tf_frame_imu0,args.tf_frame_cam1)

			# IMU 0
			imu0_extr = tf_topic_pose(args.tf_frame_imu0,args.tf_frame_imu0)

			break

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		rate.sleep()

	# Default values for IMU: this need to be changed! Info topic doesn't work... 
	imu_rate_hz = 400
	gyroscope_noise_density = 0.013000000
	gyroscope_random_walk = 0.001300000
	accelerometer_noise_density = 0.083000000
	accelerometer_random_walk = 0.008300000

	# Create config class (using plumb_bob = pinhole radtan)
    	sparkvio_config = spark_config(
		args.responsible_calibration, \
		args.date_calibration, \
		args.stereo_hardware, \
		args.IMU_hardware, \
		"pinhole", \
		"radtan", \
		args.cam_rate_hz, \
		np.array([cam0_intr.width,cam0_intr.height]), \
		np.array([cam0_intr.K[0],cam0_intr.K[4],cam0_intr.K[2],cam0_intr.K[5]]), \
		np.array([cam0_intr.D[0],cam0_intr.D[1],cam0_intr.D[2],cam0_intr.D[3]]), \
		cam0_extr, \
		np.array([cam1_intr.K[0],cam1_intr.K[4],cam1_intr.K[2],cam1_intr.K[5]]), \
		np.array([cam1_intr.D[0],cam1_intr.D[1],cam1_intr.D[2],cam1_intr.D[3]]), \
		cam1_extr, \
		np.eye(4), \
		imu_rate_hz, \
		imu0_extr, \
		gyroscope_noise_density, \
		gyroscope_random_walk, \
		accelerometer_noise_density, \
		accelerometer_random_walk)


    	################################ PRINT

    	# Dump config file
    	config_name = 'calibration.yaml'
    	with open(os.path.join(args.output_directory, config_name), 'w+') as outfile:
		outfile.write(sparkvio_config.str)


########################################## END MAIN ############################################
