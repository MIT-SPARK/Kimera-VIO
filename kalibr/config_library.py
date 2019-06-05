#!/usr/bin/env python

######################################### DEPENDENCIES ###########################################
import argparse, roslib, os, rospy, math, tf, yaml
import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import CameraInfo

########################################### FUNCTIONS ##############################################

def kalibr_configure_pinhole_radtan(args):

	# Read in Kalibr yaml files
    	kalibr_stereo = yaml.load(open(args.input_directory_cam))
    	kalibr_imu = yaml.load(open(args.input_directory_imu))

	# Create config class
    	sparkvio_config = spark_config_pinhole_radtan(
		args.responsible_calibration, \
		args.date_calibration, \
		args.camera_hardware, \
		args.IMU_hardware, \
		kalibr_stereo['cam0']['camera_model'], \
		kalibr_stereo['cam0']['distortion_model'], \
		args.cam_rate_hz, \
		kalibr_stereo['cam0']['resolution'], \
		kalibr_stereo['cam0']['intrinsics'], \
		kalibr_stereo['cam0']['distortion_coeffs'], \
		np.linalg.inv(np.array(kalibr_stereo['cam0']['T_cam_imu'])), \
		kalibr_stereo['cam1']['intrinsics'], \
		kalibr_stereo['cam1']['distortion_coeffs'], \
		np.linalg.inv(np.array(kalibr_stereo['cam1']['T_cam_imu'])), \
		np.eye(4), \
		kalibr_imu['imu0']['update_rate'], \
		kalibr_stereo['cam0']['timeshift_cam_imu'], \
		np.array(kalibr_imu['imu0']['T_i_b']), \
		kalibr_imu['imu0']['gyroscope_noise_density'], \
		kalibr_imu['imu0']['gyroscope_random_walk'], \
		kalibr_imu['imu0']['accelerometer_noise_density'], \
		kalibr_imu['imu0']['accelerometer_random_walk'])

	return sparkvio_config

def kalibr_configure_RGBD_radtan(args):

    	# Read in Kalibr yaml files
    	kalibr_stereo = yaml.load(open(args.input_directory_cam))
    	kalibr_imu = yaml.load(open(args.input_directory_imu))

	# Adding virtual baseline
	virtual_pose = np.eye(4)
	virtual_pose[0,3] = -float(args.baseline)
	T_depth_imu = np.matmul(virtual_pose,np.array(kalibr_stereo['cam0']['T_cam_imu']))
	
	# Create config class
    	sparkvio_config = spark_config_pinhole_radtan(
		args.responsible_calibration, \
		args.date_calibration, \
		args.camera_hardware, \
		args.IMU_hardware, \
		kalibr_stereo['cam0']['camera_model'], \
		kalibr_stereo['cam0']['distortion_model'], \
		args.cam_rate_hz, \
		kalibr_stereo['cam0']['resolution'], \
		kalibr_stereo['cam0']['intrinsics'], \
		kalibr_stereo['cam0']['distortion_coeffs'], \
		np.linalg.inv(np.array(kalibr_stereo['cam0']['T_cam_imu'])), \
		kalibr_stereo['cam0']['intrinsics'], \
		kalibr_stereo['cam0']['distortion_coeffs'], \
		np.linalg.inv(T_depth_imu), \
		np.eye(4), \
		kalibr_imu['imu0']['update_rate'], \
		kalibr_stereo['cam0']['timeshift_cam_imu'], \
		np.array(kalibr_imu['imu0']['T_i_b']), \
		kalibr_imu['imu0']['gyroscope_noise_density'], \
		kalibr_imu['imu0']['gyroscope_random_walk'], \
		kalibr_imu['imu0']['accelerometer_noise_density'], \
		kalibr_imu['imu0']['accelerometer_random_walk'])

	return sparkvio_config

def kalibr_configure_pinhole_equi(args):

    	# Read in Kalibr yaml files
    	kalibr_stereo = yaml.load(open(args.input_directory_cam))
    	kalibr_imu = yaml.load(open(args.input_directory_imu))

	# Create config class
    	sparkvio_config = spark_config_pinhole_equi(
		args.responsible_calibration, \
		args.date_calibration, \
		args.camera_hardware, \
		args.IMU_hardware, \
		kalibr_stereo['cam0']['camera_model'], \
		kalibr_stereo['cam0']['distortion_model'], \
		args.cam_rate_hz, \
		kalibr_stereo['cam0']['resolution'], \
		kalibr_stereo['cam0']['intrinsics'], \
		kalibr_stereo['cam0']['distortion_coeffs'], \
		np.linalg.inv(np.array(kalibr_stereo['cam0']['T_cam_imu'])), \
		kalibr_stereo['cam1']['intrinsics'], \
		kalibr_stereo['cam1']['distortion_coeffs'], \
		np.linalg.inv(np.array(kalibr_stereo['cam1']['T_cam_imu'])), \
		np.eye(4), \
		kalibr_imu['imu0']['update_rate'], \
		kalibr_stereo['cam0']['timeshift_cam_imu'], \
		np.array(kalibr_imu['imu0']['T_i_b']), \
		kalibr_imu['imu0']['gyroscope_noise_density'], \
		kalibr_imu['imu0']['gyroscope_random_walk'], \
		kalibr_imu['imu0']['accelerometer_noise_density'], \
		kalibr_imu['imu0']['accelerometer_random_walk'])

	return sparkvio_config

def intel_configure_pinhole_radtan(args):

	rospy.init_node('realsense_tf_listener')
	listener = tf.TransformListener()

	# Get ROS messages
	rate = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		try:
			# Camera 0
			cam0_intr = topic_calibration(args.info_topic_cam0)
			cam0_extr = tf_topic_pose(args.tf_frame_imu0,args.tf_frame_cam0,listener)
	
			# Camera 1
			cam1_intr = topic_calibration(args.info_topic_cam0)
			cam1_extr = tf_topic_pose(args.tf_frame_imu0,args.tf_frame_cam1,listener)

			# IMU 0
			imu0_extr = tf_topic_pose(args.tf_frame_imu0,args.tf_frame_imu0,listener)

			break
	
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			
			continue

		rate.sleep()

	# Default values for IMU: this need to be changed! Info topic doesn't work... 
	imu_rate_hz = 400
	imu_shift = 0.0
	gyroscope_noise_density = 0.013000000
	gyroscope_random_walk = 0.001300000
	accelerometer_noise_density = 0.083000000
	accelerometer_random_walk = 0.008300000

	# Create config class (using plumb_bob = pinhole radtan)
  	sparkvio_config = spark_config_pinhole_radtan(
		args.responsible_calibration, \
		args.date_calibration, \
		args.camera_hardware, \
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
		imu_shift, \
		imu0_extr, \
		gyroscope_noise_density, \
		gyroscope_random_walk, \
		accelerometer_noise_density, \
		accelerometer_random_walk)

	return sparkvio_config

def tf_topic_pose(frame1,frame2,listener):

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

######################################## PINHOLE RADTAN ##########################################
			
# Class to output yaml file for SparkVIO	
class spark_config_pinhole_radtan:
	# YAML format for SparkVIO
	def __init__(self,responsible_calibration, date_calibration, \
		camera_hardware, IMU_hardware, \
		camera_model,camera_distortion_model, \
		camera_rate_hz,camera_resolution, \
		left_camera_intrinsics,left_camera_distortion_coefficients, \
		left_camera_extrinsics, \
		right_camera_intrinsics, right_camera_distortion_coefficients, \
		right_camera_extrinsics, \
    		calibration_to_body_frame, \
		imu_rate_hz, imu_shift, imu_extrinsics, \
		gyroscope_noise_density, gyroscope_random_walk, \
		accelerometer_noise_density, accelerometer_random_walk):
		self.str = """\
# Calibration
# ----------------------------------------------------------------------------------
# General Info
responsible: %s
date: %s

# Hardware
camera_hardware: %s
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

# Timeshift: 
imu_shift: %.6f # t_imu0 = t_cam0 + imu_shift

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
	camera_hardware, IMU_hardware,
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
	imu_rate_hz, imu_shift, \
	gyroscope_noise_density,gyroscope_random_walk, \
	accelerometer_noise_density,accelerometer_random_walk, \
	imu_extrinsics[0,0],imu_extrinsics[0,1],imu_extrinsics[0,2],imu_extrinsics[0,3], \
	imu_extrinsics[1,0],imu_extrinsics[1,1],imu_extrinsics[1,2],imu_extrinsics[1,3], \
	imu_extrinsics[2,0],imu_extrinsics[2,1],imu_extrinsics[2,2],imu_extrinsics[2,3], \
	imu_extrinsics[3,0],imu_extrinsics[3,1],imu_extrinsics[3,2],imu_extrinsics[3,3])

######################################### PINHOLE EQUI ###########################################
			
# Class to output yaml file for SparkVIO	
class spark_config_pinhole_equi:
	# YAML format for SparkVIO
	def __init__(self,responsible_calibration, date_calibration, \
		camera_hardware, IMU_hardware, \
		camera_model,camera_distortion_model, \
		camera_rate_hz,camera_resolution, \
		left_camera_intrinsics,left_camera_distortion_coefficients, \
		left_camera_extrinsics, \
		right_camera_intrinsics, right_camera_distortion_coefficients, \
		right_camera_extrinsics, \
    		calibration_to_body_frame, \
		imu_rate_hz, imu_shift, imu_extrinsics, \
		gyroscope_noise_density, gyroscope_random_walk, \
		accelerometer_noise_density, accelerometer_random_walk):
		self.str = """\
# Calibration
# ----------------------------------------------------------------------------------
# General Info
responsible: %s
date: %s

# Hardware
camera_hardware: %s
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
left_camera_distortion_coefficients: [%.9f,%.9f,%.9f,%.9f] # k1, k2, k3, k4

# Left Camera to IMU Transformation:
left_camera_extrinsics: [%.9f, %.9f, %.9f, %.9f,
                         %.9f, %.9f, %.9f, %.9f,
                         %.9f, %.9f, %.9f, %.9f, 
                         %.9f, %.9f, %.9f, %.9f]

# Right Camera Parameters:
right_camera_intrinsics: [%.3f,%.3f,%.3f,%.3f] # fu, fv, cu, cv
right_camera_distortion_coefficients: [%.9f,%.9f,%.9f,%.9f] # k1, k2, k3, k4

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

# Timeshift: 
imu_shift: %.6f # t_imu0 = t_cam0 + imu_shift

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
	camera_hardware, IMU_hardware,
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
	imu_rate_hz, imu_shift, \
	gyroscope_noise_density,gyroscope_random_walk, \
	accelerometer_noise_density,accelerometer_random_walk, \
	imu_extrinsics[0,0],imu_extrinsics[0,1],imu_extrinsics[0,2],imu_extrinsics[0,3], \
	imu_extrinsics[1,0],imu_extrinsics[1,1],imu_extrinsics[1,2],imu_extrinsics[1,3], \
	imu_extrinsics[2,0],imu_extrinsics[2,1],imu_extrinsics[2,2],imu_extrinsics[2,3], \
	imu_extrinsics[3,0],imu_extrinsics[3,1],imu_extrinsics[3,2],imu_extrinsics[3,3])
