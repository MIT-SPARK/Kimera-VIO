#!/usr/bin/env python
# Usage    : python kalibr2sparkvio_stereo.py --flags
# 	-input_cam 		--> path to directory with input file for cameras, default='./example/camchain.yaml'
#	-input_imu 		--> path to directory with input file for imu, default='./example/imu.yaml'
# 	-output 		--> path for output directory, default='./example'
#	-responsible		--> person who performed the calibration, default='unknown'
#	-date			--> date when calibration was performed, default='unknown'
#	-stereo			--> stereo hardware used in calibration, default='unknown'
#	-IMU			--> IMU hardware used in calibration, default='unknown'
#	-cam_rate		--> camera rate in hz, default=20	
#
# Example:
# 	python kalibr2sparkvio_stereo_pinhole-fov.py -input_cam /home/sb/ETH/example/camchain.yaml -input_imu /home/sb/ETH/example/imu.yaml -output /home/sb/ETH/example -responsible 'Sandro Berchier' -date '19.02.2019' -stereo 'MyntEye S' -IMU 'MyntEye S'
##################################################################################################
##################################################################################################

######################################### DEPENDENCIES ###########################################
import argparse, yaml, numpy, os

########################################### CLASSES ##############################################
			
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
# Calibration (Kalibr)
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
left_camera_distortion_coefficients: [%.9f] # w

# Left Camera to IMU Transformation:
left_camera_extrinsics: [%.9f, %.9f, %.9f, %.9f,
                         %.9f, %.9f, %.9f, %.9f,
                         %.9f, %.9f, %.9f, %.9f, 
                         %.9f, %.9f, %.9f, %.9f]

# Right Camera Parameters:
right_camera_intrinsics: [%.3f,%.3f,%.3f,%.3f] # fu, fv, cu, cv
right_camera_distortion_coefficients: [%.9f] # w

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
	left_camera_distortion_coefficients[0], \
	left_camera_extrinsics[0,0],left_camera_extrinsics[0,1],left_camera_extrinsics[0,2],left_camera_extrinsics[0,3], \
	left_camera_extrinsics[1,0],left_camera_extrinsics[1,1],left_camera_extrinsics[1,2],left_camera_extrinsics[1,3], \
	left_camera_extrinsics[2,0],left_camera_extrinsics[2,1],left_camera_extrinsics[2,2],left_camera_extrinsics[2,3], \
	left_camera_extrinsics[3,0],left_camera_extrinsics[3,1],left_camera_extrinsics[3,2],left_camera_extrinsics[3,3], \
	right_camera_intrinsics[0],right_camera_intrinsics[1], \
	right_camera_intrinsics[2],right_camera_intrinsics[3], \
	right_camera_distortion_coefficients[0], \
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

if __name__ == "__main__":
    	# Parse flags and arguments.
    	parser = argparse.ArgumentParser(description='Parse flags and arguments..')
    	parser.add_argument('-input_cam','--input_directory_cam', help='Path of Kalibr yaml for camchain.',default='./example/camchain.yaml')
    	parser.add_argument('-input_imu','--input_directory_imu', help='Path of Kalibr yaml for imu.',default='./example/imu.yaml')
    	parser.add_argument('-output','--output_directory', help='Path to output yamls.', default='./example')
    	parser.add_argument('-responsible','--responsible_calibration', help='Person who calibrated the VI sensor.', default='unknown')
	parser.add_argument('-date','--date_calibration', help='Date of VI sensor calibration.', default='unknown')
	parser.add_argument('-stereo','--stereo_hardware', help='Visual hardware of VI sensor.', default='unknown')
	parser.add_argument('-IMU','--IMU_hardware', help='IMU hardware of VI sensor.', default='unknown')
    	parser.add_argument('-cam_rate', '--cam_rate_hz', help='Cam rate [hz].', default=20)
    	args = parser.parse_args()


    	################################ READ IN

    	# Read in Kalibr yaml files
    	kalibr_stereo = yaml.load(open(args.input_directory_cam))
    	kalibr_imu = yaml.load(open(args.input_directory_imu))

	# Create config class
    	sparkvio_config = spark_config(
		args.responsible_calibration, \
		args.date_calibration, \
		args.stereo_hardware, \
		args.IMU_hardware, \
		kalibr_stereo['cam0']['camera_model'], \
		kalibr_stereo['cam0']['distortion_model'], \
		args.cam_rate_hz, \
		kalibr_stereo['cam0']['resolution'], \
		kalibr_stereo['cam0']['intrinsics'], \
		kalibr_stereo['cam0']['distortion_coeffs'], \
		numpy.linalg.inv(numpy.array(kalibr_stereo['cam0']['T_cam_imu'])), \
		kalibr_stereo['cam1']['intrinsics'], \
		kalibr_stereo['cam1']['distortion_coeffs'], \
		numpy.linalg.inv(numpy.array(kalibr_stereo['cam1']['T_cam_imu'])), \
		numpy.eye(4), \
		kalibr_imu['imu0']['update_rate'], \
		numpy.array(kalibr_imu['imu0']['T_i_b']), \
		kalibr_imu['imu0']['gyroscope_noise_density'], \
		kalibr_imu['imu0']['gyroscope_random_walk'], \
		kalibr_imu['imu0']['accelerometer_noise_density'], \
		kalibr_imu['imu0']['accelerometer_random_walk'])


    	################################ PRINT

    	# Dump config file
    	config_name = 'calibration.yaml'
    	with open(os.path.join(args.output_directory, config_name), 'w+') as outfile:
		outfile.write(sparkvio_config.str)

    
########################################## END MAIN ############################################
