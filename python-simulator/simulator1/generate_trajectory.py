#!/usr/bin/env python

import os
import numpy as np
import transformations
import math_utils
import plot_utils
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def simulate_bias(n, dt, sigma_c, bias_init=np.array([0, 0, 0], dtype=np.float32)):
    bias = bias_init
    bias = np.zeros((n,3), dtype=np.float32)
    bias[0,:] = bias_init
    for i in range(1,n):
        N = np.array([np.random.normal(0.0, 1.0),
                      np.random.normal(0.0, 1.0),
                      np.random.normal(0.0, 1.0)])
        bias[i] = bias[i-1] + sigma_c * np.sqrt(dt) * N;
    return bias
      
def simulate_circular_trajectory(n, dt, horizontal_fequency = 2.0,
                                 horizontal_amplitude = 3.0,
                                 vertical_frequency = 3.0*np.pi,
                                 vertical_amplitude = 0.1,
                                 gravity = 9.81):
    """ Generate a circular trajectory with some sinusoidal vertical motion.
        The trajectory is parametric, so we can easily compute its derivatives
        to obtain the velocity, acceleration and angular velocity.
    """
    t = np.arange(0, dt*n, dt)
    
    # Position
    t_world_body = np.zeros((n,3))
    t_world_body[:,0] = horizontal_amplitude * np.sin(horizontal_fequency * t)
    t_world_body[:,1] = horizontal_amplitude * np.cos(horizontal_fequency * t)
    t_world_body[:,2] = vertical_amplitude * np.sin(vertical_frequency * t)

    # Velocity
    # world_v_body = dot(t_world_body)   assuming world is an inertial frame.
    world_v_body = np.zeros((n,3))
    world_v_body[:,0] = horizontal_amplitude * np.cos(horizontal_fequency * t) * horizontal_fequency
    world_v_body[:,1] = horizontal_amplitude * (-np.sin(horizontal_fequency * t)) * horizontal_fequency
    world_v_body[:,2] = vertical_amplitude * np.cos(vertical_frequency * t) * vertical_frequency
    
    # Acceleration
    # world_a_body = dot(world_v_body)   assuming world is an inertial frame.
    world_a_body = np.zeros((n,3))
    world_a_body[:,0] = horizontal_amplitude * (-np.sin(horizontal_fequency * t)) * horizontal_fequency**2
    world_a_body[:,1] = horizontal_amplitude * (-np.cos(horizontal_fequency * t)) * horizontal_fequency**2
    world_a_body[:,2] = vertical_amplitude * (-np.sin(vertical_frequency * t)) * vertical_frequency**2 + gravity
    
    # Orientation 
    R_world_body = np.zeros((n,9))
    R_world_body[:,0] = np.cos(-horizontal_fequency * t)
    R_world_body[:,1] = (-np.sin(-horizontal_fequency * t))
    R_world_body[:,3] = np.sin(-horizontal_fequency * t)
    R_world_body[:,4] = np.cos(-horizontal_fequency * t)
    R_world_body[:,8] = 1.0
    
    # Derivative of rotation matrix.
    dot_R_world_body = np.zeros((n,9))
    dot_R_world_body[:,0] = (-np.sin(-horizontal_fequency * t)) * (-horizontal_fequency)
    dot_R_world_body[:,1] = (-np.cos(-horizontal_fequency * t)) * (-horizontal_fequency)
    dot_R_world_body[:,3] = np.cos(-horizontal_fequency * t) * (-horizontal_fequency)
    dot_R_world_body[:,4] = (-np.sin(-horizontal_fequency * t)) * (-horizontal_fequency) 
    dot_R_world_body[:,8] = 1.0
    
    # Angular velocity
    # body_w_world_body = R_world_body^T * dot(R_world_body)
    body_w_world_body = np.zeros((n,3))
    for i in range(n):
        R = np.reshape(R_world_body[i,:], (3,3))
        dot_R = np.reshape(dot_R_world_body[i,:], (3,3))
        W = np.dot(np.transpose(R), dot_R)
        body_w_world_body[i,:] = math_utils.unskew(W)
        
    return t_world_body, world_v_body, world_a_body, R_world_body, body_w_world_body

def save_trajectory_to_file(time, t_world_body, R_world_body,
                            body_w_world_body_measured, body_a_measured,
                            gyro_bias, acc_bia,  world_v, output_dir = '/tmp'):

    # Trace groundtruth poses.
    file_out = open(os.path.join(output_dir, 'groundtruth.csv'), 'w')
    file_out.write('# timestamp [ns], t_w_b(x) [m], t_w_b(y) [m], t_w_b(z) [m], q_w_b(x) [rad], q_w_b(y) [rad], q_w_b(z) [rad], q_w_b(w) [rad]\n')
    for i in range(len(time)):
        T = np.eye(4)
        T[:3,:3] = np.reshape(R_world_body[i,:], (3,3))
        q_world_body = transformations.quaternion_from_matrix(T)
        q_world_body = q_world_body / np.linalg.norm(q_world_body)
        #q_world_body = q_world_body / np.linalg.norm(q_world_body)
        file_out.write('%d, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f\n' %
                       (time[i],
                        t_world_body[i,0], t_world_body[i,1], t_world_body[i,2],
                        q_world_body[0], q_world_body[1], q_world_body[2], q_world_body[3]))
    file_out.close()
    
    # Trace all groundtruth states (bias, velocity, pose)
    file_out = open(os.path.join(output_dir, 'groundtruth_states.csv'), 'w')
    file_out.write('# timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],  v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n')
    for i in range(len(time)):
        T = np.eye(4)
        T[:3,:3] = np.reshape(R_world_body[i,:], (3,3))
        q_world_body = transformations.quaternion_from_matrix(T)
        q_world_body = q_world_body / np.linalg.norm(q_world_body)
        #q_world_body = q_world_body / np.linalg.norm(q_world_body)
        file_out.write('%d, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f\n' %
                       (time[i],
                        t_world_body[i,0], t_world_body[i,1], t_world_body[i,2],
                        q_world_body[3], q_world_body[0], q_world_body[1], q_world_body[2],
                        world_v[i,0], world_v[i,1], world_v[i,2],
                        gyro_bias[i,0], gyro_bias[i,1], gyro_bias[i,2],
                        acc_bia[i,0], acc_bia[i,1], acc_bia[i,2]))
    file_out.close()
    
    # Trace IMU Measurements
    file_out = open(os.path.join(output_dir, 'imu0_data.csv'), 'w')
    file_out.write('# timestamp, gx, gy, gz, ax, ay, az\n')
    for i in range(len(time)):
        file_out.write('%d, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f\n' %
                       (time[i],
                        body_w_world_body_measured[i,0], body_w_world_body_measured[i,1], body_w_world_body_measured[i,2],
                        body_a_measured[i,0], body_a_measured[i,1], body_a_measured[i,2]))
    file_out.close()

if __name__ == "__main__":
    
    # -------------------------------------------------------------------------
    # IMU Parameters
    
    # Gyro noise density (sigma). [rad/s*1/sqrt(Hz)]
    gyro_noise_density = 0.0 # 0.0007

    # Accelerometer noise density (sigma). [m/s^2*1/sqrt(Hz)]
    acc_noise_density = 0.0 # 0.019

    # Gyro bias random walk (sigma). [rad/s^2*1/sqrt(Hz)]
    gyro_bias_random_walk_sigma = 0.0 # 0.0004

    # Accelerometer bias random walk (sigma). [m/s^3*1/sqrt(Hz)]
    acc_bias_random_walk_sigma = 0.0 #  0.012

    # Norm of the Gravitational acceleration. [m/s^2]
    gravity_magnitude = 9.81
     
    # -------------------------------------------------------------------------
    # Simulation Parameters
     
    # Number of samples.
    n = 60000
    n_loops = 6
    
    # Time between two measurements.
    dt = 1.0/500
    
    # Circular trajectory parametrization.
    horizontal_fequency = 2.0*np.pi/(n/n_loops*dt)
    horizontal_amplitude = 3.0
    vertical_frequency = 4 * 2.0*np.pi/(n/n_loops*dt)
    vertical_amplitude = 0.5
    
    output_dir = ''
    
    # -------------------------------------------------------------------------
    # Simulate IMU Data
    
    # Generate trajectory.
    t_world_body, world_v, world_a, R_world_body, body_w_world_body = \
        simulate_circular_trajectory(
            n, dt, horizontal_fequency, horizontal_amplitude,
            vertical_frequency, vertical_amplitude, gravity_magnitude)
    
    # Compute Bias Noise
    gyro_bias = simulate_bias(n, dt, gyro_bias_random_walk_sigma)
    acc_bias = simulate_bias(n, dt, acc_bias_random_walk_sigma)

    # Generate Gyroscope Measurements
    body_w_world_body_measured = np.zeros((n,3), dtype=np.float32)
    for i in range(n):
        N = np.array([np.random.normal(0.0, 1.0), np.random.normal(0.0, 1.0), np.random.normal(0.0, 1.0)])
        body_w_world_body_measured[i,:] = \
            body_w_world_body[i,:] + gyro_bias[i,:] + gyro_noise_density * 1.0/np.sqrt(dt) * N
            
    # Generate Accelerometer Measurements
    body_a = np.zeros((n,3), dtype=np.float32)
    body_a_measured = np.zeros((n,3), dtype=np.float32)
    for i in range(n):
        N = np.array([np.random.normal(0.0, 1.0), np.random.normal(0.0, 1.0), np.random.normal(0.0, 1.0)])
        R_body_world = np.transpose(np.reshape(R_world_body[i,:], (3,3)))
        body_a[i,:] = np.dot(R_body_world, world_a[i,:])
        body_a_measured[i,:] = \
            body_a[i,:] + acc_bias[i,:] + acc_noise_density * 1.0/np.sqrt(dt) * N    

    # -------------------------------------------------------------------------
    # Verification: Repeat integration to see how big the integration error is
    t_test = np.zeros((n,3))
    t_test[0,:] = t_world_body[0,:]
    v_test = np.zeros((n,3))
    v_test[0,:] = world_v[0,:]
    R_test = np.zeros((n, 9))
    R_test[0,:] = R_world_body[0,:]
    ang_test = np.zeros((n,))
    ang_test[0] = np.linalg.norm(transformations.logmap_so3(np.reshape(R_test[0,:],(3,3))))
    g = np.array([0, 0, -gravity_magnitude])
    for i in range(1,n):
      R = np.reshape(R_test[i-1,:], (3,3))
      ang_test[i] = np.linalg.norm(transformations.logmap_so3(R))
      v = v_test[i-1,:]
      a = body_a[i-1,:]
      w = body_w_world_body[i-1,:]
      t_test[i,:] = t_test[i-1,:] + v * dt  + g * (dt**2) * 0.5  + np.dot(R, a * (dt**2) * 0.5)
      v_test[i,:] = v  + g * dt + np.dot(R, a * dt)
      R_test[i,:] = np.reshape(np.dot(R, transformations.expmap_so3(w * dt)), (9,))

    fig = plt.figure(figsize=(8,5))
    ax = Axes3D(fig, xlabel='x [m]', ylabel='y [m]', zlabel='z [m]')
    ax.plot(t_test[:,0], t_test[:,1], t_test[:,2], label='re-integrated')
    plot_utils.axis_equal_3d(ax)

    # -------------------------------------------------------------------------
    # Write to File
    time = np.arange(0, dt*n*1e9, dt*1e9, dtype=np.uint64)
    save_trajectory_to_file(
        time, t_world_body, R_world_body, body_w_world_body_measured,
        body_a_measured, gyro_bias, acc_bias, world_v, output_dir)

    # -------------------------------------------------------------------------
    # Plot

    # Plot Trajectory
    t = np.arange(0, dt*n, dt)
    fig = plt.figure(figsize=(8,5))
    ax = Axes3D(fig, xlabel='x [m]', ylabel='y [m]', zlabel='z [m]')
    for i in range(0,n,70):
        plot_utils.draw_coordinate_frame(ax, t_world_body[i,:], np.reshape(R_world_body[i,:], (3,3)), 1.0)
    ax.legend()
    plot_utils.axis_equal_3d(ax)
    fig.savefig('sim_trajectory.png')
    
    # Plot Orientation
    #fig = plt.figure(figsize=(8,5))
    #ax = fig.add_subplot(111)
    #ax.plot(R_world_body, 'r')
    
    # Plot Accelereometer
    fig = plt.figure(figsize=(8,5))
    ax = fig.add_subplot(311, xlabel='t [s]', ylabel='', title='Accelerations')
    ax.plot(t, body_a_measured[:,0], 'r', label='$a_x$')
    ax.plot(t, body_a[:,0], 'k', lw=2)
    ax.legend()
    ax = fig.add_subplot(312, xlabel='t [s]', ylabel='')
    ax.plot(t, body_a_measured[:,1], 'g', label='$a_y$')    
    ax.plot(t, body_a[:,1], 'k')    
    ax.legend()
    ax = fig.add_subplot(313, xlabel='t [s]', ylabel='')
    ax.plot(t, body_a_measured[:,2], 'b', label='$a_z$')
    ax.plot(t, body_a[:,2], 'k')
    ax.legend()    
    fig.tight_layout()
    fig.savefig('sim_accelerometer.png')
    
    # Plot Angular Velocity
    fig = plt.figure(figsize=(8,5))
    ax = fig.add_subplot(311, xlabel='t [s]', ylabel='', title='Accelerations')
    ax.plot(t, body_w_world_body_measured[:,0], 'r', label='$\omega_x$')
    ax.plot(t, body_w_world_body[:,0], 'k', lw=2)
    ax.legend()
    ax = fig.add_subplot(312, xlabel='t [s]', ylabel='')
    ax.plot(t, body_w_world_body_measured[:,1], 'g', label='$\omega_y$')    
    ax.plot(t, body_w_world_body[:,1], 'k')    
    ax.legend()
    ax = fig.add_subplot(313, xlabel='t [s]', ylabel='')
    ax.plot(t, body_w_world_body_measured[:,2], 'b', label='$\omega_z$')
    ax.plot(t, body_w_world_body[:,2], 'k')
    ax.legend()    
    fig.tight_layout()
    fig.savefig('sim_gyroscope.png')
    
    # Plot Biases
    fig = plt.figure(figsize=(8,5))
    ax = fig.add_subplot(211, xlabel='t [s]', ylabel='', title='Accelerations')
    ax.plot(t, gyro_bias[:,0], 'r', label='$b^\omega_x$')
    ax.plot(t, gyro_bias[:,1], 'g', label='$b^\omega_y$')
    ax.plot(t, gyro_bias[:,2], 'b', label='$b^\omega_z$')
    ax.legend()
    ax = fig.add_subplot(212, xlabel='t [s]', ylabel='')
    ax.plot(t, acc_bias[:,0], 'r', label='$b^a_x$')
    ax.plot(t, acc_bias[:,1], 'g', label='$b^a_y$')
    ax.plot(t, acc_bias[:,2], 'b', label='$b^a_z$')
    ax.legend()    
    fig.tight_layout()
    fig.savefig('sim_biases.png')
    
    # Plot Biases
    fig = plt.figure(figsize=(8,5))
    ax = fig.add_subplot(311, xlabel='t [s]', ylabel='')
    ax.plot(t, t_world_body[:,0], 'r-', label='$p_x$')
    ax.plot(t, world_v[:,0], 'r--', label='$v_x$')    
    ax.plot(t, world_a[:,0], 'r.', label='$a_x$')
    ax.legend()
    ax = fig.add_subplot(312, xlabel='t [s]', ylabel='')
    ax.plot(t, t_world_body[:,1], 'g-', label='$p_y$')
    ax.plot(t, world_v[:,1], 'g--', label='$v_y$')
    ax.plot(t, world_a[:,1], 'g.', label='$a_y$')
    ax.legend()
    ax = fig.add_subplot(313, xlabel='t [s]', ylabel='')
    ax.plot(t, t_world_body[:,2], 'b-', label='$p_z$')
    ax.plot(t, world_v[:,2], 'b--', label='$v_z$')
    ax.plot(t, world_a[:,2], 'b.', label='$a_z$')
    ax.legend()
    fig.tight_layout()
    fig.savefig('sim_velocity.png')