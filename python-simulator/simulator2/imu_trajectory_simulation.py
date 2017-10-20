#!/usr/bin/python
"""
Zurich Eye

This script requires the installation of Sympy:
 > sudo apt-get install python3-pip
 > sudo pip3 install sympy
"""

import numpy as np
import sympy as sy
import matplotlib.pyplot as plt
import plot_utils
import transformations as tf
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

# Time: 
t = sy.Symbol('t')
gravity = 9.81

def sy_unskew(R):
    return sy.Matrix([R[2,1], R[0,2], R[1,0]])
    
def sy_rpy_matrix(theta1, theta2, theta3):
    # Define the principal rotation matrices:
    R1 = sy.Matrix([[1,               0,              0],
                    [0,  sy.cos(theta1), sy.sin(theta1)],
                    [0, -sy.sin(theta1), sy.cos(theta1)]])
                    
    R2 = sy.Matrix([[sy.cos(theta2), 0, -sy.sin(theta2)],
                    [0,              1,               0],
                    [sy.sin(theta2), 0,  sy.cos(theta2)]])
    
    R3 = sy.Matrix([[ sy.cos(theta3), sy.sin(theta3), 0],
                    [-sy.sin(theta3), sy.cos(theta3), 0],
                    [ 0,             0,               1]])
    
    # This sequence, which is very common in aerospace applications, is called
    # the 1-2-3 attitude sequence or the ‘roll-pitch-yaw’ convention.
    R_W_B = R3 * R2 * R1    
    
    # IMPORTANT: a singularity occurs when theta2 = pi/2, in this case theta1
    #            and theta3 are associated to the same rotation.
    return R_W_B
   
def simulate_rotation_matrix(w1=1, w2=1, w3=1, 
                             a1=1, a2=1, a3=1,
                             b1=0, b2=0, b3=0):
    """The rotation matrix is computed using Euler angles. Each angle is
    modeled as sinus with amplitude a, frequency w and phase b:
    """
    theta1 = a1 * sy.sin(b1 + w1 * t) # yaw
    theta2 = a2 * sy.sin(b2 + w2 * t) # pitch
    theta3 = a3 * sy.sin(b3 + w3 * t) # roll
    R_W_B = sy_rpy_matrix(theta1, theta2, theta3)
    return R_W_B
    
def rotation_rate_from_rotation_matrix(R_W_B):
    """Computes rotation rate from the derivative of a rotation matrix:
    body_w_world_body = skew(R_world_body^T * dot(R_world_body))
    
    """
    R_W_B_dot = sy.diff(R_W_B, t)
    W = R_W_B.T * R_W_B_dot
    B_w_W_B = sy_unskew(W)
    return B_w_W_B

def simulate_pos_vel_acc(w1=1, w2=1, w3=1, 
                         a1=1, a2=1, a3=1,
                         b1=0, b2=0, b3=0):
    """We model the position as sinus with amplitude a, frequency w and phase b.
    
    """
    t_W_B = sy.Matrix([a1 * sy.sin(b1 + w1 * t),
                       a2 * sy.sin(b2 + w2 * t),
                       a3 * sy.sin(b3 + w3 * t)])
    
    # Velocity = dot(Position), assuming World is an inertial frame.
    v_W_B = sy.diff(t_W_B, t)
     
    # Acceleration = dot(Velocity), assuming World is an inertial frame.        
    a_W_B = sy.diff(v_W_B, t) + sy.Matrix([0, 0, gravity])
    
    return t_W_B, v_W_B, a_W_B
    
def evaluate_symbolic_trajectory(t_W_B, v_W_B, a_W_B, R_W_B, B_w_W_B, t_min, t_max, dt):
    
    t_values = np.arange(t_min, t_max, dt)
    n = len(t_values)
    
    def copy_to_Nx3(a):
        x = np.zeros((n,3))
        x[:,0] = a[0][0,:]
        x[:,1] = a[1][0,:]
        x[:,2] = a[2][0,:]
        return x
        
    def copy_to_Nx9(a):
        x = np.zeros((n,9))
        x[:,0] = a[0][0,:]
        x[:,1] = a[0][1,:]
        x[:,2] = a[0][2,:]
        x[:,3] = a[1][0,:]
        x[:,4] = a[1][1,:]
        x[:,5] = a[1][2,:]
        x[:,6] = a[2][0,:]
        x[:,7] = a[2][1,:]
        x[:,8] = a[2][2,:]
        return x
        
    eval_t_W_B   = sy.utilities.lambdify(t, t_W_B, 'numpy')
    eval_v_W_B   = sy.utilities.lambdify(t, v_W_B, 'numpy')
    eval_a_W_B   = sy.utilities.lambdify(t, a_W_B, 'numpy')
    eval_R_W_B   = sy.utilities.lambdify(t, R_W_B, 'numpy')
    eval_B_w_W_B = sy.utilities.lambdify(t, B_w_W_B, 'numpy')
    t_W_B_val   = copy_to_Nx3(eval_t_W_B(t_values))
    v_W_B_val   = copy_to_Nx3(eval_v_W_B(t_values))
    a_W_B_val   = copy_to_Nx3(eval_a_W_B(t_values))
    R_W_B_val   = copy_to_Nx9(eval_R_W_B(t_values))
    B_w_W_B_val = copy_to_Nx3(eval_B_w_W_B(t_values))
    
    return t_W_B_val, v_W_B_val, a_W_B_val, R_W_B_val, B_w_W_B_val
    
def plot_trajectory(t_W_B_val, R_W_B_val, interval = 10):
    n = np.shape(t_W_B_val)[0]
    fig = plt.figure(figsize=(8,5))
    ax = Axes3D(fig, xlabel='x [m]', ylabel='y [m]', zlabel='z [m]')
    ax.plot(t_W_B_val[:,0], t_W_B_val[:,1], t_W_B_val[:,2], 'b-')
    for i in range(0, n, interval):
        R = np.reshape(R_W_B_val[i,:], (3,3))
        assert(np.abs(np.linalg.det(R) - 1.0) < 1e-5)
        plot_utils.draw_coordinate_frame(ax, t_W_B_val[i,:], R, 1.0)
    ax.legend()
    return ax, fig
      
def get_simulated_imu_data(t_max, dt, plot=False):
    
    t_W_B, v_W_B, a_W_B = simulate_pos_vel_acc(0.7, 1.0, .5,
                                               2.0, 1.0, 2.0,
                                               0.7, 1.0, 1.0)
    
    R_W_B  = simulate_rotation_matrix(0.5, 0.05, 0.2,
                                      1.0, 2.0, 1.0,
                                      0.0, 1.0, 4.0)
    
    B_w_W_B = rotation_rate_from_rotation_matrix(R_W_B)
    
    if plot:
        # Plot symbolic
        sy.plot(B_w_W_B[0], B_w_W_B[1], B_w_W_B[2], (t, 0, 2.0))
        sy.plotting.plot3d_parametric_line(t_W_B[0], t_W_B[1], t_W_B[2], (t, 0, 2.0))
        sy.plot(t_W_B[0], v_W_B[0], a_W_B[0], (t, 0, 2.0))
        sy.plot(t_W_B[1], v_W_B[1], a_W_B[1], (t, 0, 2.0))
        sy.plot(t_W_B[2], v_W_B[2], a_W_B[2], (t, 0, 2.0))
    
    # Evaluate symbolic trajectory for specific time instances
    t_W_B_val, v_W_B_val, a_W_B_val, R_W_B_val, B_w_W_B_val = \
        evaluate_symbolic_trajectory(t_W_B, v_W_B, a_W_B, R_W_B, B_w_W_B, 0.0, t_max, dt)
        
    # Transform IMU Measurements to body frame, where they are measured.
    n = np.shape(t_W_B_val)[0]
    B_a_W_B = np.zeros((n,3), dtype=np.float32)
    for i in range(n):
        R_B_W = np.transpose(np.reshape(R_W_B_val[i,:], (3,3)))
        B_a_W_B[i,:] = np.dot(R_B_W, a_W_B_val[i,:]) 

    return t_W_B_val, v_W_B_val, R_W_B_val, B_a_W_B, B_w_W_B_val


if __name__=='__main__':
    t_max = 10
    dt = 1.0/200.0
    t_W_B_val, v_W_B_val, R_W_B_val, B_a_W_B, B_w_W_B_val = get_simulated_imu_data(t_max, dt)
    ax, fig = plot_trajectory(t_W_B_val, R_W_B_val, interval=10)