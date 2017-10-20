#!/usr/bin/env python

import os
import numpy as np
import square_environment
import transformations
import plot_utils
import pinhole_camera
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def project_landmarks_in_camera(cam, landmarks, T_world_cam):
    n = np.shape(landmarks)[0]
    T_cam_world = np.linalg.inv(T_world_cam)
    xyz_world =  np.concatenate((np.transpose(landmarks), np.ones((1,n))))
    xyz_cam = np.dot(T_cam_world, xyz_world)
    observations = dict()
    for i in range(n):
        px, visible = cam.project(xyz_cam[:,i])
        if visible:
            observations[i] = px
    
    return observations
    
def visualize_observations(ax, landmarks, obs, T_world_cam):
    for i, px in iter(obs.items()): # python2: obs.iteritems():
        d = np.array([landmarks[i,:], T_world_cam[:3,3]])
        ax.plot(d[:,0], d[:,1], d[:,2], 'g-', alpha=0.5)
        

if __name__ == "__main__":
    
    # -------------------------------------------------------------------------
    # Parameters

    output_dir = ''
    T_body_cam = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0],
                           [0, -1, 0, 0],
                           [0, 0, 0, 1]])
    
    # Create camera
    cam = pinhole_camera.PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0)    
    
    # Load Trajectory
    data_dir = os.path.join(output_dir, 'groundtruth.csv')
    data = np.genfromtxt(data_dir, delimiter=',', skip_header=1)
    time = data[:,0]
    t_world_body = data[:,1:4]
    q_world_body = data[:,4:8]
    n = len(time)
    
    # Create Environment
    landmarks = square_environment.sample_square_fence(2000, 10, 5) 

    file_out_landmarks = open(os.path.join(output_dir, 'landmarks.csv'), 'w')
    file_out_landmarks.write('landmark_id, x [m], y [m], z [m]\n')
    nlmk = np.shape(landmarks)[0]
    xyz_world =  np.concatenate((np.transpose(landmarks), np.ones((1,nlmk))))
    for i in range(nlmk):
        file_out_landmarks.write('%d, %.10f, %.10f, %.10f\n' % (i, xyz_world[0,i], xyz_world[1,i], xyz_world[2,i]))
    file_out_landmarks.close()

    # Select Key-Frames
    keyframe_indices = np.arange(0, n, n/300)
    
    # Compute landmark observations
    fig = plt.figure(figsize=(6,4))
    ax = Axes3D(fig, xlabel='x [m]', ylabel='y [m]', zlabel='z [m]')
    ax.view_init(elev=54, azim=-27)
    ax.plot(landmarks[:,0], landmarks[:,1], landmarks[:,2], 'b.')
    file_out = open(os.path.join(output_dir, 'cam0_tracks.csv'), 'w')
    file_out.write('timestamp [ns], landmark_id, z_tilde_x [px], z_tilde_y [px], z_tilde_stdev [px]\n')
    for i in keyframe_indices:
        T_world_body = transformations.matrix_from_quaternion(q_world_body[i,:])
        T_world_body[:3,3] = t_world_body[i,:]
        T_world_cam = np.dot(T_world_body, T_body_cam)
        plot_utils.draw_coordinate_frame(ax, T_world_cam[:3,3], T_world_cam[:3,:3], 1.0)
        obs = project_landmarks_in_camera(cam, landmarks, T_world_cam)
        print('Keyframe ' + str(i) + ' observes ' + str(len(obs)) + ' landmarks' + ' at time ' + str(time[i]))

        if i == 0:#np.mod(i, 800) == 0:
            visualize_observations(ax, landmarks, obs, T_world_cam)
                
        # Trace Observations
        for landmark_id, px in iter(obs.items()): #python2: obs.iteritems():
            file_out.write('%d, %d, %.10f, %.10f, %.3f\n' % (time[i], landmark_id, px[0], px[1], 1.0))
    
    
    file_out = open(os.path.join(output_dir, 'cam0_data.csv'), 'w')
    file_out.write('timestamp [ns], image_name\n')
    for i in keyframe_indices:
        file_out.write('%d, empty.png\n' % (time[i]))
            
    file_out.close()
    plot_utils.axis_equal_3d(ax)
    fig.savefig('synthetic_environment.pdf')
    
    
    