#!/usr/bin/env python

import numpy as np
import plot_utils
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def sample_vertical_plane(num_samples, width, height, bottom_left, orientation='yz'):
    """Orientation = {'yz', 'xz'}"""
    X = np.zeros((num_samples, 3), dtype=np.float32)
    if orientation is 'yz':
        X[:,1] = np.random.uniform(0, width, num_samples)
        X[:,2] = np.random.uniform(0, height, num_samples)
    elif orientation is 'xz':
        X[:,0] = np.random.uniform(0, width, num_samples)
        X[:,2] = np.random.uniform(0, height, num_samples)
    else:
        raise ValueError('Orientation unknown.')
        
    X += bottom_left
    return X
    
def sample_horizontal_plane(num_samples, width, height, bottom_left, orientation='xy'):
    """Orientation = {'xy'}"""
    X = np.zeros((num_samples, 3), dtype=np.float32)
    if orientation is 'xy':
        X[:,0] = np.random.uniform(0, width, num_samples)
        X[:,1] = np.random.uniform(0, height, num_samples)
    else:
        raise ValueError('Orientation unknown.')
        
    X += bottom_left
    return X
    
def sample_box(num_samples, size):
    """Sample six vertical planes around the origin"""
    X1 = sample_vertical_plane(num_samples/6, size, size, [-size/2, -size/2, -size/2], 'yz')
    X2 = sample_vertical_plane(num_samples/6, size, size, [size/2, -size/2, -size/2], 'yz')
    X3 = sample_vertical_plane(num_samples/6, size, size, [-size/2, -size/2, -size/2], 'xz')
    X4 = sample_vertical_plane(num_samples/6, size, size, [-size/2, size/2, -size/2], 'xz')
    X5 = sample_horizontal_plane(num_samples/6, size, size, [-size/2, -size/2, -size/2])
    X6 = sample_horizontal_plane(num_samples/6, size, size, [-size/2, -size/2, size/2])
    X = np.concatenate((X1, X2, X3, X4, X5, X6), axis=0)
    return X    

def sample_square_fence(num_samples, width, height):
    """Sample four vertical planes around the origin"""
    X1 = sample_vertical_plane(num_samples/4, width, height, [-width/2, -width/2, -height/2], 'yz')
    X2 = sample_vertical_plane(num_samples/4, width, height, [width/2, -width/2, -height/2], 'yz')
    X3 = sample_vertical_plane(num_samples/4, width, height, [-width/2, -width/2, -height/2], 'xz')
    X4 = sample_vertical_plane(num_samples/4, width, height, [-width/2, width/2, -height/2], 'xz')
    X = np.concatenate((X1, X2, X3, X4), axis=0)
    return X    
    
def sample_city(num_samples, width_road=8, length_road=400, height_buildings=12, height_camera=3):
    """Sample a city like environment: camera is centered in a long street
       with horizontal road and vertical buildings along the street"""
    area_road = width_road * length_road
    area_buildings = 2.0 * height_buildings * length_road
    num_samples_road = area_road / (area_road+area_buildings) * num_samples
    num_samples_building = num_samples - num_samples_road
    road = sample_horizontal_plane(num_samples_road, width_road, length_road, [-width_road/2, -length_road/2, -height_camera/2])
    buildings_left = sample_vertical_plane(num_samples_building / 2, length_road, height_buildings, [-width_road/2, -length_road/2, -height_camera/2], 'yz')
    buildings_right = sample_vertical_plane(num_samples_building / 2, length_road, height_buildings, [+width_road/2, -length_road/2, -height_camera/2], 'yz')
    city = np.concatenate((road, buildings_left, buildings_right), axis=0)
    return city

def plot_environment(samples):
     
    fig = plt.figure(figsize=(8,5))
    ax = Axes3D(fig, xlabel='x [m]', ylabel='y [m]', zlabel='z [m]')
    ax.plot(samples[:,0], samples[:,1], samples[:,2], 'b.')
    plot_utils.draw_coordinate_frame(ax, [0,0,0], np.identity(3), 1.0)
    plot_utils.axis_equal_3d(ax)
    
if __name__ == '__main__':
    samples = sample_city(3000)
    plot_environment(samples)