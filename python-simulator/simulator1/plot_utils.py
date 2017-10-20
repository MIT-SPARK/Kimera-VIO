#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def axis_equal_3d(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)
        
def draw_coordinate_frame(ax, t_world_body, R_world_body, alpha=1.0):
    """draws the coordinate frame in the provided figure-axis (world coordinates)"""
    T_wf = np.zeros((3,4))
    T_wf[0:3,0:3] = R_world_body
    T_wf[0:3,3]   = t_world_body
    line_length = 0.1
    ax.plot([T_wf[0,3], T_wf[0,3]+T_wf[0,0]*line_length],
            [T_wf[1,3], T_wf[1,3]+T_wf[1,0]*line_length],
            [T_wf[2,3], T_wf[2,3]+T_wf[2,0]*line_length], color='r', alpha=alpha)
    ax.plot([T_wf[0,3], T_wf[0,3]+T_wf[0,1]*line_length],
            [T_wf[1,3], T_wf[1,3]+T_wf[1,1]*line_length],
            [T_wf[2,3], T_wf[2,3]+T_wf[2,1]*line_length], color='g', alpha=alpha)
    ax.plot([T_wf[0,3], T_wf[0,3]+T_wf[0,2]*line_length],
            [T_wf[1,3], T_wf[1,3]+T_wf[1,2]*line_length],
            [T_wf[2,3], T_wf[2,3]+T_wf[2,2]*line_length], color='b', alpha=alpha) 