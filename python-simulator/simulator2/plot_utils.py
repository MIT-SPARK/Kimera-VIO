#!/usr/bin/env python3
"""
Zurich Eye
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def axis_equal_3d(ax):
    """Makes all axes of a 3D plot of same scale.
    """
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)
        
def draw_coordinate_frame(ax, t_world_body, R_world_body, alpha=1.0):
    """Draws the coordinate frame in the provided figure-axis (world coordinates)
    """
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
            
def circles(x, y, s, c='b', ax=None, vmin=None, vmax=None, **kwargs):
    """
    Make a scatter of circles plot of x vs y, where x and y are sequence 
    like objects of the same lengths. The size of circles are in data scale.

    Parameters
    ----------
    x,y : scalar or array_like, shape (n, )
        Input data
    s : scalar or array_like, shape (n, ) 
        Radius of circle in data scale (ie. in data unit)
    c : color or sequence of color, optional, default : 'b'
        `c` can be a single color format string, or a sequence of color
        specifications of length `N`, or a sequence of `N` numbers to be
        mapped to colors using the `cmap` and `norm` specified via kwargs.
        Note that `c` should not be a single numeric RGB or
        RGBA sequence because that is indistinguishable from an array of
        values to be colormapped.  `c` can be a 2-D array in which the
        rows are RGB or RGBA, however.
    ax : Axes object, optional, default: None
        Parent axes of the plot. It uses gca() if not specified.
    vmin, vmax : scalar, optional, default: None
        `vmin` and `vmax` are used in conjunction with `norm` to normalize
        luminance data.  If either are `None`, the min and max of the
        color array is used.  (Note if you pass a `norm` instance, your
        settings for `vmin` and `vmax` will be ignored.)

    Returns
    -------
    paths : `~matplotlib.collections.PathCollection`

    Other parameters
    ----------------
    kwargs : `~matplotlib.collections.Collection` properties
        eg. alpha, edgecolors, facecolors, linewidths, linestyles, norm, cmap

    Examples
    --------
    a = np.arange(11)
    circles(a, a, a*0.2, c=a, alpha=0.5, edgecolor='none')

    License
    --------
    This code is under [The BSD 3-Clause License]
    (http://opensource.org/licenses/BSD-3-Clause)
    """
    from matplotlib.patches import Circle
    from matplotlib.collections import PatchCollection
    if ax is None:
        ax = plt.gca()    

 
    color = None  # use cmap, norm after collection is created
    kwargs.update(color=color)

    if np.isscalar(x):
        patches = [Circle((x, y), s, edgecolor='none'),]
    elif np.isscalar(s):
        patches = [Circle((x_,y_), s, edgecolor='none') for x_,y_ in zip(x,y)]
    else:
        patches = [Circle((x_,y_), s_, edgecolor='none') for x_,y_,s_ in zip(x,y,s)]
    collection = PatchCollection(patches, **kwargs)

    if color is None:
        collection.set_array(np.asarray(c))
        if vmin is not None or vmax is not None:
            collection.set_clim(vmin, vmax)

    ax.add_collection(collection)
    ax.autoscale_view()
    return collection