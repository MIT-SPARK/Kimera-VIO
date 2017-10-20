#!/usr/bin/python

import numpy as np
import math_utils

class PinholeCamera:
    """ Pinhole Camera Model """
         
    def __init__(self, width, height, fx, fy, cx, cy):
        self.width = width
        self.height = height
        self.focal_length = np.array([fx, fy])
        self.principle_point = np.array([cx, cy])
    
    def back_project(self, px):
        """Computes the bearing-vector from the pixel measurement. Z-Component is 1"""
        uv = 1.0/self.focal_length * (px - self.principle_point)
        f = math_utils.unproject(uv)
        return f
            
    def project(self, xyz):
        """Projects a euclidean point to a 2d image measurement"""
        if(xyz[2] < 0):
            return np.zeros(2), False
        uv = math_utils.project(xyz[:3])
        px = uv*self.focal_length + self.principle_point
        visible = (px[0] >= 0 and px[0] < self.width and px[1] >= 0 and px[1] < self.height)      
        return px, visible