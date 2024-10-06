"""
Transforms Module - Contains code for to learn about rotations
and eventually homogenous transforms. 

Empty outline derived from code written by John Morrell, former TA. 
"""

import numpy as np
import sympy as sm
from numpy import sin, cos, sqrt
from numpy.linalg import norm

## 2D Rotations
def rot2(th):
    """
    R = rot2(theta)
    Parameters
        theta: float or int, angle of rotation
    Returns
        R: 2 x 2 numpy array representing rotation in 2D by theta
    """

    ## TODO - Fill this out
    R = sm.Matrix([[sm.cos(th), -sm.sin(th)],
                   [sm.sin(th), sm.cos(th)]])
    return R

## 3D Transformations
def rotx(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    ## TODO - Fill this out
    R = sm.Matrix([[1, 0, 0],
                   [0, sm.cos(th), -sm.sin(th)],
                   [0, sm.sin(th), sm.cos(th)]])

    return R

def roty(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """
    ## TODO - Fill this out
    R = sm.Matrix([[sm.cos(th), 0, sm.sin(th)],
                  [0, 1, 0],
                  [-sm.sin(th), 0, sm.cos(th)]])

    return R

def rotz(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """

    ## TODO - Fill this out
    R = sm.Matrix([[sm.cos(th), -sm.sin(th), 0],
                   [sm.sin(th), sm.cos(th), 0],
                   [0, 0, 1]])

    return R

# inverse of rotation matrix 
def rot_inv(R):
    '''
    R = rot_inv(R)
    Parameters
        R: 2x2 or 3x3 numpy array representing a proper rotation matrix
    Returns
        R: 2x2 or 3x3 inverse of the input rotation matrix
    '''
    ## TODO - Fill this out
    return R.T

def se3(R=sm.eye(3), p=sm.Matrix([0, 0, 0])):
    """
        T = se3(R, p)
        Description:
            Given a numpy 3x3 array for R, and a 1x3 or 3x1 array for p, 
            this function constructs a 4x4 homogeneous transformation 
            matrix "T". 

        Parameters:
        R - 3x3 numpy array representing orientation, defaults to identity
        p = 3x1 numpy array representing position, defaults to [0, 0, 0]

        Returns:
        T - 4x4 numpy array
    """
    # TODO - fill out "T"
    T = sm.eye(4)
    T[0:3,0:3] = R
    T[0:3, 3] = p

    return T

def inv(T):
    """
        Tinv = inv(T)
        Description:
        Returns the inverse transform to T

        Parameters:
        T

        Returns:
        Tinv - 4x4 numpy array that is the inverse to T so that T @ Tinv = I
    """
    
    #TODO - fill this out 
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    R_inv = R.T
    p_inv = -R_inv @ p

    T_inv = sm.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = p_inv

    return T_inv
