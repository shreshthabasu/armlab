"""!
Implements Forward and backwards kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm


def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """

    
    A = []
    for i in range(link):
        A.append(get_transform_from_dh(dh_params[i]))
    return np.eye(4)

def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transform matrix.
    """
    c_theta = np.cos(theta)
    c_alpha = np.cos(alpha)
    s_theta = np.sin(theta)
    s_alpha = np.sin(alpha)
    
    T = np.zeros((4,4))
    T[0,0] = c_theta
    T[0,1] = -s_theta * c_alpha
    T[0,2] = s_theta * s_alpha
    T[0,3] = a * c_theta
    T[1,0] = s_theta
    T[1,1] = c_theta * c_alpha
    T[1,2] = -c_theta * s_alpha
    T[1,3] = a * s_theta
    T[2,1] = s_alpha
    T[2,2] = c_alpha
    T[2,3] = d
    T[3,3] = 1

    return T

def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the Euler angles from a T matrix

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    # FIXME
    # This assumes that the upper right 3x3 matrix of T is the rotaion matrix 
    #used page 48 of spong for formlas
    #not 100% sure its correct
    if T[2,2] != 1 and T[2,2] != -1:
        theta = np.atan2(T[2,2], np.sqrt(1-T[2,2]*T[2,2]))
        psi = np.atan2(-T[2,0], T[2,1])
        phi = np.atan2(T[0,2], T[1,2])
    else:
        phi = 0
        if T[2,2] == -1:
            theta = np.pi
            psi = phi - np.atan2(-T[0,0], -T[0,1])
        else:
            theta = 0
            psi = -phi + np.atan2(-T[0,1], -T[0,2])
    #not sure if they need to be clamped
    euler_angles = [clamp(theta), clamp(psi), clamp(phi)]
    return euler_angles

def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the joint pose from a T matrix of the form (x,y,z,phi) where phi is
                rotation about base frame y-axis

    @param      T     transformation matrix

    @return     The pose from T.
    """
    return np.array([0, 0, 0, 0])


def FK_pox(joint_angles):
    """!
    @brief      Get a 4-tuple (x, y, z, phi) representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4-tuple (x, y, z, phi) representing the pose of the desired link note: phi is the euler
                angle about y in the base frame

    @param      joint_angles  The joint angles

    @return     a 4-tuple (x, y, z, phi) representing the pose of the desired link
    """
    
    return np.array([0, 0, 0, 0])

def to_s_matrix(w,v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    # FIXME
    # !!!! Very confused what w and v are 
    # and their sizes, 
    # I assume they are 1x3 in slides it says
    # s_mat shoudl be in se(3) but shows in s as a 4x4 !!!!!
    w_mat = np.zeros((3,3))
    w_mat[0,1] = -w[2]
    w_mat[0,2] = w[1]
    w_mat[1,0] = w[2]
    w_mat[1,2] = -w[0]
    w_mat[2,0] = -w[1]
    w_mat[2,1] = w[0]

    s_mat = np.vstack(np.hstack(w_mat, v), [0,0,0,0])
    return s_mat

#Isbael Made this function, it assumes s is 4x4 fnc expm()
def calc_e_from_s_mat(s_mat, theta):
    """!
    @brief      Converts s matrix to e^([s]*theta.

    @param      s_mat   [s]
    @param      theta

    @return     e^([s]*theta
    """


def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array x,y,z,phi to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose as np.array x,y,z,phi

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    return np.zeros((4,4))
