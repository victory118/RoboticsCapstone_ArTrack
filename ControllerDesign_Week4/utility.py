#!/usr/bin/env python
"""
Utility functions that are commonly used across the visual_servoing package.
Used mostly for extending coordinate transformations beyond the scope of transformations.py.
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)
"""
import numpy as np
import math

import roslib
from std_msgs.msg import (
    Header,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Quaternion,
)
from tf.transformations import *

def get_t_R(pose):
    """
    Returns the translation vector (4x1) and rotation matrix (4x4) from a pose message
    """
    t=np.transpose(np.matrix([pose.position.x,pose.position.y,pose.position.z,0]))
    quat=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_full=quaternion_matrix(quat)
    R=R_full
    return t,R

def make_pose_stamped_msg(t,R):
    """
    Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
    NOTE: Does not set the target frame.
    """
    pose_stamped_msg=PoseStamped()
    pose_stamped_msg.header=Header()
    pose_stamped_msg.header.stamp=rospy.Time.now()
    pose_msg=Pose()
    pose_msg.position.x=t[0]
    pose_msg.position.y=t[1]
    pose_msg.position.z=t[2]
    quat=quaternion_from_matrix(R)
    pose_msg.orientation.x=quat[0]
    pose_msg.orientation.y=quat[1]
    pose_msg.orientation.z=quat[2]
    pose_msg.orientation.w=quat[3]
    pose_stamped_msg.pose=pose_msg
    return pose_stamped_msg

def get_transform_matrix(X):
    """
    Given an X = [x,y,theta], create associated transform
    Inputs: X - an array of size 3 with [x,y,theta] in it
    Output: H - a 3 by 3 numpy array of homogeneous representation rotation
                and translation

    Note: This helper method is part of the RobotSim class from RobotSim.py
    """
    return np.array([[np.cos(X[2]), -np.sin(X[2]), X[0]],
                    [np.sin(X[2]),  np.cos(X[2]), X[1]],
                    [         0.0,           0.0,  1.0]])
    
def get_pose_from_transform(H):
    """
    Given H created from H(X), extract the X
    Inputs: H - a 3 by 3 numpy array of homogeneous representation rotation
                and translation
    Outpus: X - an array of size 3 of [x,y,theta] of transformation

    Note: This helper method is part of the RobotSim class from RobotSim.py
    """
    return [H[0,2],H[1,2],math.atan2(H[1,0],H[0,0])]