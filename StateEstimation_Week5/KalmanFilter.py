#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math
import utility as util

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers, sample_time):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements 
        self.Q_t = np.diag([0.1, 0.1])
        self.R_t = np.diag([0.01, 0.01, np.pi/180/2])
        # YOUR CODE HERE
        
        self.sample_time = sample_time
        self.x_t = np.zeros(3) # initialize state variables
        self.P_t = np.diag([10, 10, 10]) # initialize covariance matrix

    def limit_angle(self, angle):
        """
        Limit angle to [-pi, pi]
        """
        if angle > np.pi:
            angle -= 2*np.pi
        elif angle < -np.pi:
            angle += 2*np.pi
        
        return angle

    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the prediction of the state
        predicted_covariance - a 3 by 3 numpy array of the prediction of the
            covariance
        """
        # YOUR CODE HERE

        # print "self.x_t[0] = ", self.x_t[0]
        # print "v = ", v
        # print "np.cos(self.x_t[2]) =", np.cos(self.x_t[2])
        # print "v*np.cos(self.x_t[2])*self.sample_time = ", v*np.cos(self.x_t[2])*self.sample_time

        x_pre = self.x_t[0] + v*np.cos(self.x_t[2])*self.sample_time
        y_pre = self.x_t[1] + v*np.sin(self.x_t[2])*self.sample_time
        th_pre = self.x_t[2] + imu_meas[3][0]*self.sample_time

        ########### KEY STEP ############
        # Limit the angle to range [-pi, pi]
        th_pre = util.limit_angle(th_pre)
        
        #print("v = ", v)
        #print("dt = ", dt)
        # print "imu_meas = ", imu_meas
        # print "type(imu_meas) = ", type(imu_meas)
        # print "imu_meas.shape = ", imu_meas.shape
        # print "x_t = ", self.x_t
        # print "x_t.shape = ", self.x_t.shape
        # print "x_pre = ", x_pre
        # print "y_pre = ", y_pre
        # print "th_pre = ", th_pre
        
        dfdx = np.array([[1., 0., 0.]]) # derivative of prediction model with respect to x position
        dfdy = np.array([[0., 1., 0.]]) # derivative of prediction model with respect to y position
        dfdth = np.array([[-np.sin(self.x_t[2])*v*self.sample_time, np.cos(self.x_t[2])*v*self.sample_time, 1.]]) # deriv. wrt theta
        dfdq = np.transpose(np.concatenate((dfdx, dfdy, dfdth), axis=0)) # Jacobian of prediction model wrt states
        
        dfdnv = self.sample_time*np.array([[np.cos(self.x_t[2]), np.sin(self.x_t[2]), 0]]) # deriv. wrt velocity input uncertainty
        dfdnw = self.sample_time*np.array([[0., 0., 1.]]) # deriv. wrt omega input uncertainty
        dfdn = np.transpose(np.concatenate((dfdnv, dfdnw), axis=0)) # Jacobian of prediction model wrt input uncertainty
        
        # print "x_t (pre-concat) = ", self.x_t
        self.x_t = np.array([x_pre, y_pre, th_pre]) # predicted state
        # print "x_t (post-concat = ", self.x_t
        # print "dfdth = ", dfdth
        # print "dfdq = ", dfdq
        # print "dfdnv = ", dfdnv
        # print "dfdnw = ", dfdnw
        # print "dfdn = ", dfdn
        # print "P_t (pre-prediction) = ", self.P_t
        self.P_t = dfdq.dot(self.P_t).dot(dfdq.T) + dfdn.dot(self.Q_t).dot(dfdn.T) # predicted covariance
        # print "P_t (post-prediction) = ", self.P_t
        
        return self.x_t, self.P_t

    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        # YOUR CODE HERE
        
        dh = np.eye(3) # Jacobian of measurement model wrt states
        # print "P_t (pre-update) = ", self.P_t
        # print "num_meas = ", z_t.shape[0]
        for i in range(z_t.shape[0]):
            # print "meas_idx = ", i
            # print "P_t (pre-update) = ", self.P_t
            K = self.P_t.dot(dh.T).dot(np.linalg.inv(dh.dot(self.P_t).dot(dh.T) + self.R_t))
            # print "K = ", K

            # Using the pose of the observed tag in the robot frame, calculate the
            # pose of the robot in the world frame
            tag_id = int(z_t[i][3])
            # print "tag_id = ", tag_id
            pose_tag_in_robot = z_t[i][0:3] # get pose of tag in robot frame
            H_RT = util.get_transform_matrix(pose_tag_in_robot) # get transform matrix of robot to tag frame
            pose_tag_in_world = self.markers[tag_id, 0:3] # get pose of tag in world frame
            H_WT = util.get_transform_matrix(pose_tag_in_world) # get transform matrix of world to tag frame
            H_TR = np.linalg.inv(H_RT) # get transform matrix of tag to robot frame
            H_WR = np.dot(H_WT, H_TR) # get transform matrix of world to robot frame
            pose_robot_in_world = util.get_pose_from_transform(H_WR) # get pose of robot in world frame
            
            # print "x_t = ", self.x_t
            # print "pose_robot_in_world = ", pose_robot_in_world
            self.x_t = self.x_t + K.dot(np.array(pose_robot_in_world)-self.x_t)

            ########### KEY STEP ############
            # Limit the angle to range [-pi, pi]
            self.x_t[2] = util.limit_angle(self.x_t[2])

            # print "x_t.shape = ", self.x_t.shape
            self.P_t = (np.eye(3) - K.dot(dh)).dot(self.P_t)
            # print "P_t (post-update) = ", self.P_t
        
        # self.P_t = (np.eye(3) - K.dot(dh)).dot(self.P_t)
        # print "P_t (post-update) = ", self.P_t
        return self.x_t, self.P_t
        
    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # YOUR CODE HERE
        if imu_meas is not None:
            self.x_t, self.P_t = self.prediction(v, imu_meas)
        
        # print "z_t = ", z_t
        # print "type(z_t) = ", type(z_t)
        # print "z_t == None: ", z_t == None
        # print "z_t != None: ", z_t != None
        # print "z_t is None: ", z_t is None
        if len(z_t.shape) > 0:
            self.x_t, self.P_t = self.update(z_t)
            
        return self.x_t
            
