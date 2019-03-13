#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""
#import rospy

import yaml
import numpy as np

import sys

# TODO for student: Comment this section when running on the robot 
from RobotSim import RobotSim
import matplotlib.pyplot as plt

import math

# TODO for student: uncomment when changing to the robot
# from ros_interface import ROSInterface

# TODO for student: User files, uncomment as completed
#from MyShortestPath import dijkstras
#from KalmanFilter import KalmanFilterS
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        Inputs: (all loaded from the parameter YAML file)
        world_map - a P by 4 numpy array specifying the location, orientation,
            and identification of all the markers/AprilTags in the world. The
            format of each row is (x,y,theta,id) with x,y giving 2D position,
            theta giving orientation, and id being an integer specifying the
            unique identifier of the tag.
        occupancy_map - an N by M numpy array of boolean values (represented as
            integers of either 0 or 1). This represents the parts of the map
            that have obstacles. It is mapped to metric coordinates via
            x_spacing and y_spacing
        pos_init - a 3 by 1 array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - a 3 by 1 array specifying the final position of the robot,
            also formatted as (x,y,theta)
        max_speed - a parameter specifying the maximum forward speed the robot
            can go (i.e. maximum control signal for v)
        max_omega - a parameter specifying the maximum angular speed the robot
            can go (i.e. maximum control signal for omega)
        x_spacing - a parameter specifying the spacing between adjacent columns
            of occupancy_map
        y_spacing - a parameter specifying the spacing between adjacent rows
            of occupancy_map
        t_cam_to_body - numpy transformation between the camera and the robot
            (not used in simulation)
        """
        
        # TODO for student: Comment this when running on the robot 
        self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                  max_speed, max_omega, x_spacing, y_spacing)
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        #self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        self.pos_goal = pos_goal # (x,y,theta) specifying final position of robot
        
        # Uncomment as completed
        #self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def get_transform_matrix(self,X):
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
    
    def get_pose_from_transform(self,H):
        """
        Given H created from H(X), extract the X
        Inputs: H - a 3 by 3 numpy array of homogeneous representation rotation
                    and translation
        Outpus: X - an array of size 3 of [x,y,theta] of transformation

        Note: This helper method is part of the RobotSim class from RobotSim.py
        """
        return [H[0,2],H[1,2],math.atan2(H[1,0],H[0,0])]

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and esimtaiton
        are done. This function is called at 60Hz
        """
        # TODO for student: Comment this when running on the robot

        # (x,y,theta,id,time) with x,y being the 2D
        # position of the marker relative to the robot, theta being the
        # relative orientation of the marker with respect to the robot, id
        # being the identifier from the map, and time being the current time
        # stamp. If no tags are seen, the function returns None. 
        meas = self.robot_sim.get_measurements()
        imu_meas = self.robot_sim.get_imu()

        done = False
        goal = self.pos_goal
        # print "goal = ", goal

        if True:
            state = self.robot_sim.get_gt_pose()
            v, omega, done = self.diff_drive_controller.compute_vel(state, goal)
        elif meas is not None and meas != []:
            #print meas
            #print type(meas)

            tag_id = int(meas[0][3]) # get the id of the tag seen
            # print "tag_id = ", tag_id

            pose_tag_in_robot = meas[0][0:3] # get pose of tag in robot frame
            H_RT = self.get_transform_matrix(pose_tag_in_robot) # get transform matrix of robot to tag frame
            print "markers = ", self.robot_sim.markers
            print "type(markers) = ", type(self.robot_sim.markers)
            print "markers[tag_id] = ", self.robot_sim.markers[tag_id]
            pose_tag_in_world = self.robot_sim.markers[tag_id, 0:3] # get pose of tag in world frame
            print "pose_tag_in_world = ", pose_tag_in_world
            H_WT = self.get_transform_matrix(pose_tag_in_world) # get transform matrix of world to tag frame
            H_TR = np.linalg.inv(H_RT) # get transform matrix of tag to robot frame
            H_WR = np.dot(H_WT, H_TR) # get transform matrix of world to robot frame

            pose_robot_in_world = self.get_pose_from_transform(H_WR)
            state = pose_robot_in_world
            print "state = ", state
            print "true pose = ", self.robot_sim.get_gt_pose()

            v, omega, done = self.diff_drive_controller.compute_vel(state, goal)
            #print done
        else:
            v = 0
            omega = 0

        self.robot_sim.command_velocity(v, omega)
            
        # TODO for student: Use this when transferring code to robot
        # meas = self.ros_interface.get_measurements()
        # imu_meas = self.ros_interface.get_imu()

        return done
    
def main(args):
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)

    # TODO for student: Comment this when running on the robot 
    # Run the simulation
    done = False
    while not robotControl.robot_sim.done and plt.get_fignums() and done is False:
        done = robotControl.process_measurements()
        #print done
        robotControl.robot_sim.update_frame()

    plt.ioff()
    plt.show()

    # TODO for student: Use this to run the interface on the robot
    # Call process_measurements at 60Hz
    """r = rospy.Rate(60)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)"""

if __name__ == "__main__":
    main(sys.argv)


