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
import utility as util

# TODO for student: uncomment when changing to the robot
# from ros_interface import ROSInterface

# TODO for student: User files, uncomment as completed
#from MyShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body, sample_time):
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
        sample_time - the sample time in seconds between each measurement (added by me)
        """

        # TODO for student: Comment this when running on the robot 
        self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                  max_speed, max_omega, x_spacing, y_spacing)
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        #self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS

        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map, sample_time)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        self.v_last = 0.0
        self.omega_last = 0.0

    def process_measurements(self, goal):
        """ 
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and esimtaiton
        are done. This function is called at 60Hz
        """
        # TODO for student: Comment this when running on the robot 
        meas = np.array(self.robot_sim.get_measurements()) # measured pose of tag in robot frame (x,y,theta,id,time)
        imu_meas = self.robot_sim.get_imu() # 5 by 1 numpy vector (acc_x, acc_y, acc_z, omega, time)
        
        # Do KalmanFilter step
        # Note: The imu_meas could be None if it is sampled at a lower rate than the integration time step.
        # For the simulation, assume that imu_meas will always return a valid value because we control it in RobotSim.py.
        # For running on the robot, you should include protection for potentially bad measurements.  
        pose_est = self.kalman_filter.step_filter(self.v_last, imu_meas, meas)

        at_goal = False
        
        v, omega, at_goal = self.diff_drive_controller.compute_vel(pose_est, goal)
        self.robot_sim.command_velocity(v, omega)
        self.v_last = v
        self.omega_last = omega

        #print("estimated state = ", est_state)
        #print("true state = ", self.robot_sim.get_gt_pose())
        # Draw ghost robot
        est_state = np.array([[pose_est[0]],[pose_est[1]],[pose_est[2]]])
        self.robot_sim.set_est_state(est_state)
                   
        # TODO for student: Use this when transferring code to robot
        # meas = self.ros_interface.get_measurements()
        # imu_meas = self.ros_interface.get_imu()

        return at_goal
    
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
    waypoints = np.array(params['waypoints'])
    print "waypoints = ", waypoints

    pos_goal = pos_goal.reshape(len(pos_goal)) # reshape to (3,)

    # Intialize the RobotControl object
    sample_time = 0.05 # simulation time step
    # sample_time = 1/60 # when using ROS with sample rate 60 Hz
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body, sample_time)

    # TODO for student: Comment this when running on the robot 
    # Run the simulation
    goal = pos_goal.reshape(len(pos_goal))
    at_goal = False
    route_done = False
    waypoint_idx = 0
    while not robotControl.robot_sim.done and plt.get_fignums() and not route_done:
        goal = waypoints[waypoint_idx]
        # print "goal = ", goal
        at_goal = robotControl.process_measurements(goal)    
        robotControl.robot_sim.update_frame()

        if at_goal: # at the current waypoint, move to next one
            waypoint_idx += 1
            at_goal = False
            if waypoint_idx >= waypoints.shape[0]: # finished all waypoints
                route_done = True

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


