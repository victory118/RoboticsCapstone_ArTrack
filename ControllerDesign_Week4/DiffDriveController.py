#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp=2
        self.ka=10 # 10
        self.kb=0 # 0
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        #pass

        dx = goal[0] - state[0] # x displacement of tag measured at robot in meters
        dy = goal[1] - state[1] # y displacement of tag measured at robot in meters
        theta = -state[2] # angle offset of robot X-axis to AprilTag X-axis
	    
        rho = np.sqrt(dx*dx + dy*dy)
        #print rho
        alpha = -theta + np.arctan2(dy,dx)
        beta = -theta - alpha
        done = False

        if abs(rho) < 0.15:
            #print('in here')
            v = 0.0
            #print v
            omega = 0.0
            #print omega
            done = True
            #print done
            return v, omega, done
        else:
            v = self.kp*rho
            omega = self.ka*alpha + self.kb*beta
            
	    if abs(v) > self.MAX_SPEED:
	        v = self.MAX_SPEED*v/abs(v)
	        
        if abs(omega) > self.MAX_OMEGA:
            omega = self.MAX_OMEGA*omega/abs(omega)
	    
	    #print done
	    #print('now here')
	    return v, omega, done

