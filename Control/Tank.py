#!/usr/bin/env python

"""
Class responsible for simulating a drone with a mobile camera attached to a gimble. It contains all the methods needed to 
communicate with the simulated model in the gazebo, as well as retrieve and change its data and parameters.
"""

import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
import time
import math

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class Tank():


    def __init__(self, name):

        #Variables responsible for storing the tank's speed
        self._linear_speed = 0
        self._angular_speed = 0

        #Variables responsible for storing position and orientation
        self._position = [4,0,0]
        self._rotation = [0,0,0]
        self.tank_name = name

        self._time = 0

        self.image = [0]


    
    ############Drone Functions############

    def setTankSpeed(self, linear, angular):
        '''
                Function responsible for changing the speed of the drone. The drone will maintain the speed until it is changed by a 
                new call of this same function.
           
            Inputs:
                    "linear"  -> 3-element list representing the linear velocity of the drone [Vx, Vy, Vz]
                    "angular" -> 3-element list representing the linear velocity of the drone [Wx, wy, Wz]
            Outputs:
                
        '''

    def getTankSpeed(self):
        '''
            The function will return the momentary linear and angular velocities of the tank
           
            Inputs:
                   
            Outputs:
                -2x3 matrix containing the two momentary linear and angular velocity vectors of the tank (V , W)
        '''
        return self._linear_speed,self._angular_speed
 
    def setTankPosition(self, position, orientation):
            '''
                    Function responsible for changing the tank position and orientation
            
                Inputs:
                        "position"  -> 3-element list representing the position of the tank [Px, Py, Pz]
                        "orientation" -> 3-element list representing the orientation of the tank [Ox, Oy, Oz]
                Outputs:
                
            '''
            state_msg = ModelState()
            state_msg.model_name = self.tank_name
            state_msg.pose.position.x = position[0]
            state_msg.pose.position.y = position[1]
            state_msg.pose.position.z = position[2]
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = math.sin(orientation[2]/2)
            state_msg.pose.orientation.w = math.cos(orientation[2]/2)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state( state_msg )

    def getTankPosition(self):
        '''
            The function will return the position of the Tank
           
            Inputs:
                   
            Outputs:
                -1x3 matrix containing the position [Px, Py, Pz] 
        '''

        return self._position.copy()

    def getTankOrientation(self):
        '''
            The function will return the orientation of the Tank
           
            Inputs:
                   
            Outputs:
                -1x3 matrix containing the orientation [Px, Py, Pz] 
        '''

        model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        object_coordinates = model_coordinates(self.tank_name, "")
        euler = euler_from_quaternion(object_coordinates.pose.orientation.x, object_coordinates.pose.orientation.y, object_coordinates.pose.orientation.z, object_coordinates.pose.orientation.w)

        return euler

    def followTankTraj(self, time):

        if( time <= 1):
            self._linear_speed = [1.5*time,0,0]
            self._angular_speed = [0,0,0]

        elif(time > 1 and time <= 3):
            self._linear_speed = [10,0,0]
            self._angular_speed = [0,0,0]
        
        elif(time > 3 and time <= 5):
            self._linear_speed = [10,0,0]
            self._angular_speed = [0,0,2]
        
        elif(time > 5 and time <= 7):
            self._linear_speed = [2 ,0,0]
            self._angular_speed = [0,0,5]

        elif(time > 7 and time <= 10):
            self._linear_speed = [5 ,0,0]
            self._angular_speed = [0,0,-1]
        
        else:
            return False


        delta = time - self._time
        self._time = time

        self._position[0] += math.cos(self._rotation[2])*self._linear_speed[0]*delta
        self._position[1] += math.sin(self._rotation[2])*self._linear_speed[0]*delta
        self._rotation[2] += self._angular_speed[2]*delta

        self.setTankPosition(self._position, self._rotation)
        return True


