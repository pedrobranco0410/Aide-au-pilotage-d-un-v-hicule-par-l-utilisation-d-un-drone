"""
Class responsible for simulating a drone with a mobile camera attached to a gimble. It contains all the methods needed to 
communicate with the simulated model in the gazebo, as well as retrieve and change its data and parameters.
"""

import roslibpy
import roslibpy
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
import time
import math

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

        return [object_coordinates.pose.orientation.x,object_coordinates.pose.orientation.y,object_coordinates.pose.orientation.z]

    def followTankTraj(self, time):

        if( time <= 10):
            self._linear_speed = [1.5*time,0,0]
            self._angular_speed = [0,0,0]

        elif(time > 10 and time <= 25):
            self._linear_speed = [1.5*time,0,0]
            self._angular_speed = [0,0,0]
        
        elif(time > 25 and time <= 30):
            self._linear_speed = [15 - (time - 25)*1.5 ,0,0]
            self._angular_speed = [0,0,0]
        
        elif(time > 30 and time <= 35):
            self._linear_speed = [7.5 ,0,0]
            self._angular_speed = [0,0,0.628]

        elif(time > 35 and time <= 40):
            self._linear_speed = [7.5 + (time - 35)*0.5 ,0,0]
            self._angular_speed = [0,0,0]
        
        elif(time > 40 and time <= 50):
            self._linear_speed = [10 ,0,0]
            self._angular_speed = [0,0,0]
        
        elif(time > 50 and time <= 60):
            self._linear_speed = [10 ,0,0]
            self._angular_speed = [0,0,0.2]
        
        elif(time > 60 and time <= 70):
            self._linear_speed = [10 ,0,0]
            self._angular_speed = [0,0,-0.2]
        
        elif(time > 70 and time <= 80):
            self._linear_speed = [10 - (time - 70)*0.5 ,0,0]
            self._angular_speed = [0,0,-0.5]
        
        elif(time > 80 and time <= 90):
            self._linear_speed = [5 ,0,0]
            self._angular_speed = [0,0,0]
        
        elif(time > 90 and time <= 95):
            self._linear_speed = [5 - (time - 90) ,0,0]
            self._angular_speed = [0,0,0.6]

        elif(time > 95 and time <= 100):
            self._linear_speed = [0 ,0,0]
            self._angular_speed = [0,0,0]
        else:
            return False


        delta = time - self._time
        self._time = time

        self._position[0] += math.cos(self._rotation[2])*self._linear_speed[0]*delta
        self._position[1] += math.sin(self._rotation[2])*self._linear_speed[0]*delta
        self._rotation[2] += self._angular_speed[2]*delta

        self.setTankPosition(self._position, self._rotation)
        return True


