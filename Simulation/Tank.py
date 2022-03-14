"""
Class responsible for simulating a the tank the we will use as reference for the drone.
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
        self._position = [8,0,0]
        self._rotation = [0,0,0]

        #Name of the model in simulation
        self.tank_name = name

        self._time = 0

        self.cmd_vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        
        #Initializing the tank in origion
        self.setTankPosition(self._position,self._rotation)
        self.setTankSpeed([0,0,0],[0,0,0])


    
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

        vel_msg = Twist()
        vel_msg.linear.x = linear[0]
        vel_msg.linear.y = linear[1]
        vel_msg.linear.z = linear[2]
        vel_msg.angular.x = angular[0]
        vel_msg.angular.y = angular[1]
        vel_msg.angular.z = angular[2]

        

        self.cmd_vel_pub.publish(vel_msg)

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
                
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        object_coordinates = model_coordinates(self.tank_name, "")

        return [object_coordinates.pose.position.x,object_coordinates.pose.position.y,object_coordinates.pose.position.z]

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

        return [object_coordinates.pose.orientation.x,object_coordinates.pose.orientation.y,2*math.asin(object_coordinates.pose.orientation.z)]

    def followTankTraj(self, time):

        if(time <= 5):
            self.setTankSpeed([0,0,0],[0,0,0])
            self.setTankPosition(self._position, self._rotation)

        elif(time <= 15):
            self.setTankSpeed([(time-5)*0.22,0,0],[0,0,0])
        elif(time <= 25):
            self.setTankSpeed([2.2,0,0],[0,0,0])
        elif(time <= 30):
            self.setTankSpeed([2.2 -(time-25)*0.4,0,0],[0,0,0])
        
        elif(time <= 35):
            self.setTankSpeed([0,0,0],[0,0,0.5])
        
        elif(time <= 40):
            self.setTankSpeed([(time -35)*0.4,0,0],[0,0,0])
        
        elif(time <= 90):
            self.setTankSpeed([2,0,0],[0,0,0])

        elif(time <= 95):
            self.setTankSpeed([2 - (time-90)*0.4,0,0],[0,0,0]) 

        if(time > 100):
            return False

        return True


