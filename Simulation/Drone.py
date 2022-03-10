"""
Class responsible for simulating a drone with a mobile camera attached to a gimble. It contains all the methods needed to 
communicate with the simulated model in the gazebo, as well as retrieve and change its data and parameters.
"""

import rospy
from gazebo_msgs.srv    import GetModelState,SetModelState
from gazebo_msgs.msg    import ModelState
from geometry_msgs.msg  import Twist
from std_msgs.msg       import Float64
from sensor_msgs.msg    import CompressedImage, Imu
from rosgraph_msgs.msg  import Clock
import time
import math 
import cv2
import numpy as np
import random

class Drone():


    def __init__(self):

        #Variables responsible for storing the drone's speed 
        self._linear_speed = [0,0,0]
        self._angular_speed = [0,0,0]

        #variables for camera rotation (radian)
        self._pitch = 0
        self._yaw = 0

        self.image =  np.zeros((100,100,3), dtype=np.uint8)
        self.t_now = 0
        self.t_begin =0

        self.drone_name = "uav1"

        #Setting up the topic that will be responsible for sending the speed commands to the drone
        self.cmd_vel_pub = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=10)

        #Setting up the topic that will be responsible for sending the orientation commands to the camera
        self.cmd_pitch_pub = rospy.Publisher('/uav1/gimbal_pitch_controller/command', Float64, queue_size=10)
        self.cmd_yaw_pub = rospy.Publisher('/uav1/gimbal_yaw_controller/command', Float64, queue_size=10)
        self.cmd_roll_pub = rospy.Publisher('/uav1/gimbal_roll_controller/command', Float64, queue_size=10)

        #Setting up the topic that will be responsible for recover the camera image and the imu data
        self.camera_sub = rospy.Subscriber("/uav1/camera/dji_sdk/image_raw/compressed", CompressedImage, self.img_callback,  queue_size = 1)
        self.imu_sub = rospy.Subscriber('/uav1/imu', Imu, self.imu_callback,  queue_size = 1)

        #Simulation functions and topics
        self.clock_sub =  rospy.Subscriber('/clock', Clock, self.clock_callback,queue_size = 1)
        self.resetSimulation()


    
    ############Drone Functions############

    def setDroneSpeed(self, linear, angular):
        '''
                Function responsible for changing the speed of the drone. The drone will maintain the speed until it is changed by a 
                new call of this same function.
           
            Inputs:
                    "linear"  -> 3-element list representing the linear velocity of the drone [Vx, Vy, Vz]
                    "angular" -> 3-element list representing the linear velocity of the drone [Wx, wy, Wz]
            Outputs:
                
        '''
        self._linear_speed = linear
        self._angular_speed = angular

        vel_msg = Twist()
        vel_msg.linear.x = linear[0]
        vel_msg.linear.y = linear[1]
        vel_msg.linear.z = linear[2]
        vel_msg.angular.x = angular[0]
        vel_msg.angular.y = angular[1]
        vel_msg.angular.z = angular[2]

        

        self.cmd_vel_pub.publish(vel_msg)

    def getDroneSpeed(self):
        '''
            The function will return the momentary linear and angular velocities of the drone
           
            Inputs:
                   
            Outputs:
                -2x3 matrix containing the two momentary linear and angular velocity vectors of the drone [[Vx, Vy, Vz],[Wx, Wy, Wz]]   
        '''
        return [self._linear_speed,self._angular_speed]
 
    def setDronePosition(self, position, orientation):
            '''
                    Function responsible for changing the drone position and orientation
            
                Inputs:
                        "position"  -> 3-element list representing the position of the drone [Px, Py, Pz]
                        "orientation" -> 3-element list representing the orientation of the drone [Ox, Oy, Oz]
                Outputs:
                
            '''
            state_msg = ModelState()
            state_msg.model_name = self.drone_name
            state_msg.pose.position.x = position[0]
            state_msg.pose.position.y = position[1]
            state_msg.pose.position.z = position[2]
            state_msg.pose.orientation.x = orientation[0]
            state_msg.pose.orientation.y = orientation[1]
            state_msg.pose.orientation.z = orientation[2]
            state_msg.pose.orientation.w = 0
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state( state_msg )

    def getDronePosition(self):
        '''
            The function will return the position of the drone
           
            Inputs:
                   
            Outputs:
                -1x3 matrix containing the position [Px, Py, Pz] 
        '''
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        object_coordinates = model_coordinates(self.drone_name, "")

        return [object_coordinates.pose.position.x,object_coordinates.pose.position.y,object_coordinates.pose.position.z]

    def getDroneOrientation(self):
        '''
            The function will return the orientation of the drone
           
            Inputs:
                   
            Outputs:
                -1x3 matrix containing the orientation [Px, Py, Pz] 
        '''
        model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        object_coordinates = model_coordinates(self.drone_name, "")

        return [object_coordinates.pose.orientation.x,object_coordinates.pose.orientation.y,2*math.asin(object_coordinates.pose.orientation.z)]
    
    def getTargetPosition(self, position, orientation, view):
        '''
            The function will return the taget position based on the wanted view
           
            Inputs:
                -position: matrix containing the position of the target [Px, Py, Pz]
                -orientation: matrix containing the  orientation of the target [Ox, Oy, Oz]
                -view: {0:Top, 1:behind, 2:front, 3:left: 4:right}
                   
            Outputs:
                -1x3 matrix containing the target position [Px, Py, Pz] 
        '''

        radius = 7

        t_pose = position.copy()
        

        if(view == 0): 
            t_pose[2] += radius

        elif(view == 1):
            t_pose[0] -= radius*math.cos(orientation[2])
            t_pose[1] -= radius*math.sin(orientation[2])
            t_pose[2] += radius

        elif(view == 2):
            t_pose[0] += radius*math.cos(orientation[2])
            t_pose[1] += radius*math.sin(orientation[2])
            t_pose[2] += radius
        
        elif(view == 3):
            t_pose[1] -= radius*math.cos(orientation[2])
            t_pose[0] -= radius*math.sin(orientation[2])
        
        elif(view == 4):
            t_pose[1] += radius*math.cos(orientation[2])
            t_pose[0] += radius*math.sin(orientation[2])
        
        #t_pose[2] = 10

        return t_pose.copy()

    #todo
    def take_off(self):
        return
    
    #todo
    def land_on(self):
        return


    ############Camera adn Sensor Functions############

    def setCameraOrientation(self, pitch, yaw, radian=True):
        '''
                Function responsible for changing the orientation of the camera
           
            Inputs:
                    "pitch"  ->  rotation in the y-axis in degrees or radians [0,pi] -[0,180]
                    "yaw"    ->  rotation in the z-axis in degress or radians [-2pi,2pi] - [-360,360]
                    "radian" ->  bool to indicate if the values are in degrees or radians
            Outputs:
                
        '''

        #Change for radians if it is in degrees
        if(not radian):
            self._pitch = pitch * math.pi / 180
            self._yaw = yaw *  math.pi / 180
        else:
            self._pitch = pitch
            self._yaw = yaw

        #Verify limits
        if(pitch < 0):self._pitch = 0
        if(pitch > math.pi): self._pitch = math.pi
        if(yaw < -1):self._yaw = -1
        if(yaw > 1): self._yaw = 1

        p = Float64()
        p.data = self._pitch
        y = Float64()
        y.data = self._yaw
        r =  Float64()
        r.data = 0
        
        self.cmd_pitch_pub.publish(p)
        self.cmd_yaw_pub.publish(y)
        self.cmd_roll_pub.publish(r)

        return

    def getCameraOrientation(self, degree=False):
        '''
            The function will return the orientation of the camera
           
            Inputs:
                   degree - boolean to indicate if the values are in radians or degrees
            Outputs:
                -pitch, yaw in radians or degrees 
        '''


        if(degree):
            return _pitch*180/math.pi, _yaw*180/math.pi
        return  self._pitch, self._yaw

    def getCameraImage(self):
        '''
                Function responsible for get the camera image
           
            Inputs:

            Outputs:
                    -image in CV2
        '''
        return self.image

    def img_callback(self, ros_data):

        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.image = cv2.imdecode(np_arr,  cv2.IMREAD_COLOR)

    #todo
    def getIMUData(self):
        return

    #todo
    def imu_callback(self, ros_data):
        '''
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu'''
        return

    ##########Simulation Functions###########

    def clock_callback(self, ros_data):
        if(self.t_begin == 0):
             self.t_begin = float(str(ros_data.clock))/1000000000
        self.t_now = float(str(ros_data.clock))/1000000000

    def getSimulationTime(self):
        return self.t_now - self.t_begin

    def resetSimulation(self):
        self.t_begin = self.t_now
        self.setDroneSpeed([0,0,1],[0,0,0])
        self.setDroneSpeed([0,0,0],[0,0,0])
        self.setDronePosition([1,0,1],[0,0,0])

