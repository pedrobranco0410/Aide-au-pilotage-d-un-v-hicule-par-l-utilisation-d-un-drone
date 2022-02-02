#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Code responsible for testing the performance of the speed PID controller developed by the drone. 
This code only tests the controller with reliable data and the result is not influenced by any 
other vision processing code etc. The drone will be subjected to different trajectories and we 
will calculate the accumulated errors to issue a final evaluation of the code's performance.
"""
"""
Class responsible for simulating a drone with a mobile camera attached to a gimble. It contains all the methods needed to 
communicate with the simulated model in the gazebo, as well as retrieve and change its data and parameters.
"""

#from http.client import ImproperConnectionState
import sys
sys.path.append('../')
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import math 
from Drone import Drone
from pid_3D import PID_3D
from Tank import Tank


#Starting the node that will listen and publish the messages
rospy.init_node('testing_pid', anonymous=True)

#Initializing the drone
drone = Drone()

#Initializig the Tank
tank = Tank("polaris_ranger_ev")


#Initializing the PID controllers
#TO DO
pid = PID_3D(Kx=(5,1,1), Ky=(5,1,1), Kz=(5,1,1), cible=(0,0,0))


#Variables
drone_positions = []
tank_positions = []
target_positions = []

d_vel = [0,0,0]

time = 0

#Control Loop
while (tank.followTankTraj(time)):

    #Getting Positions
    drone_positions += [drone.getDronePosition()]
    tank_positions += [tank.getTankPosition()]
    tank_ori = tank.getTankOrientation()

    #Getting Image
    image = drone.getCameraImage()

    #Calculate Target position
    """change that """
    target_positions += [[tank_positions[-1][0] -5 * math.cos(tank_ori[2]),
                         tank_positions[-1][1] -5 * math.sin(tank_ori[2]),
                         tank_positions[-1][2] + 5]]


    #Calculate PID Commands for the drone
    '''CHANGE THAT FOR THE PID'''
    pid.set_position(drone.getDronePosition())
    pid.set_target(target_positions[-1])
    pid.correction()
    d_vel[0] = pid.vx
    d_vel[1] = pid.vy
    d_vel[2] = pid.vz


    #Calculate PID Commands for the camera
    #Todo

    #Send commands to Drone
    drone.setDroneSpeed(d_vel,[0,0,0])

    #Sendo Commands to Camera
    drone.setCameraOrientation(0,0)

    #Getting simulation time
    time += 0.005

    #Show image
    """
    cv2.imshow("camera", image)
    cv2.waitKey(2)
    """

plt.figure()
plt.subplot(221)
plt.plot(range(len(drone_positions)), np.array(drone_positions)[:,0], 'r') # plotting t, a separately 
plt.plot(range(len(drone_positions)), np.array(tank_positions)[:,0], 'b') # plotting t, b separately 
plt.plot(range(len(drone_positions)), np.array(target_positions)[:,0], 'g') # plotting t, c separately 
plt.subplot(222)
plt.plot(range(len(drone_positions)), np.array(drone_positions)[:,1], 'r') # plotting t, a separately 
plt.plot(range(len(drone_positions)), np.array(tank_positions)[:,1], 'b') # plotting t, b separately 
plt.plot(range(len(drone_positions)), np.array(target_positions)[:,1], 'g') # plotting t, c separately 
plt.subplot(223)
plt.plot(range(len(drone_positions)), np.array(drone_positions)[:,2], 'r') # plotting t, a separately 
plt.plot(range(len(drone_positions)), np.array(tank_positions)[:,2], 'b') # plotting t, b separately 
plt.plot(range(len(drone_positions)), np.array(target_positions)[:,2], 'g') # plotting t, c separately 
plt.show()