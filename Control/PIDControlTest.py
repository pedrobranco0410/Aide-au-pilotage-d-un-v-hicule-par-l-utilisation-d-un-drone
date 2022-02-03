"""
Code responsible for testing the performance of the speed PID controller developed by the drone. 
This code only tests the controller with reliable data and the result is not influenced by any 
other vision processing code etc. The drone will be subjected to different trajectories and we 
will calculate the accumulated errors to issue a final evaluation of the code's performance.
"""

import sys
sys.path.append('../')
from Simulation.Drone import Drone
from Simulation.Tank import Tank
import PIDControl
import math
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

#Starting the node that will listen and publish the messages
rospy.init_node('testing_pid', anonymous=True)

#Initializing the drone
drone = Drone()

#Initializig the Tank
tank = Tank("suv")


#Initializing the PID controllers
#TO DO


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
    target_positions += [drone.getTargetPosition(tank_positions[-1],tank_ori,1)]
        
    #Calculate PID Commands for the drone
    '''CHANGE THAT FOR THE PID'''
    d_vel[0] = (target_positions[-1][0] - drone_positions[-1][0]) 
    d_vel[1] = (target_positions[-1][1] - drone_positions[-1][1]) 
    d_vel[2] = (target_positions[-1][2] - drone_positions[-1][2]) 

    #Calculate PID Commands for the camera
    #Todo

    #Send commands to Drone
    drone.setDroneSpeed(d_vel,[0,0,0])

    #Sendo Commands to Camera
    drone.setCameraOrientation(0, tank_ori[2])

    #Getting simulation time
    time += 0.003

    #Show image
    cv2.imshow("camera", image)
    cv2.waitKey(2)


plt.plot(range(len(drone_positions)), np.array(drone_positions)[:,0], 'r') # plotting t, a separately 
plt.plot(range(len(drone_positions)), np.array(tank_positions)[:,0], 'b') # plotting t, b separately 
plt.plot(range(len(drone_positions)), np.array(target_positions)[:,0], 'g') # plotting t, c separately 
plt.show()