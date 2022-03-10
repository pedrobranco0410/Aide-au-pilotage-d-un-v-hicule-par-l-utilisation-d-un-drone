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
from PID_Drone import PID_3D
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
tank = Tank("husky")

#Initializing the PID controllers
pid = PID_3D(Kx=(2,0.1,1), Ky=(2,0.1,1), Kz=(2,0.1,1), cible=(5,0,0))


#Variables for store trajectory
drone_positions = []
tank_positions = []
target_positions = []

#Initializing the simulation and the controlled variable (drone speed)
d_vel = [0,0,0]
drone.resetSimulation()

#Control Loop
while (tank.followTankTraj(drone.getSimulationTime())):

    #Getting real positions and orientations from simulation
    drone_positions += [drone.getDronePosition()]
    tank_positions += [tank.getTankPosition()]
    tank_ori = tank.getTankOrientation()

    #Getting Image
    image = drone.getCameraImage()

    #Updating the drone's position to the controller
    pid.set_position(drone_positions[-1])

    #Calculate Target position based on real position and orientation
    target_positions += [drone.getTargetPosition(tank_positions[-1],tank_ori,1)]
    pid.set_target(target_positions[-1])
        
    #Calculate PID Commands for the drone
    pid.correction()
    d_vel[0] = pid.vx
    d_vel[1] = pid.vy
    d_vel[2] = pid.vz 

    #Send commands to Drone
    drone.setDroneSpeed(d_vel,[0,0,0])
    drone.setCameraOrientation(1.6,0) # the camera will always point down

    #Show image
    cv2.imshow('camera', image)
    cv2.waitKey(2)

cv2.destroyAllWindows()

#Just to check the path taken
#pid.print_PID_2D(drone_positions, target_positions, tank_positions)