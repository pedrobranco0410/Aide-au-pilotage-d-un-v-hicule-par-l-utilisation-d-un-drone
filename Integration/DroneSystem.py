"""


"""


import sys
sys.path.append('../')
from Simulation.Drone import Drone
from Simulation.Tank import Tank
from Vision.Recognition.TankRecognition4 import findTank
from Control.PID_Drone import PID_3D
from Control.PID_Camera import PID_2D
import math
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

#Starting the node that will listen and publish the messages
rospy.init_node('testing_pid', anonymous=True)

#Initializing the Drone
drone = Drone()

#Initializig the Tank
tank = Tank("husky")

#Initializing the PID controllers
pid_D = PID_3D(Kx=(2,0.1,1), Ky=(2,0.1,1), Kz=(2,0.1,1), cible=(5,0,0))
pid_C = PID_2D(Kx=(2,0.1,1), Ky=(0.1,0,0), cible=(320,240))


#Variables for store traj
drone_positions = []
tank_positions = []
target_positions = []
rect_position = []

#
d_vel = [0,0,0]
c_posi = [1.6,0]
drone.resetSimulation()

#Control Loop
while (tank.followTankTraj(drone.getSimulationTime())):

        #Getting Image from camera
        image = drone.getCameraImage()
        
        ''' VISION '''

        #Getting Rectangle in image
        x, y, rect, w, h = findTank(image)

        #Getting Tank Distance 
        t_distance = 0

        #Getting Tank Orientation
        t_orientation = 0

        ''' SIMULATION '''

        #Getting drone real Data
        drone_positions += [drone.getDronePosition()]
        drone_ori = drone.getDroneOrientation()

        #Getting Tank real Data
        tank_positions += [tank.getTankPosition()]
        tank_ori = tank.getTankOrientation()

        #Getting real distance
        r_distance = (drone_positions[-1][0] - tank_positions[-1][0])**2 + (drone_positions[-1][1] - tank_positions[-1][1])**2 +(drone_positions[-1][2] - tank_positions[-1][1])**2 
        r_distance = math.sqrt(r_distance)


        ''' CONTROL '''

        #In the first 7 sec we will simulate a remoted control with the take off
        if(drone.getSimulationTime() < 7):

            #Setting Drone Position to Control
            pid_D.set_position(drone_positions[-1])

            #Calculate Target position
            target_positions += [drone.getTargetPosition(tank_positions[-1],tank_ori,0)]
            pid_D.set_target(target_positions[-1])

            c_posi = [1.6,0]
            
        else:

            #Setting Drone Position to Control
            pid_D.set_position(drone_positions[-1])

            #Calculate Target position
            t_z = drone_positions[-1][2] - (r_distance * math.cos(c_posi[0]))
            t_x = tank_positions[-1][0]#r_distance * math.sin(c_posi[0]) *  math.cos(c_posi[1])
            t_y = tank_positions[-1][1]#r_distance * math.sin(c_posi[0]) *  math.sin(c_posi[1])
            
            target_positions += [drone.getTargetPosition([t_x, t_y, t_z],tank_ori,0)]
            pid_D.set_target(target_positions[-1])

            #Calculate camera control commands
            c_posi[0] += (y - 240) * 0.0005
            c_posi[1] -= (x - 320) * 0.0005
            c_posi[0] = max(c_posi[0], 1)
            c_posi[0] = min(c_posi[0], 2)

    


        #Calculate PID Commands for the drone
        pid_D.correction()
        d_vel[0] = pid_D.vx
        d_vel[1] = pid_D.vy
        d_vel[2] = pid_D.vz 


        #Send commands to Drone
        drone.setDroneSpeed(d_vel,[0,0,0])
        drone.setCameraOrientation(c_posi[0],c_posi[1])

cv2.destroyAllWindows()

