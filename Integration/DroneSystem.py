"""


"""


import sys
sys.path.append('../')
from Simulation.Drone import Drone
from Simulation.Tank import Tank
from Vision.Recognition.TankRecognition3 import findTank
from Vision.Distance.Telemeter import getDistance
from Control.PID_Drone import PID_3D
from Control.PID_Camera import PID_2D
import math
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import random 

#Starting the node that will listen and publish the messages
rospy.init_node('testing_pid', anonymous=True)

#Initializing the Drone
drone = Drone()

#Initializig the Tank
tank = Tank("husky")

#Initializing the PID controllers
pid_D = PID_3D(Kx=(2,0.1,1), Ky=(2,0.1,1), Kz=(2,0.1,1), cible=(8,0,0))
pid_C = PID_2D(Kx=(1,0,0), Ky=(1,0,0), cible=(320,240))

#Variables for store trajectory
drone_positions = []
tank_positions = []
target_positions = []
rect_position = []

#Initialization of the controlled and simulation variables
d_vel = [0,0,0]
c_posi = [1.6,0]
drone.resetSimulation()

#Code required for video capture
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('resultmatch.avi', fourcc, 20.0, (640,480))

#Variables that will simulate the operating mode of the drone
controlled = False
conection = True

#Code needed to save information in txt form
with open('label.txt', 'w') as f:

    #Control Loop
    while (tank.followTankTraj(drone.getSimulationTime())):

            #Getting Image from camera
            image = drone.getCameraImage()
            out.write(image)

            #Getting real Data from Simulation
            drone_positions += [drone.getDronePosition()]
            drone_ori = drone.getDroneOrientation()
            tank_positions += [tank.getTankPosition()]
            tank_ori = tank.getTankOrientation()

            #Applying recognition algorithm to locate the tank in the image
            x, y, rect, w, h = findTank(image)

            #Using telemeter to mesure de real distance between drone and tank
            r_distance = getDistance(drone_positions[-1], tank_positions[-1])

            #Using vision to mesure de distance between drone and tank
            t_distance = 0

            #Using vision to estimate the tank orientation
            t_orientation = 0


            #Calculate theta e sigma
            t_theta = 0
            t_sigma = 0

            r_theta = math.atan2(math.sqrt((drone_positions[-1][0] - tank_positions[-1][0])**2 + (drone_positions[-1][1] - tank_positions[-1][1])**2) , math.sqrt((drone_positions[-1][2] - tank_positions[-1][2])**2))
            r_sigma = math.atan2(drone_positions[-1][0] - tank_positions[-1][0], drone_positions[-1][1] - tank_positions[-1][1])

        


            #We will simulate a loss of connection 10 seconds after takeoff
            if(drone.getSimulationTime() > 10):
                conection = False
                #f.write(str(x) + "," + str(y) + "," + str(c_posi[0]) + "," + str(c_posi[1]) +  "," + str(drone_ori[2]) +  "," + str(r_theta) +  "," + str(r_sigma) +"\n")


    
            #If the drone is not in manual mode then the control is done by the system
            if(not controlled):
                
                #If the drone has a stable connection to the tank we will use the gps data to perform the control
                if(conection):
                    #Setting Drone Position to Control
                    pid_D.set_position(drone_positions[-1])

                    #Calculate Target position
                    target_positions += [drone.getTargetPosition(tank_positions[-1],tank_ori,0)]
                    pid_D.set_target(target_positions[-1])

                    #Camera will have a fixed position looking down
                    c_posi = [1.6,0]
                

                #If the drone loses control then we will perform a vision-based navigation
                else:

                    #Setting Drone Position to Control (Used as a reference only)
                    pid_D.set_position(drone_positions[-1])

                    #Calculate camera control commands
                    c_posi[0] += (y - 240) * 0.0005
                    c_posi[1] -= (x - 320) * 0.0005
                    c_posi[0] = max(c_posi[0], 1)
                    c_posi[0] = min(c_posi[0], 2)


                        
                    #Calculate Target position
                    t_z = drone_positions[-1][2] - (r_distance * math.cos(r_theta))
                    t_x = drone_positions[-1][0] - (r_distance * math.sin(r_theta) *  math.sin(r_sigma))
                    t_y = drone_positions[-1][1] - (r_distance * math.sin(r_theta) *  math.cos(r_sigma))


                    target_positions += [drone.getTargetPosition([t_x, t_y, t_z],tank_ori,0)]
                    pid_D.set_target(target_positions[-1])




            #Calculate PID Commands for the drone
            pid_D.correction()
            d_vel[0] = pid_D.vx
            d_vel[1] = pid_D.vy
            d_vel[2] = pid_D.vz 


            #Send commands to Drone
            drone.setDroneSpeed(d_vel,[0,0,0])
            drone.setCameraOrientation(c_posi[0],c_posi[1])
            
out.release()
cv2.destroyAllWindows()

