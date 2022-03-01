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
from Vision.Recognition.TankRecognition4 import findTank
from PID_Drone import PID_3D
from PID_Camera import PID_2D
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
pid_D = PID_3D(Kx=(2,0.1,1), Ky=(2,0.1,1), Kz=(2,0.1,1), cible=(5,0,0))
pid_C = PID_2D(Kx=(2,0.1,1), Ky=(0.1,0,0), cible=(320,240))


#Variables for store traj
drone_positions = []
tank_positions = []
target_positions = []
rect_position = []

d_vel = [0,0,0]
c_posi = [1.6,0]
drone.resetSimulation()

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('resultmatch.avi', fourcc, 20.0, (640,480))

k = 0

with open('label.txt', 'w') as f:
    #Control Loop
    while (tank.followTankTraj(drone.getSimulationTime())):

        #Getting Image
        image = drone.getCameraImage()
        
        #Getting Positions
        x, y, rect, w, h = findTank(image)
        rect_position += [[x,y]]
        pid_C.set_position(rect_position[-1])

        drone_positions += [drone.getDronePosition()]
        drone_ori = drone.getDroneOrientation()
        pid_D.set_position(drone_positions[-1])

        tank_positions += [tank.getTankPosition()]
        tank_ori = tank.getTankOrientation()

        if(drone.getSimulationTime() > 5):
            out.write(image)
            f.write(str(x) + "," + str(y) + "," + str(w) + "," + str(h)+ "\n")
            #f.write(str(drone_positions[-1]) + " " + str(drone_ori) + " ")
            #f.write(str(tank_positions[-1]) + " " + str(tank_ori) + " " )
            #f.write(str(c_posi) + '\n')
            #cv2.imwrite("../Database/image"+ str(k)+  ".png", rect)
            k+= 1 
            '''cv2.imshow("Rect", rect)
            cv2.waitKey(1)'''

        #Calculate Target position
        target_positions += [drone.getTargetPosition(tank_positions[-1],tank_ori,0)]
        pid_D.set_target(target_positions[-1])
            
        #Calculate PID Commands for the drone
        pid_D.correction()
        d_vel[0] = pid_D.vx
        d_vel[1] = pid_D.vy
        d_vel[2] = pid_D.vz 

        #Calculate PID Commands for the camera
        pid_C.correction()

        if(drone.getSimulationTime() > 5):
            c_posi[0] += (y - 240) * 0.0005
            c_posi[1] -= (x - 320) * 0.0005

        c_posi[0] = max(c_posi[0], 1)
        c_posi[0] = min(c_posi[0], 2)

        #Send commands to Drone
        drone.setDroneSpeed(d_vel,[0,0,0])
        drone.setCameraOrientation(c_posi[0],c_posi[1])

    out.release()
    cv2.destroyAllWindows()

#pid.print_PID_2D(drone_positions, target_positions, tank_positions)