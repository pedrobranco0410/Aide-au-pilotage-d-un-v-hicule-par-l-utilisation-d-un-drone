"""
Code responsible for testing the performance of the speed PID controller developed by the drone. 
This code only tests the controller with reliable data and the result is not influenced by any 
other vision processing code etc. The drone will be subjected to different trajectories and we 
will calculate the accumulated errors to issue a final evaluation of the code's performance.
"""

import sys
sys.path.append('../')
from Simulation.Drone import Drone
import rospy
import roslibpy

#Starting the client that will be responsible for sending and receiving the messages from the ROS nodes.
client = roslibpy.Ros(host="localhost", port=9090)
client.run()

#Initializing the drone
drone = Drone(client)

#Initializing the PID controller
#TO DO