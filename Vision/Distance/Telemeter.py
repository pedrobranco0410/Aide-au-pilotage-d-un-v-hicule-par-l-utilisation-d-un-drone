
import math
import random

def getDistance(drone_position, tank_position):

    r_distance = (drone_position[0] - tank_position[0])**2 + (drone_position[1] - tank_position[1])**2 +(drone_position[2] - tank_position[2])**2 
    r_distance = math.sqrt(r_distance) + random.uniform(0,2)

    return r_distance