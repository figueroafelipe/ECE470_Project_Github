#
# Project_Assignment4.py
#
# Created by Felipe F. and Di Z.
# 4/1/18
#
# To show inverse kinematics for robot in V_Rep Simulator


import vrep
import math
import time
import numpy as np
import scipy as sp
from scipy import linalg
import os.path
#import sys
#sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import ECE470_Project_Functions as myfct

(position_vector, euler_angle_vector) = myfct.get_user_position_and_orientation()

(theta) = myfct.inverse_kinematic(position_vector, euler_angle_vector)

clientID = myfct.start_simulation()

myfct.move_robot(clientID, theta, 0)

#myfct.move_robot(clientID, theta, 1)

robo0_collision_state = myfct.inCollision(clientID,0)
if robo0_collision_state == 0:
    print('UR3 is not in collision.')
else:
    print('UR3 is in collision!')

#robo1_collision_state = myfct.inCollision(clientID,1)
#if robo1_collision_state == 0:
#    print('UR3_0 is not in collision.')
#else:
#    print('UR3_0 is in collision!')


myfct.end_simulation(clientID)
