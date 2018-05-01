#
# Project_Assignment4.py
#
# Created by Felipe F. and Di Z.
# 3/21/18
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

(theta) = myfct.inverse_kinematic(position_vector, euler_angle_vector,0)

clientID = myfct.start_simulation()
robot0_handles = myfct.get_robot_handles(clientID, 0)
robot1_handles = myfct.get_robot_handles(clientID, 1)
#collision_handles = myfct.get_collision_handles(clientID)
ref_handles = myfct.get_reference_object_handles(clientID)

myfct.move_robot(clientID, theta, 0,robot0_handles,ref_handles)

myfct.move_robot(clientID, theta, 1,robot1_handles,ref_handles)

myfct.end_simulation(clientID)
