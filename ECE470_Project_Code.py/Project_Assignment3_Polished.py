#
# Project_Assignment3.py
#
# Created by Felipe F. and Di Z.
# 3/12/18
#
# Poslished: 3/21/18
#
# To show forward kinematics for robot in V_Rep Simulator


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

(theta) = myfct.get_user_angles()

#(T_1in0, R_1in0, eulerAngles, p_1in0) = myfct.forward_kinematics(theta)

clientID = myfct.start_simulation()

myfct.move_robot(clientID, theta,0)
myfct.move_robot(clientID, theta,1)

myfct.end_simulation(clientID)
