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

#(position_vector, euler_angle_vector) = myfct.get_user_position_and_orientation()

#T_goal = np.array([[1,0,0,-0.1],[0,1,0,0.3],[0,0,1,0.4],[0,0,0,1] ])
#euler = myfct.rotationMatrixToEulerAngles(T_goal[0:3,0:3])
#euler.shape = (3,1)
#position_vector = T_goal[0:3,3]
#position_vector.shape = (3,1)
theta = np.zeros((6,10))
theta.shape = (6,10)
euler = np.array([3.14,3.14,3.14])
euler.shape = (3,1)
position = np.array([-0.2, 0.1,0.6])
position.shape = (3,1)
theta0 = myfct.inverse_kinematic(position, euler,0)

position = np.array([-0.2, -0.1,0.6])
position.shape = (3,1)
theta1 = myfct.inverse_kinematic(position, euler,0)

position = np.array([0.1, -0.1,0.6])
position.shape = (3,1)
theta2 = myfct.inverse_kinematic(position, euler,0)

position = np.array([0.1, 0.1,0.6])
position.shape = (3,1)
theta3 = myfct.inverse_kinematic(position, euler,0)

position = np.array([-0.2, -0.1,0.2])
position.shape = (3,1)
theta4 = myfct.inverse_kinematic(position, euler,0)

position = np.array([-0.2, -0.15,0.2])
position.shape = (3,1)
theta5 = myfct.inverse_kinematic(position, euler,0)

position = np.array([-0.2, 0.2,0.2])
position.shape = (3,1)
theta6 = myfct.inverse_kinematic(position, euler,0)

#position = np.array([-0.2, 0.05, 0.2])
#position.shape = (3,1)
#theta7 = myfct.inverse_kinematic(position, euler,0)

position = np.array([-0.2, 0.1,0.2])
position.shape = (3,1)
theta8 = myfct.inverse_kinematic(position, euler,0)

position = np.array([-0.1, 0.1,0.2])
position.shape = (3,1)
theta9 = myfct.inverse_kinematic(position, euler,0)

position = np.array([0.15, 0.1,0.2])
position.shape = (3,1)
theta10 = myfct.inverse_kinematic(position, euler,0)

position = np.array([0.1, 0.1,0.2])
position.shape = (3,1)
theta11 = myfct.inverse_kinematic(position, euler,0)

#T_rob2 = np.array([[1,0,0,0],[0,1,0,0.6],[0,0,1,0],[0,0,0,1]]).dot(T_goal)
#euler2 = myfct.rotationMatrixToEulerAngles(T_rob2[0:3,0:3])
#euler2.shape = (3,1)
#position_vector2 = T_rob2[0:3,3]
#position_vector2.shape = (3,1)
#(theta2) = myfct.inverse_kinematic(position_vector, euler,1)

clientID = myfct.start_simulation()
robot0_handles = myfct.get_robot_handles(clientID, 0)
robot1_handles = myfct.get_robot_handles(clientID, 1)
#collision_handles = myfct.get_collision_handles(clientID)
ref_handles = myfct.get_reference_object_handles(clientID)


myfct.move_robot(clientID, theta0, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta0, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta1, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta1, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta2, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta2, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta3, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta3, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta4, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta4, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta5, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta5, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta6, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta6, 1,robot1_handles,ref_handles)

#myfct.move_robot(clientID, theta7, 0,robot0_handles,ref_handles)
#myfct.move_robot(clientID, theta7, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta8, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta8, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta9, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta9, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta10, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta10, 1,robot1_handles,ref_handles)

myfct.move_robot(clientID, theta11, 0,robot0_handles,ref_handles)
myfct.move_robot(clientID, theta11, 1,robot1_handles,ref_handles)

myfct.end_simulation(clientID)
