#
# Project_Assignment4.py
#
# Created by Felipe F. and Di Z.
# 4/10/18
#
# To show path planning for robot in V_Rep Simulator


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

#values=np.array([[-0.2,-0.1,0.42,np.pi,np.pi,np.pi],[-0.2,-0.1,0.44,np.pi,np.pi,np.pi],[-0.2,-0.1,0.46,np.pi,np.pi,np.pi],[-0.2,-0.1,0.48,np.pi,np.pi,np.pi],[-0.2,-0.1,0.5,np.pi,np.pi,np.pi],[-0.2,-0.1,0.52,np.pi,np.pi,np.pi], [-0.2, -0.1, 0.54,np.pi,np.pi,np.pi], [-0.2,-0.1,0.56,np.pi,np.pi,np.pi], [-0.2,-0.1,0.58,np.pi,np.pi,np.pi],[-0.2,-0.1,0.6,np.pi,np.pi,np.pi], [-0.2,-0.1,0.1,np.pi,np.pi,np.pi], [-0.2,-0.1,0.12,np.pi,np.pi,np.pi],[-0.2,-0.1,0.14,np.pi,np.pi,np.pi],[-0.2,-0.1,0.16,np.pi,np.pi,np.pi],[-0.2,-0.1,0.18,np.pi,np.pi,np.pi],[-0.2,-0.1,0.2,np.pi,np.pi,np.pi],[-0.2,-0.1,0.21,np.pi,np.pi,np.pi],[-0.2,-0.1,0.22,np.pi,np.pi,np.pi],[-0.2,-0.1,0.23,np.pi,np.pi,np.pi],[-0.2,-0.1,0.24,np.pi,np.pi,np.pi], [-0.4,0,0.64,np.pi,np.pi,np.pi],[-0.4,0,0.62,np.pi,np.pi,np.pi],[-0.4,0,0.6,np.pi,np.pi,np.pi],[-0.4,0,0.58,np.pi,np.pi,np.pi],[-0.4,0,0.56,np.pi,np.pi,np.pi],[-0.2,0.2,0.38,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.4,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.42,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.44,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.46,np.pi/2,np.pi/2,np.pi/2]])
#print(values)
#theta = np.zeros((6,len(values)))
#theta.shape = (6,len(values))

#(start_position_vector, start_euler_angle_vector) = myfct.get_user_position_and_orientation()
#(goal_position_vector, goal_euler_angle_vector) = myfct.get_user_position_and_orientation()

#position_array = np.array([values[i,0], values[i,1], values[i,2]])
#position_array.shape = (3, 1)
#position_vector = np.matrix(position_array)


#euler_angle_array = np.array([values[i,3], values[i,4], values[i,5]])
#euler_angle_array.shape = (3, 1)
#euler_angle_vector = np.matrix(euler_angle_array)

#theta_start = myfct.inverse_kinematic(start_position_vector, start_euler_angle_vector)
#theta_goal = myfct.inverse_kinematic(goal_position_vector, goal_euler_angle_vector)
#theta[:, i] = theta_i.reshape(1,6)





#theta_start = np.array([[ 1.53729721],[ 0.53527977],[-1.38640437],[-2.29201791],[ 1.53903389],[ 3.13940153]])
#theta_start = np.array([[ 1.53729721, 0.53527977, -1.38640437, -2.29201791, 1.53903389, 3.13940153]])
theta_start = np.array([[-0.21158543],[-0.55443982],[-0.19809841],[ 0.73747074],[ 0.21035263],[ 0.01840278]])
theta_start.shape= (6,1)
print('Start Theta:')
print(theta_start)
#theta_goal = np.array([[ 1.52733207],[ 0.71842342],[-1.90200317],[ 1.18206001],[-1.52906009],[-0.00279136]])
#theta_goal = np.array( [[ 0.21546055],[ 1.30721092],[-1.05589541],[-0.27952938],[-0.21716709],[ 0.03084803]])
theta_goal = np.array([[ 3.66849413e-01],[ 2.14416151e-02],[ 1.91755391e+00],[-1.93681197e+00],[-3.68052706e-01],[-3.68622596e-04]])
theta_goal.shape=(6,1)
print('\nGoal Theta:')
print(theta_goal)
print()




clientID = myfct.start_simulation()
robot0_handles = myfct.get_robot_handles(clientID, 0)
#robot1_handles = myfct.get_robot_handles(clientID, 1)

collision_handles = myfct.get_collision_handles(clientID)
ref_handles = myfct.get_reference_object_handles(clientID)
#add in future: get current state (should be zero angle position) and make a path that takes you to the start theta, then continue to end theta)
#myfct.pause_simulation(clientID)
print('Starting path planner')
path = myfct.path_planner(clientID,theta_start, theta_goal,robot0_handles,collision_handles,ref_handles)
l = path.shape[1]
#print(l)
#print(path)
#print(type(path))
path_matrix = np.matrix(path)
path_matrix.shape = (6,l)
#print(type(path_matrix))
#myfct.restart_simulation(clientID)

print('Starting simulation with planned path')
for i in range(0,l):
    myfct.move_robot(clientID, path_matrix[:,i], 0, robot0_handles, ref_handles)

myfct.end_simulation(clientID)
