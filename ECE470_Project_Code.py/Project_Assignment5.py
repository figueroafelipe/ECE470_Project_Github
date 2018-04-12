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

values=np.array([[-0.2,-0.1,0.42,np.pi,np.pi,np.pi],[-0.2,-0.1,0.44,np.pi,np.pi,np.pi],[-0.2,-0.1,0.46,np.pi,np.pi,np.pi],[-0.2,-0.1,0.48,np.pi,np.pi,np.pi],[-0.2,-0.1,0.5,np.pi,np.pi,np.pi],[-0.2,-0.1,0.52,np.pi,np.pi,np.pi], [-0.2, -0.1, 0.54,np.pi,np.pi,np.pi], [-0.2,-0.1,0.56,np.pi,np.pi,np.pi], [-0.2,-0.1,0.58,np.pi,np.pi,np.pi],[-0.2,-0.1,0.6,np.pi,np.pi,np.pi], [-0.2,-0.1,0.1,np.pi,np.pi,np.pi], [-0.2,-0.1,0.12,np.pi,np.pi,np.pi],[-0.2,-0.1,0.14,np.pi,np.pi,np.pi],[-0.2,-0.1,0.16,np.pi,np.pi,np.pi],[-0.2,-0.1,0.18,np.pi,np.pi,np.pi],[-0.2,-0.1,0.2,np.pi,np.pi,np.pi],[-0.2,-0.1,0.21,np.pi,np.pi,np.pi],[-0.2,-0.1,0.22,np.pi,np.pi,np.pi],[-0.2,-0.1,0.23,np.pi,np.pi,np.pi],[-0.2,-0.1,0.24,np.pi,np.pi,np.pi], [-0.4,0,0.64,np.pi,np.pi,np.pi],[-0.4,0,0.62,np.pi,np.pi,np.pi],[-0.4,0,0.6,np.pi,np.pi,np.pi],[-0.4,0,0.58,np.pi,np.pi,np.pi],[-0.4,0,0.56,np.pi,np.pi,np.pi],[-0.2,0.2,0.38,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.4,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.42,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.44,np.pi/2,np.pi/2,np.pi/2],[-0.2,0.2,0.46,np.pi/2,np.pi/2,np.pi/2]])
#print(values)
theta = np.zeros((6,len(values)))
theta.shape = (6,len(values))

#(position_vector, euler_angle_vector) = myfct.get_user_position_and_orientation()

theta = np.array([[ 1.54835912e+00,  1.54579162e+00,  1.54310666e+00,  1.54063279e+00,
   1.53775220e+00,  1.53588179e+00,  1.53353313e+00,  1.53039667e+00,
   1.52720530e+00,  1.52480772e+00,  1.65243576e+00,  1.57278199e+00,
   1.57374620e+00,  1.57542267e+00,  1.58415466e+00,  1.56691745e+00,
   1.57651118e+00,  1.60671777e+00,  1.56538588e+00,  2.19618583e+00,
   7.69845992e-01, -7.68358792e-01,  7.66465123e-01, -7.68541545e-01,
  -7.70847266e-01,  3.82849627e-01,  3.95026915e-01,  3.94939669e-01,
   3.98000076e-01,  4.03927597e-01],
 [ 8.13920892e-01, -1.14012125e+00, -1.09122424e+00,  7.53181341e-01,
  -9.89644556e-01, -9.35995436e-01, -8.80206899e-01, -8.21728405e-01,
   5.48039892e-01,  4.91043289e-01, -3.79091420e-01, -1.80053189e+00,
  -3.85641106e+00, -3.73734238e+00, -3.51887446e+00, -3.16768558e+00,
  -2.97263250e+00, -2.64938335e+00,  8.26924540e-01, -8.27720271e-01,
  -9.94670525e-02,  1.71101077e-01, -2.29509486e-01, -9.21175808e-01,
   3.14931072e-01,  5.34388746e-01,  4.37608467e-01,  3.35548463e-01,
   2.51145260e-01,  1.82576363e-01],
 [-2.30545903e+00,  2.20733357e+00,  2.10801937e+00, -2.00653559e+00,
   1.90003899e+00,  1.79016495e+00,  1.67514927e+00,  1.55374703e+00,
  -1.42480471e+00, -1.28481831e+00,  3.19411997e+00,  3.65504727e+00,
   2.72238536e+00,  2.81327929e+00,  2.90826242e+00,  2.98269125e+00,
   3.01477439e+00,  3.05832940e+00, -2.41998587e+00,  2.37299782e+00,
   8.18046158e-01, -1.00320500e+00,  1.16341848e+00,  1.29552997e+00,
  -1.43552202e+00,  2.35431489e+00,  2.32696603e+00,  2.31065961e+00,
   2.28034350e+00,  2.23718323e+00],
 [ 1.49145685e+00, -1.06726235e+00, -1.01681251e+00,  1.25336409e+00,
  -9.10352447e-01, -8.54103537e-01, -7.94851381e-01, -7.31899429e-01,
   8.76906117e-01,  7.93939402e-01,  3.26637023e-01, -1.85464741e+00,
   1.13390101e+00,  9.23912277e-01,  6.10447371e-01,  1.84983235e-01,
  -4.22525899e-02, -4.09400088e-01, -1.54871719e+00,  1.59481515e+00,
  -7.23658585e-01,  8.37247384e-01, -9.39122542e-01, -3.69343242e-01,
   1.12539682e+00, -1.31857150e+00, -1.19367462e+00, -1.07527795e+00,
  -9.60226470e-01, -8.47908125e-01],
 [-1.54848206e+00, -1.54591684e+00, -1.54323665e+00, -1.54076803e+00,
  -1.53788912e+00, -1.53602053e+00, -1.53367361e+00, -1.53053916e+00,
  -1.52735016e+00, -1.52495327e+00,  1.65261410e+00, -1.57287207e+00,
  -1.57383193e+00, -1.57549764e+00, -1.58421850e+00, -1.56705324e+00,
  -1.57662639e+00, -1.60672635e+00,  1.56550227e+00,  2.19606442e+00,
  -7.70237089e-01,  7.68768535e-01, -7.66891379e-01,  7.68984903e-01,
   7.71264691e-01,  1.57174155e+00,  1.57114443e+00,  1.57110527e+00,
   1.57077684e+00,  1.57021774e+00],
 [-3.85141455e-03, -3.98694737e-03, -4.14189996e-03, -4.30107290e-03,
  -4.43954647e-03, -4.57028805e-03, -4.70335657e-03, -4.84410759e-03,
  -4.98718056e-03, -5.10735255e-03, -3.14295026e+00, -1.18014026e-03,
  -1.26355675e-03, -1.38512228e-03, -1.39306349e-03, -1.88290187e-03,
  -2.21176159e-03, -2.28465488e-03,  3.13934577e+00,  3.13871347e+00,
   6.55990811e-03, -6.65695563e-03,  6.73581999e-03, -6.60804483e-03,
  -6.18194759e-03, -1.95375309e+00, -1.96568422e+00, -1.96558878e+00,
  -1.96858938e+00, -1.97441414e+00]])

#for i in range(0,len(values)):
#    position_array = np.array([values[i,0], values[i,1], values[i,2]])
#    position_array.shape = (3, 1)
#    position_vector = np.matrix(position_array)

#    euler_angle_array = np.array([values[i,3], values[i,4], values[i,5]])
#    euler_angle_array.shape = (3, 1)
#    euler_angle_vector = np.matrix(euler_angle_array)

#    theta_i = myfct.inverse_kinematic(position_vector, euler_angle_vector)
#    theta[:, i] = theta_i.reshape(1,6)

print('The thirty path angles are:')
print(theta)

for i in range(0,len(theta[0])):
    clientID = myfct.start_simulation()
    theta_i = theta[:,i]
    myfct.move_robot(clientID, theta_i.reshape(6,1), 0)
    myfct.move_robot(clientID, theta_i.reshape(6,1), 1)
    print('Path # ')
    print(i+1)
    robo0_collision_state = myfct.inCollision(clientID)
    myfct.end_simulation(clientID)

