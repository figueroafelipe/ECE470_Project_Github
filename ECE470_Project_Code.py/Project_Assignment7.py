#
# Project_Assignment4.py
#
# Created by Felipe F. and Di Z.
# 4/10/18
#
# To show a closed chain robot in V_Rep Simulator



import numpy as np
import ECE470_Project_Functions as myfct
import vrep
import ece470_lib as ece470
import time

S0 = myfct.S_robot0
M0 = myfct.M_robot0
S1 = myfct.S_robot1
M1 = myfct.M_robot1
L = 0.1

robo_0_cup = "BaxterVacuumCup#_active"
robo_1_cup = "BaxterVacuumCup#0_active"

#Deactivate:
#vrep.simxSetIntegerSignal(clientID, leftcup, 0, vrep.simx_opmode_oneshot)

clientID = myfct.start_simulation()
robot0_handles = myfct.get_robot_handles(clientID, 0)
robot1_handles = myfct.get_robot_handles(clientID, 1)
#collision_handles = myfct.get_collision_handles(clientID)
ref_handles = myfct.get_reference_object_handles(clientID)
result, obj_handle = vrep.simxGetObjectHandle(clientID,'Cylinder',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for Object to pick up')
world_handle = ref_handles[1]
pos_test = np.array([-0.3,0,0.6])
pos_test.shape = (3,1)
euler_test = np.array([3.14,3.14,3.14])
euler_test.shape = (3,1)
theta_test = myfct.inverse_kinematic(pos_test,euler_test,1)
myfct.move_robot(clientID, theta_test, 1, robot0_handles, ref_handles)





#Get object Pose
obj_T, euler = myfct.get_object_T(clientID,obj_handle, world_handle)

robo_Ts = myfct.get_robo_T_from_obj_T(obj_T)
#print(robo_Ts[0])


#theta0 = ece470.findIK(robo_Ts[0],S0,M0) #NEED M's and S's
theta0 = myfct.inverse_kinematic(robo_Ts[0][0:3,3],myfct.rotationMatrixToEulerAngles(robo_Ts[0][0:3,0:3]),0)
myfct.move_robot(clientID, theta0, 0, robot0_handles, ref_handles)
#Activation:
time.sleep(2)
vrep.simxSetIntegerSignal(clientID, robo_0_cup, 1, vrep.simx_opmode_oneshot)
time.sleep(2)
euler = np.array([3.14,3.14,3.14])
euler.shape = (3,1)
position = np.array([-0.3,0,0.6])
position.shape = (3,1)
print(euler)
theta_start = myfct.inverse_kinematic(position,euler,0)
#theta_start = np.array([0,0,0,0,0,0]) #np.zeros((6,1))
#theta_start.shape = (6,1)
myfct.move_robot(clientID, theta_start, 0, robot0_handles, ref_handles) #Back to zero position

#Get object Pose
#obj_T, euler = myfct.get_object_T(clientID,obj_handle, world_handle)
obj_T = np.array([[1,0,0,position[0]], [0,1,0,position[1]],[0,0,1,position[2]], [0,0,0,1] ]).dot(np.array([[0, 0, -1, -L/2],[0, 1, 0, 0],[1,0,0,0],[0,0,0,1]]))
robo_Ts = myfct.get_robo_T_from_obj_T(obj_T)
print(robo_Ts)
#euler = np.array(euler)
position2 = robo_Ts[1][0:3,3]
print(position2)
#euler.shape = (3,1)
#print(euler)
#myfct.rotationMatrixToEulerAngles(robo_Ts[1][0:3,0:3])
#Get angles for robot 2 for where the object is
#theta1 = myfct.inverse_kinematic(robo_Ts[1][0:3,3],euler,1)
theta1 = myfct.inverse_kinematic(position2,euler,1)
#theta1 = ece470.findIK(robo_Ts[1], S1, M1) #NEED M's and S's
myfct.move_robot(clientID, theta1, 1, robot0_handles, ref_handles)
#Activation:
vrep.simxSetIntegerSignal(clientID, robo_1_cup, 1, vrep.simx_opmode_oneshot)


#GOAL
euler = np.array([np.pi,np.pi, np.pi])
position = np.array([0.5,0.5, 0.5])
R = myfct.euler_angles_to_rotation_matrix(euler)
T_goal = myfct.toPose(R,position)

#theta_goal = ece470.findIK(T_goal, S0, M0) #NEED M's and S's
theta_goal = myfct.inverse_kinematic(position,euler,0)


path_thetas_robo_0 = myfct.closed_chain_inerpolate(np.zeros(6,1),theta_goal)
path_thetas_robo_1 = np.zeros(path_thetas_robo_0.shape[0],path_thetas_robo_0.shape[1])
path_thetas_robo_1[:,1] = theta1

for i in range(1,path_thetas_robo_0.shape[1]):
    robo_0_T = myfct.forward_kinematics(path_thetas_robo_0[:,i], 0)
    #robo_0_T = ece470.evalT(S0,path_thetas_robo_0[:,i],M0 ) #NEED M's and S's
    robo_1_T = robo_0_T.dot(np.array([[1, 0, 0, -L], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
    path_thetas_robo_1[:, i] = myfct.inverse_kinematic(robo_1_T[0:3,3],myfct.rotationMatrixToEulerAngles(robo_1_T[0:3,0:3]), 1)
    #path_thetas_robo_1[:,i] = ece470.findIK(robo_1_T, S1, M1, path_thetas_robo_1[:,i-1]) #NEED M's and S's

for i in range(0,path_thetas_robo_0.shape[1]):
    myfct.pause_simulation(clientID)
    myfct.move_robot(clientID, path_thetas_robo_0[:,i], 0, robot0_handles, ref_handles)
    myfct.move_robot(clientID, path_thetas_robo_1[:,i], 1, robot0_handles, ref_handles)
    myfct.restart_simulation(clientID)
    # Wait two seconds
    time.sleep(2)



#(goal_position_vector, goal_euler_angle_vector) = myfct.get_user_position_and_orientation()



#theta0 = myfct.closed_chain_inverse_kinematic(goal_position_vector, goal_euler_angle_vector, 0)
#theta1 = myfct.closed_chain_slave_theta(theta0)

#myfct.move_robot(clientID, theta0, 0, robot0_handles, ref_handles)
#myfct.move_robot(clientID, theta1, 1, robot1_handles, ref_handles)
