import vrep
import time
import numpy as np

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Get "handle" to ALL joints of robot
# Get "handle" to the first joint of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

# Get "handle" to the second joint of robot
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

# Get "handle" to the third joint of robot
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

# Get "handle" to the fourth joint of robot
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

# Get "handle" to the fifth joint of robot
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

# Get "handle" to the sixth joint of robot
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')


# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# Run through all the joints from lower limit to upper limit
# Get the current value of the first joint variable
result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
#print('current value of first joint variable: theta = {:f}'.format(theta1))
# Set the desired value of the first joint variable from one limit to other and back
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, 0, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, 2*np.pi, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds

# Get the current value of the second joint variable
result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
#print('current value of second joint variable: theta = {:f}'.format(theta2))
# Set the desired value of the second joint variable from one limit to other and back
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, 1.2, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, -1.2, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta2, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds



# Get the current value of the third joint variable
result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
#print('current value of third joint variable: theta = {:f}'.format(theta2))
# Set the desired value of the third joint variable from one limit to other
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, 2.6, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, -2.6, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta3, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds

# Get the current value of the fourth joint variable
result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
# Set the desired value of the fourth joint variable from one limit to other
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, 0, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, 2*np.pi, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta4, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds

# Get the current value of the fifth joint variable
result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
# Set the desired value of the fifth joint variable from one limit to other
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, 0, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, 2*np.pi, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta5, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds

# Get the current value of the sixth joint variable
result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
# Set the desired value of the sixth joint variable from one limit to other
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, 0, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, 2*np.pi, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta6, vrep.simx_opmode_oneshot)
time.sleep(2) # Wait two seconds

# Get the current value of the first joint variable
#result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
#if result != vrep.simx_return_ok:
#    raise Exception('could not get first joint variable')
#print('current value of first joint variable: theta = {:f}'.format(theta))




# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
