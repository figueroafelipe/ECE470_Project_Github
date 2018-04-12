#
# ECE470_Project_Functions
#
# Created by Felipe F. and Di Z.
# 3/21/18
#
# To store all necessary program functions

import vrep
import math
import time
import numpy as np
import scipy as sp
from scipy import linalg
from collections import namedtuple

S = np.matrix([[0, -1, -1, -1, 0, -1], [0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0],
                   [0, -0.152, -0.396, -0.609, 0.11, -0.692], [0, 0, 0, 0, 0, 0]])

S1_skew = np.matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
S2_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.152], [0, -1, 0, 0], [0, 0, 0, 0]])
S3_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.396], [0, -1, 0, 0], [0, 0, 0, 0]])
S4_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.609], [0, -1, 0, 0], [0, 0, 0, 0]])
S5_skew = np.matrix([[0, -1, 0, 0], [1, 0, 0, 0.11], [0, 0, 0, 0], [0, 0, 0, 0]])
S6_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.692], [0, -1, 0, 0], [0, 0, 0, 0]])
M = np.matrix([[1, 0, 0, -0.342], [0, 1, 0, 0], [0, 0, 1, 0.692], [0, 0, 0, 1]])

n = 6

#Calculate the final pose of the robot given the angles (in a vector) of each joint
def forward_kinematics (theta):
    #S = np.matrix([[0, -1, -1, -1, 0, -1], [0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0],
    #               [0, -0.152, -0.396, -0.609, 0.11, -0.692], [0, 0, 0, 0, 0, 0]])

    #S1_skew = np.matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
    #S2_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.152], [0, -1, 0, 0], [0, 0, 0, 0]])
    #S3_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.396], [0, -1, 0, 0], [0, 0, 0, 0]])
    #S4_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.609], [0, -1, 0, 0], [0, 0, 0, 0]])
    #S5_skew = np.matrix([[0, -1, 0, 0], [1, 0, 0, 0.11], [0, 0, 0, 0], [0, 0, 0, 0]])
    #6_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.692], [0, -1, 0, 0], [0, 0, 0, 0]])
    #M = np.matrix([[1, 0, 0, -0.342], [0, 1, 0, 0], [0, 0, 1, 0.692], [0, 0, 0, 1]])

    a = np.dot(linalg.expm(S1_skew * theta[0,0]),linalg.expm(S2_skew * theta[1,0]))
    b = np.dot(a,linalg.expm(S3_skew * theta[2,0]))
    c = np.dot(b, linalg.expm(S4_skew * theta[3,0]))
    d = np.dot(c, linalg.expm(S5_skew * theta[4,0]))
    e = np.dot(d, linalg.expm(S6_skew * theta[5,0]))
    T_1in0 = np.dot(e,M)

    R_1in0 = np.array(T_1in0[0:3, 0:3])
    eulerAngles = rotationMatrixToEulerAngles(R_1in0)
    p_1in0 = np.array(T_1in0[0:3, 3])

    #print('\nPose of end-effector: ')
    #print(T_1in0, end='\n\n')
    #print('Euler Angles: ')
    #print(eulerAngles, end='\n\n')

    return (T_1in0, R_1in0, eulerAngles, p_1in0)

#Calculate the skew (3 by 3) of a 3 by 1 vector: skew = [[0, -V[3], V[2]], [V[3], 0, -V[1]], [-V[2], V[1], 0]]]
def skew3(M):
    M_skew = np.matrix([[0, -M[2,0], M[1,0]], [M[2,0], 0, -M[0,0]], [-M[1,0], M[0,0], 0]])
    return M_skew

#Calculate the skew (4 by 4) of a 6 by 1 vector: skew = [[0, -V[3], V[2], V[4]], [V[3], 0, -V[1], V[5]], [-V[2], V[1], 0, V[6]], [0,0,0,0]]]
def skew4(V):
    V_skew = np.matrix([[0, -V[2,0], V[1,0], V[3,0]], [V[2,0], 0, -V[0,0], V[4,0]], [-V[1,0], V[0,0], 0, V[5,0]], [0, 0, 0, 0]])

    return V_skew

#Calculates the adjoint of a matrix: adj = [[R, 0], [p_skew*R,R]]
def Adj (T):
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    p.shape = (3,1)
    p_skew = skew3(p)

    a = np.dot(p_skew,R)
    T_adj = np.matrix([[R[0,0], R[0,1], R[0,2], 0, 0, 0],[R[1,0], R[1,1], R[1,2], 0, 0, 0 ], [R[2,0], R[2,1], R[2,2], 0, 0, 0], [a[0,0], a[0,1], a[0,2], R[0,0], R[0,1], R[0,2]], [a[1,0], a[1,1], a[1,2], R[1,0], R[1,1], R[1,2]], [a[2,0], a[2,1], a[2,2], R[2,0], R[2,1], R[2,2]]])

    return T_adj

#Calculate the joint angles of the robot necessary to achieve a given pose. If it is not achievable then the function throws out an exception
def inverse_kinematic(position, euler_vector):

    R = euler_angles_to_rotation_matrix(euler_vector)

    T_2in0 = np.matrix([[R[0,0], R[0,1], R[0,2], position[0,0]], [R[1,0], R[1,1], R[1,2], position[1,0]], [R[2,0], R[2,1], R[2,2], position[2,0]], [0, 0, 0, 1]])

    print('\nDesired pose: ')
    print(T_2in0, end='\n\n')

    #S = np.matrix([[0, -1, -1, -1, 0, -1], [0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0],
    #               [0, -0.152, -0.396, -0.609, 0.11, -0.692], [0, 0, 0, 0, 0, 0]])

    #S1_skew = np.matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
    #S2_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.152], [0, -1, 0, 0], [0, 0, 0, 0]])
    #S3_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.396], [0, -1, 0, 0], [0, 0, 0, 0]])
    #S4_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.609], [0, -1, 0, 0], [0, 0, 0, 0]])
    #S5_skew = np.matrix([[0, -1, 0, 0], [1, 0, 0, 0.11], [0, 0, 0, 0], [0, 0, 0, 0]])
    #S6_skew = np.matrix([[0, 0, 0, 0], [0, 0, 1, -0.692], [0, -1, 0, 0], [0, 0, 0, 0]])
    #M = np.matrix([[1, 0, 0, -0.342], [0, 1, 0, 0], [0, 0, 1, 0.692], [0, 0, 0, 1]])

    #n = 6

    epsilon = 0.01

    theta = np.random.rand(n, 1)

    V_01in0 = 1

    t_start = time.process_time()
    count = 0
    while linalg.norm(V_01in0) >= epsilon:
        mu = 1e-1
        (T_1in0, R_1in0, eulerAngles, p_1in0) = forward_kinematics(theta)

        V_01in0_bracket = linalg.logm(np.dot(T_2in0, linalg.inv(T_1in0)))

        V_01in0 = np.matrix([V_01in0_bracket[2, 1], V_01in0_bracket[0, 2], V_01in0_bracket[1, 0], V_01in0_bracket[0, 3], V_01in0_bracket[1, 3], V_01in0_bracket[2, 3]])
        V_01in0.shape = (6,1)

        a = S[0:6, 0]
        b = np.dot(Adj(linalg.expm(S1_skew * theta[0,0])), S[0:6, 1])
        c = np.dot(Adj(np.dot(linalg.expm(S1_skew*theta[0,0]), linalg.expm(S2_skew*theta[1,0]))), S[0:6, 2])
        d = np.dot(Adj(np.dot(np.dot(linalg.expm(S1_skew*theta[0,0]),linalg.expm(S2_skew*theta[1,0])), linalg.expm(S3_skew*theta[2,0]))), S[0:6, 3])
        e = np.dot(Adj(np.dot(np.dot(np.dot(linalg.expm(S1_skew*theta[0,0]),linalg.expm(S2_skew*theta[1,0])), linalg.expm(S3_skew*theta[2,0])), linalg.expm(S4_skew*theta[3,0]))), S[0:6, 4])
        f = np.dot(Adj(np.dot(np.dot(np.dot(np.dot(linalg.expm(S1_skew*theta[0,0]),linalg.expm(S2_skew*theta[1,0])), linalg.expm(S3_skew*theta[2,0])), linalg.expm(S4_skew*theta[3,0])), linalg.expm(S5_skew*theta[4,0]))), S[0:6, 5])

        j_in0 = np.matrix([[a[0,0], b[0,0], c[0,0], d[0,0], e[0,0], f[0,0]], [a[1,0], b[1,0], c[1,0], d[1,0], e[1,0], f[1,0]], [a[2,0], b[2,0], c[2,0], d[2,0], e[2,0], f[2,0]], [a[3,0], b[3,0], c[3,0], d[3,0], e[3,0], f[3,0]], [a[4,0], b[4,0], c[4,0], d[4,0], e[4,0], f[4,0]], [a[5,0], b[5,0], c[5,0], d[5,0], e[5,0], f[5,0]]])

        thetadot = np.dot(np.dot(linalg.inv(np.dot(j_in0.transpose(), j_in0) + np.dot(mu,np.eye(6))), j_in0.transpose()), V_01in0)

        theta = theta + thetadot*1

        t_now = time.process_time()

        if t_now-t_start > 15:
            t_start = time.process_time()
            theta = np.random.rand(n, 1)
            count= count+1
            if count > 15:
                print('Stopping program because the input position and orientation is not reachable by the robot\n')
                raise Exception('Goal position is not reachable')

    print('\nFinal angles that achieve the desired pose: ')
    print(theta, end='\n\n')

    return (theta)

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# Calculates Rotation Matrix given euler angles.
def euler_angles_to_rotation_matrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

#Begins a V-Rep simulation and returns the clientID of the V-Rep connection
def start_simulation():
    # Close all open connections (just in case)
    vrep.simxFinish(-1)

    # Connect to V-REP (raise exception on failure)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')

    # Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    # Wait two seconds
    time.sleep(2)

    return clientID

def pause_simulation(clientID):
    result = vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not pause simulation!')

    # Wait two seconds
    time.sleep(2)


def restart_simulation(clientID):
    # Start simulation
    result = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    if result != vrep.simx_return_ok:
        raise Exception('could not restart simulation!')

    # Wait two seconds
    time.sleep(2)


def get_robot_handles (clientID, robo):
    if robo == 0:
        # Get "handle" to ALL joints of robot
        # Get "handle" to the first joint of robot
        result, robot0_joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for first joint')

        # Get "handle" to the second joint of robot
        result, robot0_joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for first joint')

        # Get "handle" to the third joint of robot
        result, robot0_joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for first joint')

        # Get "handle" to the fourth joint of robot
        result, robot0_joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for first joint')

        # Get "handle" to the fifth joint of robot
        result, robot0_joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for first joint')

        # Get "handle" to the sixth joint of robot
        result, robot0_joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for first joint')

        # Wait two seconds
        time.sleep(2)

        robo_handles = np.array([robot0_joint_one_handle, robot0_joint_two_handle, robot0_joint_three_handle, robot0_joint_four_handle, robot0_joint_five_handle, robot0_joint_six_handle])
        return robo_handles

    elif robo == 1:
        # Get "handle" to ALL joints of robot
        # Get "handle" to the first joint of robot
        result, robot1_joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1#0', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for first joint')

        # Get "handle" to the second joint of robot
        result, robot1_joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2#0', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for second joint')

        # Get "handle" to the third joint of robot
        result, robot1_joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3#0', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for third joint')

        # Get "handle" to the fourth joint of robot
        result, robot1_joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4#0', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for fourth joint')

        # Get "handle" to the fifth joint of robot
        result, robot1_joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5#0', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for fifth joint')

        # Get "handle" to the sixth joint of robot
        result, robot1_joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6#0', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get object handle for sixth joint')

        # Wait two seconds
        time.sleep(2)

        robo_handles = np.array([robot1_joint_one_handle, robot1_joint_two_handle, robot1_joint_three_handle, robot1_joint_four_handle,
             robot1_joint_five_handle, robot1_joint_six_handle])
        return robo_handles

    else:
        raise Exception('Wrong input, could not get handles')

def get_reference_object_handles(clientID):
    # Get "handle" to WorldReferenceFrame
    result, WorldReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'WorldReferenceFrame',
                                                                  vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for world reference frame')

    # Get "handle" to T_1in0_ReferenceFrame
    result, T_1in0_ReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'T_1in0_ReferenceFrame',
                                                                    vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for T_1in0 reference frame')

    # Get "handle" to UR3_0_ReferenceFrame
    result, UR3_0_ReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'UR3_0_ReferenceFrame',
                                                                   vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for UR3_0 reference frame')

    # Wait two seconds
    time.sleep(2)

    ref_object_handles = np.array([WorldReferenceFrame_handle, T_1in0_ReferenceFrame_handle, UR3_0_ReferenceFrame_handle])
    return ref_object_handles

#Moves the joint of the robot to the prescribed angle given
def move_robot (clientID, theta, robo, robot_handles,ref_object_handles):

    (T_1in0, R_1in0, end_eulerAngles, end_position) = forward_kinematics(theta)

    #robot_handles = get_robot_handles(clientID, robo)

    # Get "handle" to WorldReferenceFrame
    #result, WorldReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'T_1in0_ReferenceFrame',
    #                                                              vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for world reference frame')

    # Get "handle" to T_1in0_ReferenceFrame
    #result, T_1in0_ReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'T_1in0_ReferenceFrame',
    #                                                                vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for T_1in0 reference frame')

    # Get "handle" to UR3_0_ReferenceFrame
    #result, UR3_0_ReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'UR3_0_ReferenceFrame',
    #                                                                    vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for UR3_0 reference frame')

    # Wait two seconds
    #time.sleep(2)

    if robo == 0:
        # Set T_1in0_ReferenceFrame position and orientation
        vrep.simxSetObjectPosition(clientID, ref_object_handles[1], ref_object_handles[0], end_position,
                                   vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(clientID, ref_object_handles[1], ref_object_handles[0], end_eulerAngles,
                                      vrep.simx_opmode_oneshot)
    elif robo == 1:
        # Set T_1in0_ReferenceFrame position and orientation
        vrep.simxSetObjectPosition(clientID, ref_object_handles[1], ref_object_handles[2], end_position,
                                   vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(clientID, ref_object_handles[1], ref_object_handles[2],
                                      end_eulerAngles,
                                      vrep.simx_opmode_oneshot)
    else:
        raise Exception('Problem with robot number')

    #vrep.simxPauseCommunication(clientID, True)

    targetVelocity = 200
    # Position all the joint angles according to user input
    # Set the desired value of the first joint variable
    vrep.simxSetJointTargetVelocity(clientID, robot_handles[0], targetVelocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, robot_handles[0], theta[0], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the second joint variable
    vrep.simxSetJointTargetVelocity(clientID, robot_handles[1], targetVelocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, robot_handles[1], theta[1], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the third joint variable
    vrep.simxSetJointTargetVelocity(clientID, robot_handles[2], targetVelocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, robot_handles[2], theta[2], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the fourth joint variable
    vrep.simxSetJointTargetVelocity(clientID, robot_handles[3], targetVelocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, robot_handles[3], theta[3], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the fifth joint variable
    vrep.simxSetJointTargetVelocity(clientID, robot_handles[4], targetVelocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, robot_handles[4], theta[4], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the sixth joint variable
    vrep.simxSetJointTargetVelocity(clientID, robot_handles[5], targetVelocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, robot_handles[5], theta[5], vrep.simx_opmode_streaming)
    time.sleep(2)  # Wait two seconds

    #vrep.simxPauseCommunication(clientID, False)


#Moves the joint of the robot to the prescribed angle given without simulation running
def move_robot_without_simulation(clientID, theta, robo, robot_handles,ref_object_handles):

    (T_1in0, R_1in0, end_eulerAngles, end_position) = forward_kinematics(theta)

    #robot_handles = get_robot_handles(clientID, robo)

    # Get "handle" to WorldReferenceFrame
    #result, WorldReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'T_1in0_ReferenceFrame',
    #                                                              vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for world reference frame')

    # Get "handle" to T_1in0_ReferenceFrame
    #result, T_1in0_ReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'T_1in0_ReferenceFrame',
    #                                                                vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for T_1in0 reference frame')

    # Get "handle" to UR3_0_ReferenceFrame
    #result, UR3_0_ReferenceFrame_handle = vrep.simxGetObjectHandle(clientID, 'UR3_0_ReferenceFrame',
    #                                                                    vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for UR3_0 reference frame')

    # Wait two seconds
    #time.sleep(2)

    if robo == 0:
        # Set T_1in0_ReferenceFrame position and orientation
        vrep.simxSetObjectPosition(clientID, ref_object_handles[1], ref_object_handles[0], end_position,
                                   vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(clientID, ref_object_handles[1], ref_object_handles[0], end_eulerAngles,
                                      vrep.simx_opmode_oneshot)
    elif robo == 1:
        # Set T_1in0_ReferenceFrame position and orientation
        vrep.simxSetObjectPosition(clientID, ref_object_handles[1], ref_object_handles[2], end_position,
                                   vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(clientID, ref_object_handles[1], ref_object_handles[2],
                                      end_eulerAngles,
                                      vrep.simx_opmode_oneshot)
    else:
        raise Exception('Problem with robot number')

    vrep.simxPauseCommunication(clientID,True)

    # Position all the joint angles according to user input
    # Set the desired value of the first joint variable
    vrep.simxSetJointPosition(clientID, robot_handles[0], theta[0], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the second joint variable
    vrep.simxSetJointPosition(clientID, robot_handles[1], theta[1], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the third joint variable
    vrep.simxSetJointPosition(clientID, robot_handles[2], theta[2], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the fourth joint variable
    vrep.simxSetJointPosition(clientID, robot_handles[3], theta[3], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the fifth joint variable
    vrep.simxSetJointPosition(clientID, robot_handles[4], theta[4], vrep.simx_opmode_streaming)
    #time.sleep(2)  # Wait two seconds

    # Set the desired value of the sixth joint variable
    vrep.simxSetJointPosition(clientID, robot_handles[5], theta[5], vrep.simx_opmode_streaming)
    time.sleep(2)  # Wait two seconds

    vrep.simxPauseCommunication(clientID, False)


#Ends the V-Rep simulation
def end_simulation (clientID):
    time.sleep(2)  # Wait two seconds

    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive.
    # You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Close the connection to V-REP
    vrep.simxFinish(clientID)

#Asks user for angle inputs and returns a vecotre with all six angles in it
def get_user_angles ():

    print('Enter the six angles (in radians) for each joint that you would like the robot to achieve. ')
    theta1 = eval(input('Angle 1: '))
    theta2 = eval(input('Angle 2: '))
    theta3 = eval(input('Angle 3: '))
    theta4 = eval(input('Angle 4: '))
    theta5 = eval(input('Angle 5: '))
    theta6 = eval(input('Angle 6: '))

    theta_array = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    theta_array.shape = (6,1)
    theta = np.matrix(theta_array)

    return (theta)

#Asks the user for the desired position and orientation for the robot to achieve.
#The function returns a position and euler angle vector for position and orientation.
def get_user_position_and_orientation():
    print('Enter the x,y,z position and euler rotation angles you would like the robot end-effector to achieve. ')
    x = eval(input('X: '))
    y = eval(input('Y: '))
    z = eval(input('Z: '))

    position_array = np.array([x,y,z])
    position_array.shape = (3,1)
    position_vector = np.matrix(position_array)

    theta = eval(input('Theta: '))
    gamma = eval(input('Gamma: '))
    beta = eval(input('Beta: '))

    euler_angle_array = np.array([theta,gamma,beta])
    euler_angle_array.shape = (3,1)
    euler_angle_vector = np.matrix(euler_angle_array)

    return (position_vector, euler_angle_vector)

def get_collision_handles(clientID):
    # Get "handle" to the UR3 to UR3#0 collision object
    result, collision_Handle_1 = vrep.simxGetCollisionHandle(clientID, 'Collision', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for UR3 to UR3#0 collision object')

    # Get "handle" to the UR3 to sofa collision object
    result, collision_Handle_2 = vrep.simxGetCollisionHandle(clientID, 'Collision0', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for UR3 to MicoHand#0 collision object')

    # Get "handle" to the UR3#0 to sofa collision object
    result, collision_Handle_3 = vrep.simxGetCollisionHandle(clientID, 'Collision1', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for MicoHand to UR3#0 collision object')

    # Get "handle" to the UR3 to UR3 collision object
    result, collision_Handle_4 = vrep.simxGetCollisionHandle(clientID, 'Collision2', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for MicoHand to MicoHand#0 collision object')

    collision_handles = np.array([collision_Handle_1, collision_Handle_2, collision_Handle_3, collision_Handle_4])
    return collision_handles

def inCollision(clientID, collision_handles):
    # Get "handle" to the UR3 to UR3#0 collision object
    #result, collision_Handle_1 = vrep.simxGetCollisionHandle(clientID, 'Collision', vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for UR3 to UR3#0 collision object')

    # Get "handle" to the UR3 to sofa collision object
    #result, collision_Handle_2 = vrep.simxGetCollisionHandle(clientID, 'Collision0', vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for UR3 to MicoHand#0 collision object')

    # Get "handle" to the UR3#0 to sofa collision object
    #result, collision_Handle_3 = vrep.simxGetCollisionHandle(clientID, 'Collision1', vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for MicoHand to UR3#0 collision object')

    # Get "handle" to the UR3 to UR3 collision object
    #result, collision_Handle_4 = vrep.simxGetCollisionHandle(clientID, 'Collision2', vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for MicoHand to MicoHand#0 collision object')

    # Get "handle" to the MicoHand to MicoHand#0 collision object
    #result, collision_Handle_5 = vrep.simxGetCollisionHandle(clientID, 'Collision6', vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get object handle for collection1 collision object')

    #if robo == 0:
    #    #Get "handle" to the UR3 to chair collision object
    #    result, collision_Handle_2 = vrep.simxGetCollisionHandle(clientID, 'Collision2', vrep.simx_opmode_blocking)
    #    if result != vrep.simx_return_ok:
    #        raise Exception('could not get object handle for UR3 collision object')
    #elif robo == 1:
    #    # Get "handle" to the UR3#0 to chair collision object
    #    result, collision_Handle_2 = vrep.simxGetCollisionHandle(clientID, 'Collision3', vrep.simx_opmode_blocking)
    #    if result != vrep.simx_return_ok:
    #        raise Exception('could not get object handle for UR3#0 collision object')
    #else:
    #    raise Exception('Problem with collision object handles')


    result, collision_state_1 = vrep.simxReadCollision(clientID, collision_handles[0], vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')

    result, collision_state_2 = vrep.simxReadCollision(clientID, collision_handles[1], vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')

    result, collision_state_3 = vrep.simxReadCollision(clientID, collision_handles[2], vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')

    result, collision_state_4 = vrep.simxReadCollision(clientID, collision_handles[3], vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')

    #result, collision_state_5 = vrep.simxReadCollision(clientID, collision_Handle_5, vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get collision state for robot collision object')

    #result, collision_state_2 = vrep.simxReadCollision(clientID, collision_Handle_2, vrep.simx_opmode_blocking)
    #if result != vrep.simx_return_ok:
    #    raise Exception('could not get collision state for robot collision object')

    #if collision_state_1 == False and collision_state_2 == False and collision_state_3 == False and collision_state_4 == False:
    #    collision_state = 0
    #else:
    #    collision_state = 1




    if collision_state_4 == 1:
        print('self collision')
    elif collision_state_4 == 0:
        pass #print('no self collision')
    else:
        print('problem')

    if collision_state_1 == 1 or collision_state_2 == 1 or collision_state_3 == 1 or collision_state_4 == 1:
        collision_state = True
        print('Path is in collision!')
    else:
        collision_state = False
        #print('Path is not in collision.')

    return collision_state


#I don't think we need this function. Will delete after path planner function works
def sphereLocation(theta, p_initial):
    p=np.zeros((3,n+2))
    p[:,1] = p_initial[:,1]
    p[:,2] = p_initial[:,2]

    e_o = 1

    for i in range(0,S.shape[1]):
        e_n = np.dot(e_o, linalg.expm(np.dot(skew4(S[:, i]),theta[i])))
        padded = np.dot(e_n, np.vstack(( p_initial[:, i + 2] , 1)))
        p[:, i + 2] = padded[1:3, 1]
        e_o = e_n

    return p


#need to add funcitonality for checking collision for second robot. (add a robo variable and adjust for collisions)
def robot_in_collision(clientID,theta,robot_handles,collision_handles,ref_object_handles):
    #restart_simulation(clientID)
    #time.sleep(2)  # Wait two seconds

    move_robot(clientID,theta,0, robot_handles,ref_object_handles)
    #move_robot_without_simulation(clientID,theta,0, robot_handles,ref_object_handles)
    time.sleep(2)  # Wait two seconds
    collision_state=inCollision(clientID,collision_handles)
    #time.sleep(2)  # Wait two seconds
    #pause_simulation(clientID)
    #time.sleep(2)  # Wait two seconds
    #collision_state = True
    return collision_state


def linepath_in_collision(clientID,theta_a,theta_b,robot_handles,collision_handles,ref_object_handles):
    print('Commencing linepath collission check')
    #print(theta_a)
    #print('theta_b is:')
    #print(theta_b)
    collision_state = False
    epsilon = 9e-1

    if robot_in_collision(clientID,theta_a,robot_handles,collision_handles,ref_object_handles) or robot_in_collision(clientID,theta_b,robot_handles,collision_handles,ref_object_handles):
        collision_state = True

    col_number_of_theta = theta_a.shape[1]
    #s_vector = np.zeros(1, col_number_of_theta)

    i=0
    while i < col_number_of_theta and not(collision_state):
        d = np.linalg.norm(theta_a[:, i]-theta_b[:, i])
        #print(d)
        n_step = 1 + int(math.ceil(d/epsilon))
        #print(n_step)
        s = np.linspace(0, 1, n_step)
        #print(s)

        j = 0
        while j < n_step and not(collision_state):
            #print(j)
            theta = (1-s[j])*theta_a[:,i]+s[j]*theta_b[:,i]
            #theta = np.dot((1 - s[j]), theta_a[:, i])+np.dot(s[j], theta_b[:, i])
            theta.shape = (6,1)
            #print(theta)
            collision = False
            if robot_in_collision(clientID, theta,robot_handles,collision_handles,ref_object_handles):
                collision_state = True
            j = j + 1
        i = i + 1

    return collision_state


def random_theta():
    theta = np.zeros((n,1))
    theta[0] = np.random.rand()*(2*np.pi)
    theta[1] = (np.random.rand()*1.2 - 1.2) + np.random.rand() * (1.2)
    theta[2] = (np.random.rand() * 2.6 - 2.6) + np.random.rand() * (2.6)
    theta[3] = np.random.rand() * (2 * np.pi)
    theta[4] = np.random.rand() * (2 * np.pi)
    theta[5] = np.random.rand() * (2 * np.pi)
    return theta


def path_planner(clientID,theta_start, theta_goal,robot_handles,collision_handles,ref_object_handles):
    node_start = namedtuple('node_start', ['theta', 'parent' ])
    node_end = namedtuple('node_end', ['theta', 'parent'])
    node_n = namedtuple('node_n', ['theta', 'parent'])
    #joint_number = theta_start.shape[0]

    node_start.theta = theta_start
    node_start.parent = 0

    node_end.theta = theta_goal
    node_end.parent = 0

    start_tree = [node_start]
    #print(start_tree)
    end_tree = [node_end]
    #print(end_tree)
    start_answer = []
    end_answer = []
    complete_answer = []
    complete_answer.append(node_start.theta)

    found = False
    add_to_start_tree_flag = False
    add_to_end_tree_flag = False
    i = 0
    while i<300 and not(found):
        #theta = np.random.rand(n, 1)
        theta = random_theta()
        print('New random theta:')
        #print(theta)
        #move robot to theta , or maybe put it in function robot_in_collision
        #p = sphereLocation(theta, p_robot)
        #collision_state = linepath_in_collision(clientID)

        while robot_in_collision(clientID,theta,robot_handles,collision_handles,ref_object_handles):
            #theta = np.random.rand(n, 1)
            print('New random theta:')
            theta = random_theta()
            #print(theta)
            #p = sphereLocation(theta, p_robot)
            #move robot to theta, or maybe put it in function robot_in_collision
            #collision_state = linepath_in_collision(clientID)

        #Start Tree Check
        print('Commencing start_tree check')
        distance_smallest = np.linalg.norm(theta - start_tree[0].theta)
        shortest_distance_parent_index = 0
        #print(shortest_distance_parent_index)
        for j in range(1,len(start_tree)):
            #print(j)
            distance_new = np.linalg.norm(theta - start_tree[j].theta)
            if distance_new < distance_smallest:
                distance_smallest = distance_new
                shortest_distance_parent_index = j

        #print(start_tree[shortest_distance_parent_index].theta)
        if not linepath_in_collision(clientID,theta, np.array(start_tree[shortest_distance_parent_index].theta),robot_handles,collision_handles,ref_object_handles):
            add_to_start_tree_flag = True

        if add_to_start_tree_flag:
            node_n.theta = theta
            node_n.parent = shortest_distance_parent_index
            start_tree.append(node_n)

        # End Tree Check
        print('Commencing end_tree check')
        distance_smallest = np.linalg.norm(theta - end_tree[0].theta)
        shortest_distance_parent_index = 0
        for k in range(1, len(end_tree)):
            distance_new = np.linalg.norm(theta - end_tree[k].theta)
            if distance_new < distance_smallest:
                distance_smallest = distance_new
                shortest_distance_parent_index = k

        if not linepath_in_collision(clientID, theta, np.array(end_tree[shortest_distance_parent_index].theta),robot_handles,collision_handles,ref_object_handles):
            add_to_end_tree_flag = True

        if add_to_end_tree_flag:
            node_n.theta = theta
            node_n.parent = shortest_distance_parent_index
            end_tree.append(node_n)


        #Checking if theta in both start and end trees
        print('Checking theta in both trees')
        l = 0
        #print('length of start tree:')
        #print(len(start_tree))
        #print('length of end tree:')
        #print(len(end_tree))
        while l <len(start_tree) and not(found):
            #print(l)
            m = 0
            while m < len(end_tree) and not(found):
                #print(m)
                #print('start theta:')
                #print(np.array(start_tree[l].theta))
                #print('end theta')
                #print(np.array(end_tree[m].theta))

                #if np.linalg.norm(np.array(start_tree[l].theta) - theta) < 0.001 and np.linalg.norm(np.array(end_tree[m].theta) -theta) < 0.001:
                if np.array_equal(np.array(start_tree[l].theta), theta) and np.array_equal(np.array(end_tree[m].theta),theta):
                    print('found a path!')
                    #print(found)
                    found = True
                    #print(found)
                    start_tree_connection_index = l
                    end_tree_connection_index = m
                m=m+1
            l=l+1

        if found:
            #trace back start tree
            start_answer_index = 0
            start_answer.append(start_tree[start_tree_connection_index].theta)
            start_next_parent = start_tree[start_tree_connection_index].parent
            start_answer_index = start_answer_index + 1
            while start_next_parent != 0:
                start_answer.append(start_tree[start_next_parent].theta)
                start_next_parent = start_tree[start_next_parent].parent
                start_answer_index = start_answer_index + 1

            start_answer.reverse()

            # trace back end tree
            end_answer_index = 0
            end_answer.append(start_tree[end_tree_connection_index].theta)
            end_next_parent = start_tree[end_tree_connection_index].parent
            end_answer_index = end_answer_index + 1
            while end_next_parent != 0:
                end_answer.append(end_tree[start_next_parent].theta)
                end_next_parent = end_tree[start_next_parent].parent
                end_answer_index = end_answer_index + 1

            #Combine start and end answer
            complete_answer.append(start_answer)
            complete_answer.append(end_answer)
            complete_answer.append(node_end.theta)
        i = i + 1
        #print(start_tree)
        #print(end_tree)

    #print('exited the big loop!!!!')
    #print('The complete path (list) is:')
    #print(complete_answer)

    intermediate = complete_answer[0]
    #print('intermediate')
    #print(intermediate)
    answer_array = intermediate
    #print('answer_array')
    #print(answer_array)
    for i in range(1,len(complete_answer)-1):
        intermediate2 = complete_answer[i]
        #print('intermediate2')
        #print(intermediate2)

        intermediate = intermediate2[0]
        #print('intermediate')
        #print(intermediate)

        answer_array = np.hstack((answer_array,intermediate))
        #print('answer_array')
        #print(answer_array)
        #answer_array[:,i] = complete_answer[i]
    intermediate = complete_answer[-1]
    #print('intermediate')
    #print(intermediate)
    answer_array = np.hstack((answer_array,intermediate))
    #print('answer_array')
    #print(answer_array)

    if found:
        print('The complete path (array) is:')
        print(answer_array)
    else:
        print('Unable to find path')

    return answer_array
