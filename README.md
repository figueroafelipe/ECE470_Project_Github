# ECE470_Project Demonstrate robot motion with code in V-rep
## Prerequisites
V-REP PRO EDU version 3.4.0<br />
Python 3.6 version

## Installing
**For MacOS Operation** 

1. Download the V-REP PRO EDU from the Coppelia Robotic Website. It is recommended to use version 3.4.0 instead of the newest version
[Here](http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_4_0_Mac.zip)

2. Run V-Rep: open a new Terminal -> Change directory to the path of the V-REP stored ->./vrep.app/Contents/MacOS/vrep
In the terminal find the line "Starting a remote APT server on port 19997". The port number is
the port that the code communicates with V-REP.

3. Drag the UR3 robot into the GUI from the robots-> non-mobile. Drag the Micro hand into the GUI from 
grippers. To attach the gripper to the robot arm select the gripper, then select (with command click) the connection spot (usually in the red), then click the assembly toolbar.

4. To remove the demo scripts that the robot comes with, right click on the inserted robot-> Edit -> Remove 
-> "Associated Child Script".

5. File -> Save Scene As...(e.g., my_ur3.ttt). 

6. Install Python with anaconda distribution [Here](https://www.anaconda.com/download/#macos)
   (Choose Python 3.6 version)
   
7. Download the enviroment [ece470.yml](https://d1b10bmlvqabco.cloudfront.net/attach/jchxn1s6tkg20r/h6wx8zvddi8vt/je9d8omtib3t/ece470.yml)
   this is the environment provided for this project. To active an environment:<br />
   In Terminal, run "source active ece470" 
   For more information about creating an environment [Help](https://conda.io/docs/user-guide/tasks/manage-environments.html#creating-an-environment-from-an-environment-yml-file)
   
## Running the tests
Reference the test.py to write the code showing each joint moving one at a time to its positive and negative joint limits. 

Since the arm of the UR3 might interact with itself or the floor use the program to check the 
limit of each joints. If the arm is not able to reach its target angle due to an obstacle, output the current angle of the joint.

Save the code as ... (e.g. Project_Assignment2) and run the code in the terminal with "python Project_Assignment2.py"

   
## Forward Kinematics 
## Due 03-12
Derive the forward Kinematics based on the UR3 schematic in V-Rep
![Alt text](picture new address)
All the dimensions of UR3 are found in the UR3 CAD model.
![Alt text](picture new address)

The screw axis for the 6 joints are<br /> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S1=[&nbsp;&nbsp;0;&nbsp;0;&nbsp;1;&nbsp;0;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0;&nbsp;0]<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S2=[-1;&nbsp;0;&nbsp;0;&nbsp;0;&nbsp;-0.152;&nbsp;0]<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S3=[-1;&nbsp;0;&nbsp;0;&nbsp;0;-0.396;&nbsp;0]<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S4=[-1;&nbsp;0;&nbsp;0;&nbsp;0;-0.609;&nbsp;0]<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S5=[&nbsp;0;&nbsp;0;&nbsp;&nbsp;1;&nbsp;0;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0.11;&nbsp;0]<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;S6=[-1;&nbsp;0;&nbsp;0;&nbsp;0;-0.692;&nbsp;0]<br />
The initial M is<br />
M=[1 0 0   -0.2455; 0 1 0 0; 0 0 1 0.692; 0 0 0 1]<br />
User given a set of six angles within the joints limit. The tool frame pose can be predicted as 
T=expm([S1]*theta1) * expm([S2]*theta2)*expm([S3]*theta3)*expm([S4]*theta4)*expm([S5]theta5)*expm([S6]*theta6)*M

## Running the tests
Running the test and compare the pose of the tool in V-REP with the code output pose. Both the position and the rotation.
Rotation matrix need to convert to Euler angles in python

## Inverse Kinematics
## Due 03-27
Derive the Inverse Kinematics based on the UR3 schematic and previous calculatd screw axis for the 6 joints
(1) Random generating or inputing a goal pose in the code of the UR3
(2) Draw a frame in the simulator at the goal pose and add a frame of the tool
(3) Use iterate algorithm to generate the 6 joint angles that achieve the goal pose<br />
<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Choose a random set of theta as the initial guess<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Find the current pose of the tool T1in0(theta)<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Find the bracket of the spatial twist that alighn frame 1 to frame 2 logm(T2in0*inv(T1in0))<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Find J(theta) with current set of joint variables<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Calculate the inverse velocity kinematic: theta_dot=inv(J)*spatial twist<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;theta=theta+theta_dot<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The stopping point is when the norm of the twist is smaller than a constant sigma. The tool reaches to goal position
<br />
<br />
(4) Compare the goal pose frame with the tool frame to see if it's aligned
(5) Due to the inital guess of the set of theta. The algorithm may not converge. Or the robot may be in the condition of self-collision. Indicate the goal pose is not reachable in the code

## Demostrate Collision Detection
## Due 04-02
The Instruction given three suggusted method<br />
1. Use the V-rep build in functions<br />
2. Use the external library<br />
3. Use the bounding volumes method<br />

The bounding volumes method is to set dummies or spheres at the joints to represent the robot each part's volume. Compare the distance of
the center of the two bounding volume with the the sum of two radii to check if the robot whether is in collition with itself or other obstacles. The impementation is easy to conduct. However, the result would not be as accurate as the first two. 

In the project, we choose to use the build in function simxGetCollisionHandle and function simxReadCollision to write the isCollision funtion in our code. The isCollision check 
1. Get "handle" to the UR3 TO UR3 collision - self-collision detection<br />
2. Get "handle" to the UR3 to chair collison -environment-collision detection<br />
The function print the result of the collision detection of any scenarios.

The user input the desired position. The robot is conduction the inverse kinematics with collision detection

## Demostrate Motion Planning
## Due 04-11
User input the start configuration and goal configuration. Use the V-rep build in function to detect the obstacles from the previous project and return a collision free path. If the free path could not be found, return failure

Write the move_robot_without_simulation function to moves the joint of the robot to the prescirbed angle given without simulation running to detect the locations of the obstacles

Path Plannar Algorithm
1. Initialize a star tree and a end tree
2. Find a random set of theta as presecribed set of joints angle check whether the robot is in the free space. 
3. Caluculte the distance bewteen the current node with the last node of start tree and the end tree. Find the smallest distance. If the smallest distance is between the star tree. Call the linepath_in_collision function to check whether there is a collision in the line path between current set and the start tree node. If its collision free, add the the current node to the start tree
4. If the node of end tree has the smallest distance, similar step as step 3.
5. Until the start tree connects to the end tree. Track back the collision free path. If the free path can not be found, return failure.

Simulate the robot with the collision free path

## Closed Chain Kinematics
There are two approaches to achieve the dual-arm closed chain kinematics based on the previous derived method. First is the linearly interpolate object position from the initial position to the goal position. Apply a rigid body transformation to compute the pose of the tool frame for the UR3 A and the UR3 B. Apply the inverse kinematics to compute the joint angles for both the UR3 A and UR3 B with the interpolated object position poses. The second method is linearly interpolating one robot's joints angles to calculate the other robot's joints angles. Choose sample joint angles for the UR3 A.  Apply forward kinematics to compute the pose of the tool frame for robot A. Use matrix transformation to get the pose of the UR3 B tool frame. Apply inverse kinematics to compute the joint angles for the UR3 B. Apply the forward kinematics for both the UR3 A and B the same time with the sample joint angles for A and calculated joint angles for B.

Init.
thetaAStart = InverseKinematics ( M_start)
thetaAGoal = InverseKinematics ( M_goal)

for s = linspace (0, 1, 100)
     thetaA = (1- s) * thetaAStart + s * thetaAGoal
     M_A= ForwardKinematics (thetaA)
     M_B = T * M_A
     thetaB = InverseKinematics (M_B)
 end


## References



## Authors
Felipe Figueroa<br />
Di Zhu

## License
MIT License
