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





## References



## Authors
Felipe Figueroa<br />
Di Zhu

## License
MIT License
