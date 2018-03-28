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
![Alt text](https://github.com/figueroafelipe/ECE470_Project/blob/master/schematic%20ur3.jpg?raw=true "Title")
All the dimensions of UR3 are found in the UR3 CAD model.
![Alt text](https://github.com/figueroafelipe/ECE470_Project/blob/master/UR3%20CAD.jpg?raw=true "Title")

The screw axis for the 6 joints are<br /> 
S1=[0;0;1;0;0;0]<br />
S2=[-1;0;0;0;-0.152;0]<br />
S3=[-1;0;0;0;-0.396;0]<br />
S4=[-1;0;0;0;-0.609;0]<br />
S5=[0;0;1;0;0.11;0]<br />
S6=[-1;0;0;0;-0.692;0]<br />
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

## References



## Authors
Felipe Figueroa<br />
Di Zhu

## License
MIT License
