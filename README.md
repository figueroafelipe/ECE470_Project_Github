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
   
## Forward Kinematics 
## Due 03-12
Derive the forward Kinematics based on the UR3 schematic in V-Rep
All the dimensions of UR3 are found in the UR3 CAD model.
The screw axis for the 6 joints are 
S1=[0;0;1;0;0;0]
S2=[-1;0;0;0;-0.152;0]
S3=[-1;0;0;0;-0.396;0]
S4=[-1;0;0;0;-0.609;0]
S5=[0;0;1;0;0.11;0]
S6=[-1;0;0;0;-0.692;0]

## Running the tests
Reference the test.py to write the code showing each joint moving one at a time to its positive and negative joint limits. 

Since the arm of the UR3 might interact with itself or the floor use the program to check the 
limit of each joints. If the arm is not able to reach its target angle due to an obstacle, output the current angle of the joint.

Save the code as ... (e.g. Project_Assignment2) and run the code in the terminal with "python Project_Assignment2.py"

## Authors
Felipe Figueroa<br />
Di Zhu

## License
MIT License
