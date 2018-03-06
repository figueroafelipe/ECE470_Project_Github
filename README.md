# ECE470_Project Demonstrate robot motion with code in V-rep
## Prerequisites
V-REP PRO EDU version 3.4.0
Python 3.6 version

## Installing
**For MacOS Operation** 

1. Download the V-REP PRO EDU from the Coppelia Robotic Website. The Professor
suggested to use version 3.4.0 instead of the newest version
[Here](http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_4_0_Mac.zip)

2. Run V-Rep: open Terminal -> Cd path of the V-REP stored ->./vrep.app/Contents/MacOS/vrep
In the terminal find the line "Starting a remote APT server on port 19997". The port number is
the port that the code communicate with V-REP.

3. Drag the UR3 robot into the GUI from the robots-> non-mobile. Drag the Mico hand into the GUI from 
grippers. Select the gripper, then select (with command click) the connection spot (usually in the red),
then click the assembly toolbar

4. Remove the demo scripts that robot come with. Right click on the inserted robot-> Edit -> Remove 
-> "Associated Child Script".

5. File -> Save Scene As...(e.g., my_ur3.ttt). 

6. Install Python [Here](https://www.anaconda.com/download/#macos)
   (Choose Python 3.6 version)
   
7. Download the enviroment [ece470.yml](https://d1b10bmlvqabco.cloudfront.net/attach/jchxn1s6tkg20r/h6wx8zvddi8vt/je9d8omtib3t/ece470.yml)
   the one professor uses for the class. To active an environment:
   In Terminal, run "source active ece470" 
   For more information about create the environment [Help](https://conda.io/docs/user-guide/tasks/manage-environments.html#creating-an-environment-from-an-environment-yml-file)

## Running the tests
Reference the test.py to write the code showing each joint moving one at a time to its positive and negative joints limit. 

Since the arm of the UR3 might interactive with itself or the floor. Use the test file to check the 
limit of each joints. If the program stop, output the current anle of the joint.

Save the code as ... (e.g. Project_Assignment2) and run the code in the terminal with "python Project_Assignment2.py"

## Authors

## License
