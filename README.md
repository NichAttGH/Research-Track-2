# Second Assignment for Research Track 1

### Introduction to the Code
The assignment provides the use of Rviz and Gazebo for graphical interfaces and a robot that can move in the environment with the possibility to avoid obstacles.

For the assignment were created 3 nodes: **_Node_A_**, **_Node_B_** and **_Node_C_**.

The **_Node_A_** allows the robot to go to the point that the user wants, the **_Node_B_** allows the user to know the number of goal reached and canceled, while the **_Node_C_** provides the distance from the actual position of the robot to the desired position that the robot must reach and the average speed.

If you want to run the project, there are several requirements to follow (**_Pay attention: these requirements are for ubuntu 20.04_**):
1. **_open the terminal shell;_**
2. **_install ROS noetic by following the guide in this link:_** `http://wiki.ros.org/noetic/Installation/Ubuntu`
3. **_install gazebo dependencies from this link:_** `https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros`
4. **_install xterm with this command:_** `sudo apt-get install xterm`
5. **_go into the directory of the workspace;_**
6. **_download the repository with this command:_** `git clone https://github.com/NichAttGH/2_assignment_RT1.git`
7. **_run the command:_** `roslaunch assignment_2_2022 assignment_2.launch`

At this point will be opened 4 windows: 1 for Rviz, 1 for Gazebo and 2 for respectively *Node_A.py* and *Node_C.py*.

The *Node_A.py* require that the user must type 2 coordinates: the coordinate X and the coordinate Y that the robot need to move to the desired point that the user want.

While, the *Node_C.py* show continously the distance from the current position of the robot to the desired position and the average speed.

Instead, the *Node_B.py* is a service that show the number of the goals reached from the robot and goals canceled from the user; furthermore it need to be called in a new terminal shell with this command: `rosservice call Node_B.py`

### Flowchart of the Node_A.py
![Flowchart 2_assignment_RT1 drawio](https://user-images.githubusercontent.com/123623443/222166255-0a674a1a-c6d3-4459-b657-a6159ca53077.png)


### Future improvements
Some ideas for future improvements:
- A way to know the coordinates of the whole area of the environment because in this case we could see exactly where the robot can go (this point is maybe the priority for a possible update becuase when the user type coordinates that refer to a point that is out of the simulation, the program doesn't work so good indeed when the robot can't arrive to the desired position, the program increments the counter of the goal canceled);
- A way to allow the robot to choose the fastest direction to go to the desired position from the user (because now the robot choose alone which direction to take, so there is a possibility that the robot choose the slowest direction to go to the desired position from the user);
