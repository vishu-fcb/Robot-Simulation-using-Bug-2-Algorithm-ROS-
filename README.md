# Robot-Simulation-using-Bug-2-Algorithm-ROS-
This repository contains the code for Robot  Simulation in ROS using bug 2 algorithm. The code has been successfully tested on Gazebo simulator. 

# The folder strucutre should be as folows :

1: In the src folder of your rospackage, copy the following files: dist.py, final_project.py, location.py. Also keep the optional python compiled files 'dist.pyc' and 'location.pyc'.

2: Put scan.launch file in the launch folder inside your rospackage

3: CMakeLists and package.xml are system dependent files so do not replace it directly with the files in the git repository.

### Getting Started

1: Launch Gazebo - Empty world with Turtlebot using the command - roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

2: Launch the ROS package for the final project using the command- roslaunch package_name start.launch

### Pre-requisites

1: Linux OS (Native Linux or Virtual Machine)

2: ROS Environment

3: Gazebo

4: Goal list 	(Published by the Goal Publisher)

5: Arena Model (from project issue in Gitlab)

### General Description

The ultimate goal of the project is to implement an optimized algorithm for the given arena model and reach the maximum number of goals in the stipulated time.

### Topics and Services Required

"/goals" - This topic subscribes to the target goals where the turtlebot should reach in the given maze
"/scan" -  This topic subscribes to Laser scan data.
"/gazebo/model_states" - This subscribes to the current orientation and position of the robot
"/cmd_vel" - This topic published the linear and angular velocity of the robot at each instance

### Algorithm

The BUG 2 algorithm is used in the project and it is optimized to reach the goal specified by the goal_publisher topic.

In Bug2 algorithm, the robot follows the boundary of the encountered obstacle until it reaches the leave point. The leave point is the intersection of the line passing through the start point and the goal  with the closed curve (the obstacle) such that it is closer to the goal than the point where the robot encountered the obstacle (hit point).


### Implementation of Algorithm

The goal points are subscribed from the /goals topic and assigned to a empty python list. The robot orients itself and moves towards the goal until the obstacle. Once, the obstacle is encountered, it goes around the obstacle and joins back in the original path. The obstacle avoidance is implemented by follow_wall() function, which is implemented in both left-side priority rule and also right-side priority rule. Once the robot joins back in the path, the should_leave_wall() function invokes the face_goal and orients the robot to move towards the goal.

### Problem

The usage of either of left-side priority rule or right-side priority rule alone restricts the movement of robot in deciding to fix the nearest goal position. This leads to greater amount of time taken by the robot to reach certain goals given.

### Solution

The main program is designed with options of having both left-side priority rule and right-side priority rule based on specific goal points.Right side priortiy is assigned for goals with negative X coordiante and Left side priority is assigned for goals with positive Y coordinate.Hence, for each of the given goal,  option can be made either to follow left-side priority or right-hand priority to reach the goal in minimum time.
