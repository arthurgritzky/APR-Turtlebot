


**Task:**

The task of this project is to drive a turtlebot3 model burger out of a box and around a pole (ROLL). The project should be implemented in C++ without the usage of additional software besides standard libraries (no ROS). A standard ROS application is installed on the turtlebot prior to this project. This framework publishes the lidar scan an the odometry of the turtlebot and allows for call of angular and linear movements over a TCP-IP-connection. The developed program should be able to interpret the provide data and compute a commands to drive the robot a given path.

The following applications have been created to solve the given task:

**listener_scan.cpp**

The program reads the lidar scan message from the turtlebot. The message gets written in a shared memory. 
**listener_odom.cpp**

The program reads the odom scan message from the turtlebot. The message gets written in a shared memory as well. 

**command.cpp**

This program use the data of shared memory of scan and odom listener. The messages get processed and the relevant data get extracted. The lidar message contains the distances of the lidar points obtained by the lidar sensor. The shift between distances is used to determine the relative position of pole to the robot. The odometry message provides the position and orientation of the robot absolute to the starting point. The pose at the initialization of the robot is the origin of the coordinate System of the robot. Those coordinates are used to create 4 positions around the pole. The desired movement of the robot is the following:
•	Drive from the start position to first position in front of the pole
•	Drive to the following positions around the pole in a circular counterclockwise movement
•	Return to the starting position






**Usage:**

create the following applications with the following programms:

Compile des LidarReaders:
g++ listener_scan.cpp -o lidarScanner.o -lrt

Compile des Odom-Readers:
g++ listener_odom.cpp -o odomScanner.o -lrt

Compile the controller:
g++ comander.cpp -o comander.o -lrt

telnet <ip_address> 9997 # For the LiDAR Data

telnet <ip_address> 9998 # for the Odometry Data


**TCP/IP-ports:**

9997 (LiDAR Data)
9998 (Odometry Data)
To control the turtlebot send the starting command to the port 9999 either via telnet or your commander

For example:

telnet <ip_address> 9999
---START---{"linear": 0.1, "angular": 0.05}___END___ 
---START---{"linear": 0.0, "angular": 0.00}___END___
