


**Task:**

The task of this project is to drive a turtlebot3 model burger out of a box and around a pole (ROLL). The project should be implemented in C++ without the usage of additional software besides standard libraries (no ROS). A standard ROS application is installed on the turtlebot prior to this project. This framework publishes the lidar scan an the odometry of the turtlebot and allows for call of angular and linear movements over a TCP-IP-connection. The developed program should be able to interpret the provide data and compute a commands to drive the robot a given path.


![Task](https://github.com/arthurgritzky/APR-Turtlebot/assets/89546471/84ae5df5-99ca-4b83-87f2-219a590606ed)


The following applications have been created to solve the given task:

**listener_scan.cpp**

The program reads the lidar scan message from the turtlebot. The message gets written in a shared memory. 

**listener_odom.cpp**

The program reads the odom scan message from the turtlebot. The message gets written in a shared memory as well. 

**comander.cpp**

This program use the data of shared memory of scan and odom listener. The messages get processed and the relevant data get extracted. The lidar message contains the distances of the lidar points obtained by the lidar sensor. The shift between distances is used to determine the relative position of pole to the robot. The odometry message provides the position and orientation of the robot absolute to the starting point. The pose at the initialization of the robot is the origin of the coordinate System of the robot. Those coordinates are used to create 4 positions around the pole. The desired movement of the robot is the following:

•	Drive from the start position to first position in front of the pole

•	Drive to the following positions around the pole in a circular counterclockwise movement

•	Return to the starting position


The working principle of the created application is shown in the following system context diagram:


![system_context_diagram](https://github.com/arthurgritzky/APR-Turtlebot/assets/89546471/68c6b370-64c3-49db-9a2c-3cfebbb3db92)




**Usage:**

use the following applications with the following programms:

Compile des LidarReaders:
g++ listener_scan.cpp -o lidarScan.o -lrt

Compile des Odom-Readers:
g++ listener_odom.cpp -o odomScan.o -lrt

Compile the comander:
g++ comander.cpp -o comander.o -lrt

(Note it’s important to execute the commander last, because listener scan and odom are responsible for creation the shared memory onto which the commander attaches)

./lidarScan

./odomScan

./commander




The following parameters need to be ajusted for the application:


o	IP-address of the robot  (in this case 192.168.100.54)

o	Port of scan message from robot (in this case 9997)

o	Port of odom message from robot (in this case 9998)

o	Port of cmd_vel message to the robot(in this case 9999)










