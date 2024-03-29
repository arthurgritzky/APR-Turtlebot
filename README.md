


**Task:**

The task of this project is to drive a turtlebot3 model burger out of a box and around a pole (ROLL). The project should be implemented in C++ without the usage of additional software besides standard libraries (no ROS). A standard ROS application is installed on the turtlebot prior to this project. This framework publishes the lidar scan and the odometry data of the turtlebot and allows for call of angular and linear movements over a TCP-IP-connection (cmd_vel). The developed program should be able to interpret the provide data and compute a commands to drive the robot a given path.


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

(Note it’s important to execute the comander last, because listener scan and odom are responsible for creation the shared memory and semaphores onto which the commander attaches)

./lidarScan.o

./odomScan.o

./comander.o




**Ajustable parameters:**


o	IP-address of the robot  (in this case 192.168.100.54)

o	Port of scan message from robot (in this case 9997)

o	Port of odom message from robot (in this case 9998)

o	Port of cmd_vel message to the robot (in this case 9999)


**Shared memory in combination with semaphores is used for data transfers within the application**

This application uses shared memory to transfer data from the listers to the commander. In order to avoid read and write conflicts semaphores are used.
This way simultaneous reading and writing in the shared memory is prevented. A que of write first then read is established. 

![share_memory_with_semaphores](https://github.com/arthurgritzky/APR-Turtlebot/assets/89546471/71abbb17-e117-4410-8193-6abfa39b56e6)


**Communication with the turtlebot**

•	lidar scan

![message_scan](https://github.com/arthurgritzky/APR-Turtlebot/assets/89546471/6b989244-2493-4bf6-93bb-1332757a472c)

The distances are used to determine the relative position of the pole.

•	odom message

![message_odom](https://github.com/arthurgritzky/APR-Turtlebot/assets/89546471/169e5689-4a59-4f21-8144-89383595f0f5)

The odom message is used to determine the position an oriontation of the robot in regards to the starting position (origing)

•	cmd_vel message

![message_cmd_vel](https://github.com/arthurgritzky/APR-Turtlebot/assets/89546471/f515c5e6-9438-4055-b036-739495b23527)

This command allows for the request of a linear and angular velocity of the robot.





