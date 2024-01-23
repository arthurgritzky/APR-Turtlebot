# APR-Turtlebot

Usage:

telnet <ip_address> 9997 # For the LiDAR Data
telnet <ip_address> 9998 # for the Odometry Data
or using your listener nodes on the ports:

9997 (LiDAR Data)
9998 (Odometry Data)
To control the turtlebot send the starting command to the port 9999 either via telnet or your commander

For example:

telnet <ip_address> 9999
---START---{"linear": 0.1, "angular": 0.10}___END___
---START---{"linear": 0.0, "angular": 0.00}___END___
