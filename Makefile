prog: comander.o listener_odom.o listener_scan.o
	gcc -o comander.o listener_odom.o listener_scan.o

listener_scan.o: listener_scan.cpp
	gcc -c main.cpp

listener_odom.o: listener_odom.cpp
	gcc -c foo.cpp
	
comander.o: comander.cpp
	gcc -c bar.cpp
