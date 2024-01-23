#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <bits/stdc++.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <math.h>

#define PORT 9998
#define IP_Adresse "192.168.100.54"

#define semaphore_name_odom_producer "/semaphore_odom_producer"
#define semaphore_name_odom_consumer "/semaphore_odom_consumer"

using namespace std;

const char* shared_memory_name_odom = "/shared_memory_odom";

struct SharedData_odom {
    char message[10000];
	char x_Wert_odom[20];
	char y_Wert_odom[20];
	char x_Wert_odom_orientation[20];
	char y_Wert_odom_orientation[20];
	char z_Wert_odom_orientation[20];
	char w_Wert_odom_orientation[20];
	char Eulerwinkel_Z[10];
};


int client_id_odom;
void Start_TCP_IP_Connection();
string getMessage_TCP_IP_Connection();
string GetValue(string msg_String, string startWort, string endeWort);
double Winkelberechnung_quad_to_euler(string x_Value_orientation, string y_Value_orientation, string z_Value_orientation, string w_Value_orientation);

int main(int argc, char const* argv[])
{
	string extrahierteDaten_position="";
	string extrahierteDaten_orientation="";
	string x_Value="";
	string y_Value="";
	string x_Value_orientation="";
	string y_Value_orientation="";
	string z_Value_orientation="";
	string w_Value_orientation="";

	double Winkel_Z=0.0;

	//unlink the semapores
	sem_unlink(semaphore_name_odom_consumer);
	sem_unlink(semaphore_name_odom_producer);
	
	//creat semaphore & initialize
	sem_t *semaphore_odom_write=sem_open(semaphore_name_odom_producer, O_CREAT, 0666, 0);
	if(semaphore_odom_write==SEM_FAILED)
	{
		perror("Semaphore konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	sem_t *semaphore_odom_read=sem_open(semaphore_name_odom_consumer, O_CREAT, 0666, 1);
	if(semaphore_odom_read==SEM_FAILED)
	{
		perror("Semaphore konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	// Create shared memory
    int shm_fd = shm_open(shared_memory_name_odom, O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd, sizeof(SharedData_odom));
    SharedData_odom* sharedData_odom = static_cast<SharedData_odom*>(mmap(nullptr, sizeof(SharedData_odom), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
	
	for(;;)
    {

	sem_wait(semaphore_odom_read);			//semaphore is initalized with 1
	Start_TCP_IP_Connection();
	string msg_String= "";
	msg_String= getMessage_TCP_IP_Connection();			//get Data from TCP and send to shared Memory

	cout<<"Odometry Message: "<<msg_String<<endl;

	//extract the relevant data from the whole Turtlebot-Message and copy it to shared Memory Odom
	extrahierteDaten_position=GetValue(msg_String,R"---("position": {)---", R"---(}, "orientation")---");
	const char* message_Zwischenspeicher = extrahierteDaten_position.c_str();
    strncpy(sharedData_odom->message, message_Zwischenspeicher, sizeof(sharedData_odom->message) - 1);	

	//extract X-Vaule
	x_Value=GetValue(extrahierteDaten_position, R"---("x": )---", R"---(, "y")---");
	message_Zwischenspeicher=x_Value.c_str();
	strncpy(sharedData_odom->x_Wert_odom, message_Zwischenspeicher, sizeof(sharedData_odom->x_Wert_odom) - 1);	
	cout<<"Wert X-Koordiate: "<<message_Zwischenspeicher<<endl;

	//extract Y-Value
	y_Value=GetValue(extrahierteDaten_position, R"---("y": )---", R"---(, "z")---");
	message_Zwischenspeicher=y_Value.c_str();
	strncpy(sharedData_odom->y_Wert_odom, message_Zwischenspeicher, sizeof(sharedData_odom->y_Wert_odom) - 1);
	cout<<"Wert Y-Koordiate: "<<message_Zwischenspeicher<<endl;	


	//extract Orientation out of whole Message
	extrahierteDaten_orientation=GetValue(msg_String, R"---("orientation": {)---", R"---(, "covariance)---");
	cout<<"Extrahierte Daten: "<<extrahierteDaten_orientation<<endl;

	//extract X-Orientation
	x_Value_orientation=GetValue(extrahierteDaten_orientation, R"---("x": )---", R"---(, "y")---");
	message_Zwischenspeicher=x_Value_orientation.c_str();
	strncpy(sharedData_odom->x_Wert_odom_orientation, message_Zwischenspeicher, sizeof(sharedData_odom->x_Wert_odom_orientation) - 1);	
	cout<<"Wert X-Ausrichtung: "<<message_Zwischenspeicher<<endl;

	//extract Y-Orientation
	y_Value_orientation=GetValue(extrahierteDaten_orientation, R"---("y": )---", R"---(, "z")---");
	message_Zwischenspeicher=y_Value_orientation.c_str();
	strncpy(sharedData_odom->y_Wert_odom_orientation, message_Zwischenspeicher, sizeof(sharedData_odom->y_Wert_odom_orientation) - 1);	
	cout<<"Wert Y-AUsrichtung: "<<message_Zwischenspeicher<<endl;

	//extract Z-Orientation
	z_Value_orientation=GetValue(extrahierteDaten_orientation, R"---("z": )---", R"---(, "w")---");
	message_Zwischenspeicher=z_Value_orientation.c_str();
	strncpy(sharedData_odom->z_Wert_odom_orientation, message_Zwischenspeicher, sizeof(sharedData_odom->z_Wert_odom_orientation) - 1);	
	cout<<"Wert Z-Ausrichtung: "<<message_Zwischenspeicher<<endl;

	//extract W-Orientation
	w_Value_orientation=GetValue(extrahierteDaten_orientation, R"---("w": )---", R"---(}})---");
	message_Zwischenspeicher=w_Value_orientation.c_str();
	strncpy(sharedData_odom->w_Wert_odom_orientation, message_Zwischenspeicher, sizeof(sharedData_odom->w_Wert_odom_orientation) - 1);	
	cout<<"Wert W-Ausrichtung: "<<message_Zwischenspeicher<<endl;

	close(client_id_odom);			//close TCP/IP-connection

	Winkel_Z=Winkelberechnung_quad_to_euler(x_Value_orientation, y_Value_orientation, z_Value_orientation, w_Value_orientation);		
	message_Zwischenspeicher=to_string(Winkel_Z).c_str();
	strncpy(sharedData_odom->Eulerwinkel_Z, message_Zwischenspeicher, sizeof(sharedData_odom->Eulerwinkel_Z) - 1);	
	
	cout<<"Eulerwinkel um Z: "<<Winkel_Z<<endl;

    sem_post(semaphore_odom_write);			// Post to the semaphore that the writing-process is finisheds

	usleep(300000);		//300ms delay
	
    }

	// Clean up    
	sem_destroy(semaphore_odom_write);
	munmap(sharedData_odom, sizeof(SharedData_odom));	
    shm_unlink(shared_memory_name_odom);

	return 0;
}

void Start_TCP_IP_Connection()
{
	int status;
	struct sockaddr_in serv_addr;

	if ((client_id_odom = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {	
		printf("\n Socket creation error \n");
		return;
	}
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary
	if (inet_pton(AF_INET, IP_Adresse, &serv_addr.sin_addr)
		<= 0) {
		printf(
			"\nInvalid address/ Address not supported \n");
		return;
	}

	if ((status
		= connect(client_id_odom, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		printf("\nConnection Failed \n");
		return;
	}

}


string getMessage_TCP_IP_Connection()
{
	int valread;
	char* msg = "Hello from client";
	const int buffersize = 2000;
	char buffer[buffersize]="";
	string msg_String;
	size_t PosEnd =0;

	msg_String="";
	
	while(true)
	{
	valread = read(client_id_odom, buffer, buffersize - 1);
	msg_String.append(buffer);

	if(msg_String.find(R"---(---START---)---")!=std::string::npos && msg_String.find(R"---(___END___)---")!=std::string::npos)			//Create a whole message from Turtlebot
	{
		break;
	}

	}
	close(client_id_odom);
	return msg_String;
}

string GetValue(string msg_String, string startWort, string endeWort)
{
    size_t startPos = msg_String.find(startWort);		//search for the startword in the message
    size_t endePos = msg_String.find(endeWort);			//search for the endword in the message
    string extrahierteDaten;
	
	extrahierteDaten="";

    // check, if startword and endword are in the message from the turtlebot
    if (startPos<endePos && startPos != string::npos && endePos != string::npos) {
        
        extrahierteDaten = msg_String.substr(startPos + startWort.length(), endePos - startPos - startWort.length());				//extract the data between the startword and endword

		//cout<<"Extrahierte Daten: " <<extrahierteDaten<<endl;			//print out the extracted data

    } 
	else
	{
        cout << "Markante Worte nicht gefunden." << endl;		//errormassage, when the startword or the endword (or boath) cant get found
		extrahierteDaten="Vorherige Nachricht war nicht vollständig!";
    }

	return extrahierteDaten;
}


double Winkelberechnung_quad_to_euler(string x_Value_orientation, string y_Value_orientation, string z_Value_orientation, string w_Value_orientation)
{
	double x_Value_o=0.0;
	double y_Value_o=0.0;
	double z_Value_o=0.0;
	double w_Value_o=0.0;
	double WinkelEuler=0.0;

	//Convert String to double
	x_Value_o=stod(x_Value_orientation);
	y_Value_o=stod(y_Value_orientation);
	z_Value_o=stod(z_Value_orientation);
	w_Value_o=stod(w_Value_orientation);

	//calculate the euler-angle
	WinkelEuler=atan2(2*(w_Value_o*z_Value_o+x_Value_o*y_Value_o), 1-2*((y_Value_o*y_Value_o)+(z_Value_o*z_Value_o)));

	return WinkelEuler;
}