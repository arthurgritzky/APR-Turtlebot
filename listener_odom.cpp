// Client side C/C++ program to demonstrate Socket

//Compile: gcc Client.cpp -o client


// programming
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



#define PORT 9998			//Bei Turtlebot: Port richtig einstellen
#define IP_Adresse "192.168.100.54"
//#define IP_Adresse "127.0.0.1"

#define semaphore_name_odom_producer "/semaphore_odom_producer"
#define semaphore_name_odom_consumer "/semaphore_odom_consumer"

using namespace std;


//definition shared Memory
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
//string DataExtract(string msg_String);
string GetValue(string msg_String, string startWort, string endeWort);
double Winkelberechnung_quad_to_euler(string x_Value_orientation, string y_Value_orientation, string z_Value_orientation, string w_Value_orientation);

int main(int argc, char const* argv[])
{
	//string message="";
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
	sem_t *semaphore_odom_write=sem_open(semaphore_name_odom_producer, O_CREAT, 0666, 0);		//"0"		//O_Create anstatt IPC_CREAT
	if(semaphore_odom_write==SEM_FAILED)
	{
		perror("Semaphore konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	sem_t *semaphore_odom_read=sem_open(semaphore_name_odom_consumer, O_CREAT, 0666, 1);	//"1"
	if(semaphore_odom_read==SEM_FAILED)
	{
		perror("Semaphore konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	// Create shared memory
    int shm_fd = shm_open(shared_memory_name_odom, O_CREAT | O_RDWR, 0666);			//Create SHM da "0_Create"

    ftruncate(shm_fd, sizeof(SharedData_odom));
    SharedData_odom* sharedData_odom = static_cast<SharedData_odom*>(mmap(nullptr, sizeof(SharedData_odom), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
	
	for(;;)
    {

	sem_wait(semaphore_odom_read);	

	Start_TCP_IP_Connection();

	string msg_String= "";

	//get Data from TCP and send to shared Memory
	msg_String= getMessage_TCP_IP_Connection();
	//usleep(250000);		//macht die Ausgabe lesbarer

	cout<<msg_String<<endl;

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

	//close TCP-IP Verbindung
	close(client_id_odom);

	Winkel_Z=Winkelberechnung_quad_to_euler(x_Value_orientation, y_Value_orientation, z_Value_orientation, w_Value_orientation);
	message_Zwischenspeicher=to_string(Winkel_Z).c_str();			//konvertieren von Double in String in char
	strncpy(sharedData_odom->Eulerwinkel_Z, message_Zwischenspeicher, sizeof(sharedData_odom->Eulerwinkel_Z) - 1);	
	
	cout<<"EUlerwinkel um Z: "<<Winkel_Z<<endl;

	// Post to the semaphore that the writing-process is finisheds
    sem_post(semaphore_odom_write);	//setzen Semaphre von 0 auf 1

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
	serv_addr.sin_family = AF_INET;		//AF_Inet für IPV4 und AF_Inet6 für IPV6
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary
	// form
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
	valread = read(client_id_odom, buffer, buffersize - 1); // subtract 1 for the null terminator at the end
	msg_String.append(buffer);

	if(msg_String.find(R"---(---START---)---")!=std::string::npos && msg_String.find(R"---(___END___)---")!=std::string::npos)
	{
		break;
	}

	}
	close(client_id_odom);
	//cout<<msg_String<<endl;		//Komplette Nachricht anzeigen
	return msg_String;
	
}

string GetValue(string msg_String, string startWort, string endeWort)
{

    // Die Positionen der markanten Wörter im String finden
    size_t startPos = msg_String.find(startWort);
    size_t endePos = msg_String.find(endeWort);
    string extrahierteDaten;						//Zugeschnittene Nachricht
	
	extrahierteDaten="";

    // Überprüfen, ob beide Wörter im String gefunden wurden
    if (startPos<endePos && startPos != string::npos && endePos != string::npos) {
        // Die Daten zwischen den markanten Wörtern extrahieren
        extrahierteDaten = msg_String.substr(startPos + startWort.length(), endePos - startPos - startWort.length());

		//cout<<"Extrahierte Daten: " <<extrahierteDaten<<endl;

        // Die extrahierten Daten ausgeben
        //cout << "Extrahierte Daten: " << extrahierteDaten << endl;
    } else {
        cout << "Markante Worte nicht gefunden." << endl;
		extrahierteDaten="Vorherige Nachricht war nicht vollständig!";
		//cout<<msg_String<<endl;
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



	//Berechung Winkel um z
	WinkelEuler=atan2(2*(w_Value_o*z_Value_o+x_Value_o*y_Value_o), 1-2*((y_Value_o*y_Value_o)+(z_Value_o*z_Value_o)));


	return WinkelEuler;
}