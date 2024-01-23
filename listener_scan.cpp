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

#define PORT 9997
#define IP_Adresse "192.168.100.54"

#define semaphore_name_scan_producer "/semaphore_scan_producer"
#define semaphore_name_scan_consumer "/semaphore_scan_consumer"

using namespace std;

const char* shared_memory_name_scan = "/shared_memory_scan";

struct SharedData_scan {
    char message[10000];
};

int client_id_scan;
void Start_TCP_IP_Connection();
string getMessage_TCP_IP_Connection();
string DataExtract(string msg_String);

int main(int argc, char const* argv[])
{
	string extrahierteDaten="";

	//unlink the semapores
	sem_unlink(semaphore_name_scan_consumer);
	sem_unlink(semaphore_name_scan_producer);
	
	//creat semaphore & initialize
	sem_t *semaphore_scan_write=sem_open(semaphore_name_scan_producer, O_CREAT, 0666, 0);
	if(semaphore_scan_write==SEM_FAILED)
	{
		perror("Semaphore konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	sem_t *semaphore_scan_read=sem_open(semaphore_name_scan_consumer, O_CREAT, 0666, 1);
	if(semaphore_scan_read==SEM_FAILED)
	{
		perror("Semaphore konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	// Create shared memory
    int shm_fd = shm_open(shared_memory_name_scan, O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd, sizeof(SharedData_scan));
    SharedData_scan* sharedData_scan = static_cast<SharedData_scan*>(mmap(nullptr, sizeof(SharedData_scan), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
	
	for(;;)
    {

	sem_wait(semaphore_scan_read);				//semaphore is initalized with 1
	Start_TCP_IP_Connection();
	string msg_String= "";
	msg_String= getMessage_TCP_IP_Connection();		//get Data from TCP and send to shared Memory
	extrahierteDaten=DataExtract(msg_String);		//extract the relevant data from the whole Turtlebot-Message

    const char* message = extrahierteDaten.c_str();
    strncpy(sharedData_scan->message, message, sizeof(sharedData_scan->message) - 1);	//copy the extractred Data to the shared Memory

	close(client_id_scan);			//close the TCP-IP-Connection
    sem_post(semaphore_scan_write);			// Post to the semaphore that the writing-process is finished

	usleep(300000);		//300ms delay
    }

	// Clean up    
	sem_destroy(semaphore_scan_write);
	munmap(sharedData_scan, sizeof(SharedData_scan));	
    shm_unlink(shared_memory_name_scan);

	return 0;
}

void Start_TCP_IP_Connection()
{
	int status;
	struct sockaddr_in serv_addr;

	if ((client_id_scan = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {	
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
		= connect(client_id_scan, (struct sockaddr*)&serv_addr,
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
	const int buffersize = 20000;
	char buffer[buffersize]="";
	string msg_String;
	size_t PosEnd =0;

	msg_String="";
	
	while(true)
	{
	valread = read(client_id_scan, buffer, buffersize - 1);
	msg_String.append(buffer);

	if(msg_String.find(R"---(---START---)---")!=std::string::npos && msg_String.find(R"---(___END___)---")!=std::string::npos)			//Create a whole message from Turtlebot
	{
		break;
	}

	}
	close(client_id_scan);
	cout<<msg_String<<endl;
	return msg_String;
	
}


string DataExtract(string msg_String)
{
	//define the startword "ranges" and the endword "intensities"
    string startWort = R"---("ranges": [)---";
    string endeWort = R"---(], "intensities")---";

    //search the message for the startword and endword
    size_t startPos = msg_String.find(startWort);
    size_t endePos = msg_String.find(endeWort);
    string extrahierteDaten;
	
	extrahierteDaten="";

    // check, if startword and endword are in the message from the turtlebot
    if (startPos<endePos && startPos != string::npos && endePos != string::npos) {
        
        extrahierteDaten = msg_String.substr(startPos + startWort.length(), endePos - startPos - startWort.length());		//extract the data between the startword and endword
		cout<<"Extrahierte Daten Scan: "<<extrahierteDaten<<endl;
        //cout << "Extrahierte Daten: " << extrahierteDaten << endl;		//print out the extracted data
    } 
	else 
	{
        cout << "Markante Worte nicht gefunden." << endl;			//errormassage, when the startword or the endword (or boath) cant get found
		extrahierteDaten="Vorherige Nachricht war nicht vollständig!";
    }

	return extrahierteDaten;
}