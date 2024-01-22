#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define semaphore_name_scan_producer "/semaphore_scan_producer"
#define semaphore_name_scan_consumer "/semaphore_scan_consumer"

#define semaphore_name_odom_producer  "/semaphore_odom_producer"
#define semaphore_name_odom_consumer  "/semaphore_odom_consumer"

#define PORT 9999
#define IP_Adresse "192.168.100.54"
//#define IP_Adresse "127.0.0.1"

using namespace std;

const char* shared_memory_name_scan = "/shared_memory_scan";

const char* shared_memory_name_odom = "/shared_memory_odom";


int client_id;
float Abstand_b=0.3;                //Abstand vor Rohr

struct SharedData_scan {
    char message[10000];
};

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

void Start_TCP_IP_Connection();
string Kantenerkennung(double Zahlenarray[360]);
void Kreisfahrt();

int main() {

    char msg_from_shm_lidar[10000];
    char msg_from_shm_odom[10000];

    float linear=0.00;      //zwei Nachkommastellen!
    float angular=0.00;     //zwei Nachkommastellen!
    const char* msg_cmdvel;

    int TurtlebotStatus=0;

    // Open shared memory scan
    int shm_id = shm_open(shared_memory_name_scan, O_RDWR, 0666);
    SharedData_scan* sharedData_scan = static_cast<SharedData_scan*>(mmap(nullptr, sizeof(SharedData_scan), PROT_READ | PROT_WRITE, MAP_SHARED, shm_id, 0));

    int shm_id_odom = shm_open(shared_memory_name_odom, O_RDWR, 0666);
    SharedData_odom* sharedData_odom = static_cast<SharedData_odom*>(mmap(nullptr, sizeof(SharedData_odom), PROT_READ | PROT_WRITE, MAP_SHARED, shm_id_odom, 0));

    //creat semaphore & initialize for scanning
	sem_t *semaphore_scan_write=sem_open(semaphore_name_scan_producer,  0);
    if(semaphore_scan_write==SEM_FAILED)
	{
		perror("Semaphore -scan- konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	sem_t *semaphore_scan_read=sem_open(semaphore_name_scan_consumer,  0);
    if(semaphore_scan_write==SEM_FAILED)
	{
		perror("Semaphore -scan- konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

    //creat semaphore & initialize for odometry
    	sem_t *semaphore_odom_write=sem_open(semaphore_name_odom_producer,  0);
    if(semaphore_odom_write==SEM_FAILED)
	{
		perror("Semaphore -odom- konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

	sem_t *semaphore_odom_read=sem_open(semaphore_name_odom_consumer,  0);
    if(semaphore_odom_read==SEM_FAILED)
	{
		perror("Semaphore -odom- konnte nicht erzeugt werden");
		exit(EXIT_FAILURE);
	}

    
   
    for(;;)
    {
    // Wait for the semaphore to be posted by the scanning-producer
    sem_wait(semaphore_scan_write);
    //copy data from shm into "msg_from_shm"
    strcpy(msg_from_shm_lidar, sharedData_scan->message);
    //cout << "Consumer received SCANNING-Data: " << msg_from_shm_lidar<< endl;
    sem_post(semaphore_scan_read);


    //Wait for the semaphore to be posted by the odom-producer
    sem_wait(semaphore_odom_write);

    strcpy(msg_from_shm_odom, sharedData_odom->message); 

    //cout << "Consumer received ODOMETRY-Data: " << msg_from_shm_odom<< endl;

    sem_post(semaphore_odom_read);     //->Alles wieder einkommentieren wernn odom miteingebunden werden muss (sem wait bis sem post)

    
    //datenverarbeitung_________________________________
    //Auftrennen des Strings beim "Komma" & einordnen der Werte in ein flaot Array, jedes Feld je ein Wert
    //__________________________________________________

    double Zahlenarray[360];
    string Komma=", ";
    size_t PosKomma=0;

    string str_msg_from_shm="";
    string temp1="";
    string str_msg_from_shm_temp="";
    double temp2=0.0;
    str_msg_from_shm=msg_from_shm_lidar;
    str_msg_from_shm_temp=str_msg_from_shm;     //für Kommasuche

    int Wertezähler=1;

    cout<<"CuttedMessage_LidarScan"<<str_msg_from_shm<<endl;
    //cout<<"CuttesMessage_LidarScan"<<str_msg_from_shm<<endl;

    while(true)
    {
        if(str_msg_from_shm_temp.find(Komma)==std::string::npos)
        {
            Wertezähler++;      //da nach dem letzten Wert kein Komma mehr kommt
            cout<<"Keine Kommas mehr gefunden bei Wert "<<Wertezähler<<endl;
            break;
        } 

        PosKomma = str_msg_from_shm_temp.find(Komma);
        str_msg_from_shm_temp.erase(0, PosKomma + 2);
        Wertezähler++;
    }

    if(Wertezähler>=359)
    {

    for(int i=0; i<360; i++)
    {
        //delten der ausgesuchten Daten aus dem String
        PosKomma = str_msg_from_shm.find(Komma);

        temp1=str_msg_from_shm.substr(0, PosKomma);     //Substring von 0 bis pos komma
        //cout<<temp1<<endl;
        //cout<<temp1<<" , ";
        temp2=stod(temp1);      //KOnvertieren des Strings in eine double Variable mit "std::stod"

        if (temp2==0)
        {
            temp2=10.0;  //Unendliche distanzen werden mit 0 angegeben -> zu 10 abändern
        }
        Zahlenarray[i]=temp2;

        //Löschen der übertragenden Zeichen
        str_msg_from_shm.erase(0, PosKomma + 2);
        //cout<<str_msg_from_shm<<endl;
    }


    for(int i=0; i<360; i++)
    {
        //cout<<endePos<<endl;
        cout<<Zahlenarray[i]<<";";
 
    }

    string msg_cmdvel_string = Kantenerkennung(Zahlenarray);        //cmd_vel wird von Kantenerkennung erzeugt und an mst_cmdvel_string übergeben
    msg_cmdvel=msg_cmdvel_string.c_str();
    //send comand to Turtlebot
    Start_TCP_IP_Connection();
	send(client_id, msg_cmdvel, strlen(msg_cmdvel), 0);
    close(client_id);

    cout<<"TurtloebotMessage: "<<msg_cmdvel<<endl;

    //Fahrt um ROhr
    void Kreisfahrt();


    

    //Datenauswertung
    }
    
    }

    return 0;
}

void Start_TCP_IP_Connection()
{
	int status;
	struct sockaddr_in serv_addr;

    //if ((client_id = socket(AF_INET, SOCK_STREAM, 0)) < 0) {

	if ((client_id = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {			//socket creates connection and defines the client_id
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
		= connect(client_id, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		printf("\nConnection Failed \n");
		return;
	}

}



string Kantenerkennung(double Zahlenarray[360])
{
    //Hier: Erkennung der Kanten

    //Datenauswertung: Wenn Leadar-Sensor keinen ABstand detektiert, dann gibt der Bot eine 0 zurück

    double max_Entfernung=0.50;     //AUf 3m hochsetzen!!
    double arr_KanteLinks[2];       //Pos 1: Abstand zur Kante, Pos 2: Winkel zur Kante
    double arr_KanteRechts[2];       //Pos 1: Abstand zur Kante, Pos 2: Winkel zur Kante
    double arr_EckeRechts[2];
    double arr_EckeLinks[2];

    double arr_Objekt_KanteLinks[2];
    double arr_Objekt_KanteRechts[2];

    //Erkennung des Rohrs
    double Platzhalter_aktueller_Wert=0.0;
    double Platzhalter_vorheriger_Wert=Zahlenarray[329];
    int i=350;
    int condition=0;

    double startPosition_odom_x=0.0;
    double startPosition_odom_y=0.0;

    while (true)
    {
        Platzhalter_aktueller_Wert=Zahlenarray[i];

        //cout<<Platzhalter_vorheriger_Wert<<" , "<<Platzhalter_aktueller_Wert<<endl;

        //Code zur Kantenerkennung
        if((Platzhalter_vorheriger_Wert>9.0) && (Platzhalter_aktueller_Wert<9.0) && (condition==0))       //Scan im Uhrzeigersinn: DAnn Linke Kante des Rohrs
        {
        arr_Objekt_KanteLinks[0]=Zahlenarray[i];            //ABstand auf pos.0
        arr_Objekt_KanteLinks[1]=i;                         //Winkel auf pos. 1
        condition=1;
        }


        if((Platzhalter_vorheriger_Wert<9) && (Platzhalter_aktueller_Wert>9)&&(condition==1))       //Scan im Uhrzeigersinn: DAnn Linke Kante des Rohrs
        {
        arr_Objekt_KanteRechts[0]=Zahlenarray[i];
        arr_Objekt_KanteRechts[1]=i;
        break;
        }

        //-10grad bis 0 grad überprüfen
        if(i==359)
        {
            cout<<"359"<<endl;
            i=0;
        }
 
        //0grad bis 10 grad überprüfen
        if(i==11)
        {
            cout<<"Schleife gestoppt, nicht beide Kanten gefunden!!"<<endl;
            break;
        }

        Platzhalter_vorheriger_Wert=Platzhalter_aktueller_Wert;
        cout<<i<<endl;
        i++;

        
    }
        

    //Mitte des ROhrs:
    int Winkel_Zentrum=0;
    int Winkel_links=0;
    int Winkel_rechts=0;

    Winkel_links=arr_Objekt_KanteLinks[1];
    Winkel_rechts=arr_Objekt_KanteRechts[1];

    if(Winkel_links>10)
    {
        Winkel_links-=360;
    }

    if(Winkel_rechts>10)
    {
        Winkel_rechts-=360;
    }

    Winkel_Zentrum=(Winkel_links+Winkel_rechts)/2;

    cout<<"Der Winkel der Mitte des Targets ist "<<Winkel_Zentrum<<endl;


    //Ausgabe der Kantenwerte
    for(int i=0; i<2;i++)
    {
        cout<<"Kante Links "<<arr_KanteLinks[i]<<endl;
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Ecke Links "<<arr_EckeLinks[i]<<endl;
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Kante Rechts "<<arr_KanteRechts[i]<<endl;
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Ecke Rechts "<<arr_EckeRechts[i]<<endl;
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Rechte Kante des Objekts "<<arr_Objekt_KanteLinks[i]<<endl;
    }

        for(int i=0; i<2;i++)
    {
        cout<<"Linke Kante des Objekts"<<arr_Objekt_KanteRechts[i]<<endl;
    }

    
     //Berechnung Winkelgeschwindikeit
    double k_angular=0.020;           //Alpha ist 1, beta ist 3, Max gEschwindingkeiten =0,07 für linear, 1 für angular
    double k_linear = 0.50;
    double linear=0.0;
    double angular=0.0;

    angular=k_angular*Winkel_Zentrum;           //Hier eventuell Vorzeichen anpassen (oder bei Faktor 8), je nachdem ob er sich im oder gegen den Uhrzeigersinn dreht

    char msg_cmdvel[50];

    if((Winkel_Zentrum<=1)&&(Winkel_Zentrum>=-1))
    {
        linear=k_linear*Zahlenarray[0];

        if(Zahlenarray[0]<Abstand_b)
        {
            cout<<"Turtlebot steht vor dem Ziel!"<<endl;
            linear=0;
        }
    }
    else
    {
        linear=0;
    }

    // linear=0;
    // angular=0;

    //built message for turtlebot
    sprintf(msg_cmdvel,R"---(---START---{"linear": %.2f, "angular": %.2f}___END___)---", linear, angular);

    return msg_cmdvel;

}

// void Kreisfahrt()
// {
//     int Sollwinkel=0;
//     int IstWinkel=0;

//     //90° ausrichtung

//     FehlermitAbsicht

// }