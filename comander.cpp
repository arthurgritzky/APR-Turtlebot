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
#include <math.h>

#define semaphore_name_scan_producer "/semaphore_scan_producer"
#define semaphore_name_scan_consumer "/semaphore_scan_consumer"

#define semaphore_name_odom_producer  "/semaphore_odom_producer"
#define semaphore_name_odom_consumer  "/semaphore_odom_consumer"

#define PORT 9999
#define IP_Adresse "192.168.100.54"

using namespace std;

const char* shared_memory_name_scan = "/shared_memory_scan";
const char* shared_memory_name_odom = "/shared_memory_odom";


int client_id;
double max_Range_Lidar=2.0;             //set values in meter
float Abstand_c=0.4;                    //distance to the roll
int Status_Kreisfahrt=0;

float linear=0.00;                      //linear-parameter for Turtlebot-message
float angular=0.00;                     //angular-parameter for Turtlebot-message

float aktuelle_x_koord_odom=0.00;
float aktuelle_y_koord_odom=0.00;
float aktueller_eulerwinkel_odom=0.00;

float pos_x_vor_dem_Pol=0.0;            //coordinates, when turtlebot stands in front of the roll
float pos_y_vor_dem_Pol=0.0;            //coordinates, when turtlebot stands in front of the roll
float eulerwinkel_vor_dem_Pol=0.0;      //euler-angle, when turtlebot stands in front of the roll


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
void Kantenerkennung(double Zahlenarray[360]);
void Kreisberechnung();
void Kreisfahrt(double X_Koord, double Y_Koord, double Euler);


int main() {

    char msg_from_shm_lidar[10000];
    char msg_from_shm_odom[10000];
    const char* msg_cmdvel;

    int TurtlebotStatus=0;          //defines in whitch part of the path the turtlebot is

    // Open shared memorys (lidar-scan and odom)
    int shm_id = shm_open(shared_memory_name_scan, O_RDWR, 0666);
    SharedData_scan* sharedData_scan = static_cast<SharedData_scan*>(mmap(nullptr, sizeof(SharedData_scan), PROT_READ | PROT_WRITE, MAP_SHARED, shm_id, 0));

    int shm_id_odom = shm_open(shared_memory_name_odom, O_RDWR, 0666);
    SharedData_odom* sharedData_odom = static_cast<SharedData_odom*>(mmap(nullptr, sizeof(SharedData_odom), PROT_READ | PROT_WRITE, MAP_SHARED, shm_id_odom, 0));

    //creat semaphores (write&read) & initialize them for scanning
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

    //creat semaphores (write&read) & initialize them for odometry
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

   
    for(;;)         //loop to get the data, calculate the path, create the message for the turtlebot and send it to the bot
    {

        double Zahlenarray[360];
        string Komma=", ";
        size_t PosKomma=0;

        string str_msg_from_shm="";
        string temp1="";
        string str_msg_from_shm_temp="";
        double temp2=0.0;
        str_msg_from_shm=msg_from_shm_lidar;
        str_msg_from_shm_temp=str_msg_from_shm;     //für Kommasuche

        char msg_cmdvel[50];

        int Wertezähler=1;

        //get data from shared memory scan
        sem_wait(semaphore_scan_write);         // Wait for the semaphore to be posted by the scanning-producer
        strcpy(msg_from_shm_lidar, sharedData_scan->message);       //copy data from shm into "msg_from_shm"
        sem_post(semaphore_scan_read);         // post the Semaphore if the copy-process is finished


        //get data from shared memory odom
        sem_wait(semaphore_odom_write);     //Wait for the semaphore to be posted by the odom-producer
        strcpy(msg_from_shm_odom, sharedData_odom->message);        //copy data from shm into "msg_from_shm"
        aktuelle_x_koord_odom=atof(sharedData_odom->x_Wert_odom);       //get data from shm into "msg_from_shm"
        aktuelle_y_koord_odom=atof(sharedData_odom->y_Wert_odom);       //get data from shm into "msg_from_shm"
        aktueller_eulerwinkel_odom=atof(sharedData_odom->Eulerwinkel_Z);       //get data from shm into "msg_from_shm"
        sem_post(semaphore_odom_read);         // post the Semaphore if the copy-process is finished

        if(Status_Kreisfahrt==0)            //status_Kreisfaht==0 -> drive to the roll and stop in front of it
        {

            while(true)
            {
                if(str_msg_from_shm_temp.find(Komma)==std::string::npos)            //searching for commas
                {
                    Wertezähler++;
                    cout<<"Keine Kommas mehr in der Lidar-Message gefunden bei Wert "<<Wertezähler<<endl;
                    break;
                } 

                PosKomma = str_msg_from_shm_temp.find(Komma);
                str_msg_from_shm_temp.erase(0, PosKomma + 2);           //cut the commas und the empty space out of the message
                Wertezähler++;
            }

            if(Wertezähler>=359)
            {

                for(int i=0; i<360; i++)
                {
                    
                    PosKomma = str_msg_from_shm.find(Komma);            //searching for commas
                    temp1=str_msg_from_shm.substr(0, PosKomma);         //Substring von 0 bis pos komma
                    temp2=stod(temp1);

                    if (temp2==0)           //lidar-scanner set the value 0, if the scanned point is out of the (hardware) range of the scanner
                    {
                        temp2=10.0;         //to make it in the following part vissible, set all the values, whitch are out of range, to 10 (metres)
                    }

                    if(temp2>max_Range_Lidar)
                    {
                        temp2=10.0;         //set all the values, whitch are out over the defined max range of the lidar, to 10 (metres)
                                            //this minimize the errors if the lidar detacts objects behind the roll (chairs,...)
                    }        

                    Zahlenarray[i]=temp2;
                    str_msg_from_shm.erase(0, PosKomma + 2);            //delete the reden chars
                }

                cout<<"Status Kreisfahrt: -Fahrt zum Rohr- :"<<Status_Kreisfahrt<<endl;
                Kantenerkennung(Zahlenarray);        //cmd_vel wird von Kantenerkennung erzeugt und an mst_cmdvel_string übergeben
                cout<<"TurtloebotMessage: "<<msg_cmdvel<<endl;       
            }  

        }

        if(Status_Kreisfahrt>0)         //drive to the points along the roll
        {
            cout<<"Status Kreisfahrt: -Fahrt um das Rohr- "<<Status_Kreisfahrt<<endl;
            Kreisberechnung();          //calculate the route
            cout<<"TurtloebotMessage: "<<msg_cmdvel<<endl; 
        }

        //only comment in to stop the Turtlebot (then compile and execute again)!!________________________________________________________________________________
        // linear=0.00;
        // angular=0.00;
        //only comment in to stop the Turtlebot (then compile and execute again)!!________________________________________________________________________________
        

        //send the command to the turtlebot via TCP/IP-connection
        sprintf(msg_cmdvel,R"---(---START---{"linear": %.2f, "angular": %.2f}___END___)---", linear, angular);
        Start_TCP_IP_Connection();
        send(client_id, msg_cmdvel, strlen(msg_cmdvel), 0);    
        close(client_id);
        
    }

    return 0;
}

void Start_TCP_IP_Connection()
{
	int status;
	struct sockaddr_in serv_addr;

	if ((client_id = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {			//socket creates connection and defines the client_id
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
		= connect(client_id, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		printf("\nConnection Failed \n");
		return;
	}

}



void Kantenerkennung(double Zahlenarray[360])
{
    //detection of the "edges" from the roll

    double arr_KanteLinks[2];           //position 1: distance to the edge, position 2: angle to the edge
    double arr_KanteRechts[2];          //position 1: distance to the edge, position 2: angle to the edge
    double arr_EckeRechts[2];           //position 1: distance to the edge, position 2: angle to the edge
    double arr_EckeLinks[2];            //position 1: distance to the edge, position 2: angle to the edge

    double arr_Objekt_KanteLinks[2];   
    double arr_Objekt_KanteRechts[2];

    double Platzhalter_aktueller_Wert=0.0;
    double Platzhalter_vorheriger_Wert=Zahlenarray[329];
    int i=350;
    int condition=0;

    double startPosition_odom_x=0.0;
    double startPosition_odom_y=0.0;

    while (true)
    {
        Platzhalter_aktueller_Wert=Zahlenarray[i];

        //detect the edge:
        if((Platzhalter_vorheriger_Wert>9.0) && (Platzhalter_aktueller_Wert<9.0) && (condition==0)) 
        {
        arr_Objekt_KanteLinks[0]=Zahlenarray[i];            //distance to pos.0 of the array Kante Links
        arr_Objekt_KanteLinks[1]=i;                         //angle to pos.1 of the array Kante Links
        condition=1;
        }


        if((Platzhalter_vorheriger_Wert<9) && (Platzhalter_aktueller_Wert>9)&&(condition==1))
        {
        arr_Objekt_KanteRechts[0]=Zahlenarray[i];            //distance to pos.0 of the array Kante Links
        arr_Objekt_KanteRechts[1]=i;                         //angle to pos.1 of the array Kante Rechts
        break;
        }

        if(i==359)      //check the angle in front of the turtlebot from 350 to 359
        {
            cout<<"359"<<endl;
            i=0;
        }
 
        if(i==11)      //check the angle in front of the turtlebot from 0 to 10
        {
            cout<<"Schleife gestoppt, nicht beide Kanten gefunden!!"<<endl;
            break;
        }
        Platzhalter_vorheriger_Wert=Platzhalter_aktueller_Wert;         //set the current value to an placeholder for the next run in the loop

        i++;        //count up       
    }
        
    //calculate the middle of the roll
    int Winkel_Zentrum=0;
    int Winkel_links=0;
    int Winkel_rechts=0;

    Winkel_links=arr_Objekt_KanteLinks[1];          //angle in position 0 of the array KanteLinks
    Winkel_rechts=arr_Objekt_KanteRechts[1];        //angle in position 0 of the array KanteRechts

    if(Winkel_links>10)
    {
        Winkel_links-=360;          //we start by the value of 350, if we reach 360 we have to jump to 0 again, because there is no 361'th value
    }

    if(Winkel_rechts>10)
    {
        Winkel_rechts-=360;          //we start by the value of 350, if we reach 360 we have to jump to 0 again, because there is no 361'th value
    }

    Winkel_Zentrum=(Winkel_links+Winkel_rechts)/2;      //calculate the middle of the target
    cout<<"Der Winkel der Mitte des Targets ist "<<Winkel_Zentrum<<endl;        //print out the middle of the target

    for(int i=0; i<2;i++)
    {
        cout<<"Kante Links "<<arr_KanteLinks[i]<<endl;      //print out the distance and angle of the left edge of the start-box
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Ecke Links "<<arr_EckeLinks[i]<<endl;      //print out the distance and angle of the left corner of the start-box
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Kante Rechts "<<arr_KanteRechts[i]<<endl;      //print out the distance and angle of the right edge of the start-box
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Ecke Rechts "<<arr_EckeRechts[i]<<endl;      //print out the distance and angle of the right corner of the start-box
    }

    for(int i=0; i<2;i++)
    {
        cout<<"Rechte Kante des Objekts "<<arr_Objekt_KanteLinks[i]<<endl;      //print out the distance and angle of the right edge of the roll
    }

        for(int i=0; i<2;i++)
    {
        cout<<"Linke Kante des Objekts"<<arr_Objekt_KanteRechts[i]<<endl;      //print out the distance and angle of the left edge of the roll
    }

    
    //calculate the anglular velocity
    double k_angular=0.020;
    double k_linear = 0.50;
    char* Werteingabe;
    char msg_cmdvel[50];

    angular=k_angular*Winkel_Zentrum;

    if((Winkel_Zentrum<=1)&&(Winkel_Zentrum>=-1))           //drive while the middle of the target is between -1 and +1 degree infront of the turtlebot
    {
        linear=k_linear*Zahlenarray[0];             //calculate the linear velocity

        if(Zahlenarray[0]<Abstand_c)
        {
            cout<<"Turtlebot steht vor dem Ziel!"<<endl;

            pos_x_vor_dem_Pol=aktuelle_x_koord_odom;
            pos_y_vor_dem_Pol=aktuelle_y_koord_odom;
            eulerwinkel_vor_dem_Pol=aktueller_eulerwinkel_odom;

            cout<<"X-Position vor dem Pol: "<<pos_x_vor_dem_Pol<<endl;
            cout<<"Y-Position vor dem Pol: "<<pos_y_vor_dem_Pol<<endl;
            cout<<"Eulerwinkel vor dem Pol: "<<eulerwinkel_vor_dem_Pol<<endl;

            linear=0.00;
            angular=0.00;
            Status_Kreisfahrt++;
           
        }
    }
    else                //if the middle of the target is out of the scope from -1 degree to +1 degree, stop (linear velocity=0) and align to the target
    {
        linear=0;
    }

}


void Kreisberechnung()      //set the points to drive on along the target
{

    struct P1_Koordinaten       //set the points
    {
        double x_koord=pos_x_vor_dem_Pol+ Abstand_c;
        double y_koord=pos_y_vor_dem_Pol-Abstand_c;

        double b=0.785;
    };

    struct P2_Koordinaten
    {
        double x_koord=pos_x_vor_dem_Pol+2*Abstand_c;
        double y_koord=pos_y_vor_dem_Pol+0.0;

        double b=90.0*3.14/180;
    };
        
    struct P3_Koordinaten
    {
        double x_koord=pos_x_vor_dem_Pol+Abstand_c;
        double y_koord=pos_y_vor_dem_Pol+Abstand_c;

        double b=90.0*3.14/180;
    };

    struct P4_Koordinaten
    {
        double x_koord=pos_x_vor_dem_Pol;
        double y_koord=pos_y_vor_dem_Pol;

        double b=-45.0*3.14/180;
    };

    struct P5_Koordinaten
    {
        double x_koord=pos_x_vor_dem_Pol;
        double y_koord=pos_y_vor_dem_Pol;

        double b=-45.0*3.14/180;
    };

    struct P1_Koordinaten P1;
    struct P2_Koordinaten P2;
    struct P3_Koordinaten P3;
    struct P4_Koordinaten P4;
    struct P4_Koordinaten P5;

    if (Status_Kreisfahrt==1)
    {      
        Kreisfahrt(P1.x_koord, P1.y_koord, P1.b);           //executing the void function "Kreisfahrt" with the parameters of the Point one
    }

    if(Status_Kreisfahrt==2)
    {      
        Kreisfahrt(P2.x_koord, P2.y_koord, P2.b);           //executing the void function "Kreisfahrt" with the parameters of the Point two
    }

    if(Status_Kreisfahrt==4)
    {
        Kreisfahrt(P4.x_koord, P4.y_koord, P4.b);           //executing the void function "Kreisfahrt" with the parameters of the Point four
    }

    if(Status_Kreisfahrt==3)
    {
        Kreisfahrt(P3.x_koord, P3.y_koord, P3.b);           //executing the void function "Kreisfahrt" with the parameters of the Point three
    }

    if(Status_Kreisfahrt==5)
    {
        Kreisfahrt(P5.x_koord, P5.y_koord, P5.b);           //executing the void function "Kreisfahrt" with the parameters of the Point five
    }
}


void Kreisfahrt(double X_Koord, double Y_Koord, double Euler)       //calculate the route to drive along the roll
{
    double delta_x=0.0;
    double delta_y=0.0;
    double roh=0.0;
    double alpha = 0.0;
    double beta = 0.0;

    double kp=0.50;
    double k_alpha=0.40;
    double k_beta=-0.05;
    double v=0.0;
    double w=0.0;


    delta_x=X_Koord-aktuelle_x_koord_odom;          //calculate the delta from the target position and actual position in X
    delta_y=Y_Koord-aktuelle_y_koord_odom;          //calculate the delta from the target position and actual position in Y

    //calculate the parameters to drive along the points
    roh=sqrt((pow)(delta_x,2)+pow(delta_y,2));
    alpha=-aktueller_eulerwinkel_odom+atan2(delta_y, delta_x);
    beta=-aktueller_eulerwinkel_odom-alpha;                         

    v=kp*roh;
    //w=k_alpha*alpha+k_beta*beta;
    w=k_alpha*alpha;

    cout<<"Abstand X zum Anvisierten Punkt: "<<delta_x<<endl;
    cout<<"Abstand Y zum Anvisierten Punkt: "<<delta_y<<endl;
    cout<<"Aktueller Eulerwinkel: : "<<aktueller_eulerwinkel_odom<<endl;
    cout<<"Aktuell Geschwindigkeit Linear: "<<v<<endl;
    cout<<"Aktuell Geschwindigkeit Angular: "<<w<<endl;


    //align to the new given point
    linear=0;
    angular=w;


    //drive to Position 1 and 2
    if(Status_Kreisfahrt<=2)            
    {

        if(abs(alpha)<=0.05)        //if the actual angle to the target is smaller than 0.05rad, than stop rotating and approach the position with the calculated velocity v
        {
            linear=v;
            angular=0;              //if the actual angle of the target is bigger than 0.05rad, than stop to drive (linear=0) and aling again to the position
        }   
    }

    //drive to Position 3
    if(Status_Kreisfahrt==3)
    {
        delta_x=X_Koord-aktuelle_x_koord_odom;
        delta_y=Y_Koord-aktuelle_y_koord_odom;
        alpha=-aktueller_eulerwinkel_odom+atan2(delta_y, delta_x);

        roh=sqrt((pow)(delta_x,2)+pow(delta_y,2));
        v=kp*roh;

        if(0.03<alpha<=0.05)        //if the actual angle to the target is between the range from 0.03rad and 0.05rad, than stop rotating and approach the position with the calculated velocity v
        {
            linear=v;
            angular=0;
        }
    }

    //drive to Position 4
    if(Status_Kreisfahrt==4)
    {
        linear=0.0;
        delta_x=aktuelle_x_koord_odom-X_Koord;
        delta_y=aktuelle_y_koord_odom-Y_Koord;
        alpha=-aktueller_eulerwinkel_odom+atan2(delta_y, delta_x);

        roh=sqrt((pow)(delta_x,2)+pow(delta_y,2));
        v=kp*roh;
        w=-k_alpha*alpha;

        if(0.03<abs(alpha)<=0.05)        //if the actual angle to the target is between the range from 0.03rad and 0.05rad, than stop rotating and approach the position with the calculated velocity v
        {
            linear=v;
            angular=0;
        }
    }

    //drive to Position 5
    // if(Status_Kreisfahrt==5)
    // {
    //     linear=0.0;
    //     delta_x=aktuelle_x_koord_odom-X_Koord;
    //     delta_y=aktuelle_y_koord_odom-Y_Koord;
    //     alpha=-aktueller_eulerwinkel_odom+atan2(delta_y, delta_x);

    //     roh=sqrt((pow)(delta_x,2)+pow(delta_y,2));
    //     v=kp*roh;
    //     w=-k_alpha*alpha;

    //     if(0.03<abs(alpha)<=0.05)
    //     {
    //         linear=v;
    //         angular=0;
    //     }
    // }

    cout<<"Alpha: "<<alpha<<endl;    //print out the angle between the actual direction of the turtlebot and the target
    cout<<"Abstand Roh: "<<roh<<endl;     //print out the direct distance between the actual position of the turtlebot and the position of the target

    if(abs(roh)<=0.10)          //if the distance to the current position is reached near, stop and count up the status to drive to the next position
    {
        linear=0.00;
        angular=0.00;
        Status_Kreisfahrt++;    
    }
}