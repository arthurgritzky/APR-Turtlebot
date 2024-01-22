//g++ Testfile_Mathematik.cpp -o testfile.o -lrt

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>

const char* shared_memory_name_odom = "/shared_memory_odom";
int client_id;
int Status_Kreisfahrt=0;        //0 für Fahrt zu Objekt, 1 zu fahrt Kreis P1, 2 zu Fahrt Kreis P2, 3 zu Fahrt Kreis P3 ....

using namespace std;
string DataExtract(string msg_String);
void Kantenerkennung(double Zahlenarray[360]);
void Kreisberechnung();
void Kreisfahrt(double X, double Y, double Euler);

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


int main()
{

    string extrahierteDaten="";
    extrahierteDaten= R"---(---START---{"header": {"seq": 4473, "stamp": {"secs": 1682592065, "nsecs": 686887475}, "frame_id": "base_scan"}, "angle_min": 0.0, "angle_max": 6.2657318115234375, "angle_increment": 0.01745329238474369, "time_increment": 0.0005592841189354658, "scan_time": 0.20134228467941284, "range_min": 0.11999999731779099, "range_max": 3.5, "ranges": [0.0, 0.0, 0.0, 0.0, 2.0, 3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.45399999618530273, 0.45399999618530273, 0.4339999854564667, 0.42399999499320984, 0.421999990940094, 0.4169999957084656, 0.41600000858306885, 0.41600000858306885, 0.41499999165534973, 0.41499999165534973, 0.414000004529953, 0.414000004529953, 0.41499999165534973, 0.41600000858306885, 0.4169999957084656, 0.4169999957084656, 0.4180000126361847, 0.4189999997615814, 0.41999998688697815, 0.42100000381469727, 0.42100000381469727, 0.4230000078678131, 0.4259999990463257, 0.4300000071525574, 0.43799999356269836, 0.4480000138282776, 0.453000009059906, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.440999984741211, 3.494999885559082, 3.5280001163482666, 3.5969998836517334, 3.681999921798706, 3.746000051498413, 3.8350000381469727, 3.984999895095825, 4.048999786376953, 4.0279998779296875, 4.1539998054504395, 4.159999847412109, 4.1620001792907715, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.574999988079071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.615999937057495, 3.622999906539917, 3.6010000705718994, 3.759000062942505, 3.622999906539917, 3.6019999980926514, 3.5840001106262207, 3.5980000495910645, 3.9100000858306885, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.192999839782715, 0.0, 0.0, 0.0, 1.7200000286102295, 1.7549999952316284, 1.7319999933242798, 0.7080000042915344, 0.7080000042915344, 0.6539999842643738, 0.6520000100135803, 0.6389999985694885, 0.6069999933242798, 0.5950000286102295, 0.5839999914169312, 0.5559999942779541, 0.5479999780654907, 0.5389999747276306, 0.5289999842643738, 0.515999972820282, 0.49399998784065247, 0.49399998784065247, 0.4909999966621399, 0.48500001430511475, 0.4830000102519989, 0.4860000014305115, 0.4869999885559082, 0.4860000014305115, 0.4830000102519989, 0.47999998927116394, 0.4790000021457672, 0.47999998927116394, 0.4830000102519989, 0.4860000014305115, 0.49000000953674316, 0.49399998784065247, 0.49900001287460327, 0.5059999823570251, 0.5130000114440918, 0.5220000147819519, 0.6899999976158142, 0.9269999861717224, 0.9269999861717224, 0.9729999899864197, 0.9890000224113464, 0.9980000257492065, 0.9039999842643738, 1.5429999828338623, 1.3899999856948853, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.2149999141693115, 2.188999891281128, 2.180999994277954, 2.1649999618530273, 2.1679999828338623, 2.2820000648498535, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2309999465942383, 1.2630000114440918, 1.218999981880188, 1.2599999904632568, 0.0, 0.0, 1.2319999933242798, 1.2549999952316284, 1.225000023841858, 1.2359999418258667, 1.2760000228881836, 1.2979999780654907, 1.2230000495910645, 1.3990000486373901, 1.2799999713897705, 1.2979999780654907, 1.4470000267028809, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4210000038146973, 1.3940000534057617, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6039999723434448, 0.6039999723434448, 0.593999981880188, 0.5690000057220459, 0.5580000281333923, 0.5490000247955322, 0.5419999957084656, 0.5350000262260437, 0.5320000052452087, 0.5270000100135803, 0.5220000147819519, 0.5180000066757202, 0.5120000243186951, 0.5080000162124634, 0.5049999952316284, 0.503000020980835, 0.5009999871253967, 0.5, 0.5009999871253967, 0.5040000081062317, 0.5070000290870667, 0.5120000243186951, 0.5189999938011169, 0.5289999842643738, 0.5410000085830688, 0.0, 1.1349999904632568, 1.1330000162124634, 1.1449999809265137, 1.1130000352859497, 1.1430000066757202, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9959999918937683, 1.0010000467300415, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.257999897003174, 3.2269999980926514, 3.1600000858306885, 3.0929999351501465, 3.072000026702881, 2.937000036239624, 2.890000104904175, 2.943000078201294, 2.8540000915527344, 2.7869999408721924, 2.796999931335449, 2.7850000858306885, 2.73799991607666, 2.687999963760376, 2.640000104904175], "intensities": [181.0, 152.0, 0.0, 0.0, 35.0, 0.0, 58.0, 39.0, 60.0, 79.0, 88.0, 95.0, 112.0, 254.0, 436.0, 527.0, 798.0, 1124.0, 1525.0, 2019.0, 2493.0, 3182.0, 3819.0, 4322.0, 4081.0, 4335.0, 5340.0, 3904.0, 3788.0, 3107.0, 2259.0, 1778.0, 1443.0, 1081.0, 764.0, 506.0, 322.0, 261.0, 107.0, 109.0, 87.0, 75.0, 63.0, 42.0, 35.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 71.0, 0.0, 377.0, 368.0, 377.0, 355.0, 360.0, 341.0, 327.0, 308.0, 301.0, 302.0, 349.0, 327.0, 259.0, 40.0, 0.0, 0.0, 61.0, 0.0, 0.0, 0.0, 0.0, 837.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 377.0, 383.0, 402.0, 418.0, 464.0, 501.0, 543.0, 521.0, 391.0, 426.0, 453.0, 433.0, 451.0, 416.0, 448.0, 426.0, 446.0, 430.0, 422.0, 425.0, 120.0, 651.0, 607.0, 1508.0, 1631.0, 2566.0, 3606.0, 2670.0, 3287.0, 2073.0, 2986.0, 2833.0, 3663.0, 3161.0, 3082.0, 3372.0, 3798.0, 4410.0, 3561.0, 4629.0, 3971.0, 3524.0, 4301.0, 3983.0, 4531.0, 4404.0, 4384.0, 4059.0, 4560.0, 3697.0, 3900.0, 4637.0, 4126.0, 4130.0, 4098.0, 4238.0, 115.0, 312.0, 277.0, 300.0, 303.0, 122.0, 118.0, 836.0, 494.0, 0.0, 0.0, 0.0, 0.0, 48.0, 0.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 282.0, 378.0, 391.0, 285.0, 415.0, 146.0, 87.0, 78.0, 97.0, 78.0, 0.0, 43.0, 1764.0, 1937.0, 584.0, 372.0, 0.0, 0.0, 522.0, 587.0, 641.0, 340.0, 374.0, 253.0, 241.0, 129.0, 125.0, 118.0, 115.0, 0.0, 0.0, 0.0, 64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 112.0, 95.0, 60.0, 39.0, 55.0, 0.0, 59.0, 55.0, 806.0, 1215.0, 51.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 88.0, 47.0, 32.0, 32.0, 32.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1210.0, 3647.0, 3362.0, 3832.0, 3793.0, 3913.0, 4161.0, 3983.0, 4026.0, 4352.0, 4508.0, 4554.0, 4355.0, 4509.0, 4354.0, 4462.0, 5350.0, 4261.0, 4501.0, 4192.0, 4817.0, 4483.0, 4458.0, 4929.0, 3407.0, 0.0, 1730.0, 1788.0, 1756.0, 1722.0, 1508.0, 71.0, 36.0, 37.0, 40.0, 49.0, 120.0, 3275.0, 546.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 312.0, 134.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 56.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.0, 41.0, 0.0, 278.0, 303.0, 295.0, 301.0, 304.0, 208.0, 211.0, 312.0, 218.0, 221.0, 224.0, 216.0, 229.0, 245.0, 248.0]}___END___)---";
    string msg_from_shm="";

    cout<<extrahierteDaten<<endl;



    //Kreisfahrt();


    //Daten aus shared Memory auslesen
    //int shm_id_odom = shm_open(shared_memory_name_odom, O_RDWR, 0666);
    //SharedData_odom* sharedData_odom = static_cast<SharedData_odom*>(mmap(nullptr, sizeof(SharedData_odom), PROT_READ | PROT_WRITE, MAP_SHARED, shm_id_odom, 0));

    msg_from_shm = DataExtract(extrahierteDaten);


//for(;;)
  //  {
    //cout<<"FEhlerdebugging_1"<<endl;

    // Wait for the semaphore to be posted by the scanning-producer
    //sem_wait(semaphore_scan_write);
    //copy data from shm into "msg_from_shm"
    //strcpy(msg_from_shm, sharedData_scan->message);

    //cout << "Consumer received SCANNING-Data: " << msg_from_shm<< endl;

    //sem_post(semaphore_scan_read);


    // Wait for the semaphore to be posted by the odom-producer
    // sem_wait(semaphore_odom_write);

    // strcpy(msg_from_shm_odom, sharedData_odom->message); 

    // cout << "Consumer received ODOMETRY-Data: " << msg_from_shm_odom<< endl;

    // sem_post(semaphore_odom_read);     ->Alles wieder einkommentieren wernn odom miteingebunden werden muss (sem wait bis sem post)

    
    //datenverarbeitung_________________________________
    //Auftrennen des Strings beim "Komma" & einordnen der Werte in ein flaot Array, jedes Feld je ein Wert
    
    double Zahlenarray[360];
    string Komma=", ";
    size_t PosKomma=0;

    string str_msg_from_shm="";
    string temp1="";
    double temp2=0.0;
    str_msg_from_shm=msg_from_shm;

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

    Kantenerkennung(Zahlenarray);

    //Datenauswertung



    //}
    return 0;
}


string DataExtract(string msg_String)
{
	//Filtern der Message nach den Messwerten
    string startWort = R"---("ranges": [)---";
    string endeWort = R"---(], "intensities")---";

    // Die Positionen der markanten Wörter im String finden
    size_t startPos = msg_String.find(startWort);
    size_t endePos = msg_String.find(endeWort);
    string extrahierteDaten;						//genau eine Nachricht!
	
	extrahierteDaten="";

    // Überprüfen, ob beide Wörter im String gefunden wurden
    if (startPos<endePos && startPos != string::npos && endePos != string::npos) {
        // Die Daten zwischen den markanten Wörtern extrahieren
        extrahierteDaten = msg_String.substr(startPos + startWort.length(), endePos - startPos - startWort.length());

		//cout<<extrahierteDaten<<endl;

        // Die extrahierten Daten ausgeben
        //cout << "Extrahierte Daten: " << extrahierteDaten << endl;
    } else {
        cout << "Markante Worte nicht gefunden." << endl;
		extrahierteDaten="Vorherige Nachricht war nicht vollständig!";
		//cout<<msg_String<<endl;
    }

	return extrahierteDaten;
}


void Kantenerkennung(double Zahlenarray[360])
{
    //Hier: Erkennung der Kanten

    //Datenauswertung: Wenn Leadar-Sensor keinen ABstand detektiert, dann gibt der Bot eine 0 zurück

    double max_Entfernung=0.30;     //AUf 3m hochsetzen!!
    double arr_KanteLinks[2];       //Pos 1: Abstand zur Kante, Pos 2: Winkel zur Kante
    double arr_KanteRechts[2];       //Pos 1: Abstand zur Kante, Pos 2: Winkel zur Kante
    double arr_EckeRechts[2];
    double arr_EckeLinks[2];

    double arr_Objekt_KanteLinks[2];
    double arr_Objekt_KanteRechts[2];

    //Erkennung des Rohrs
    double Platzhalter_aktueller_Wert=0.0;
    double Platzhalter_vorheriger_Wert=Zahlenarray[329];
    int i=330;
    int condition=0;
    while (true)
    {
        Platzhalter_aktueller_Wert=Zahlenarray[i];

        cout<<Platzhalter_vorheriger_Wert<<" , "<<Platzhalter_aktueller_Wert<<endl;

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

        //-30grad bis 0 grad überprüfen
        if(i==359)
        {
            cout<<"359"<<endl;
            i=0;
        }

        //0grad bis 30 grad überprüfen
        if(i==31)
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

    if(Winkel_links>30)
    {
        Winkel_links-=360;
    }

    if(Winkel_rechts>30)
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
    double k_angular=8.0;
    double k_linear = 3.0;
    double linear=0.0;
    double angular=0.0;

    angular=k_angular*Winkel_Zentrum;           //Hier eventuell Vorzeichen anpassen (oder bei Faktor 8), je nachdem ob er sich im oder gegen den Uhrzeigersinn dreht

    char msg_cmdvel[50];

    if(angular==0)
    {
        linear=k_linear*Zahlenarray[0];

        if(Zahlenarray[0]<0.3)
        {
            cout<<"Turtlebot steht vor dem Ziel!"<<endl;
            linear=0;
        }
    }
    else
    {
        linear=0;
    }

    //built message for turtlebot
    sprintf(msg_cmdvel,R"---(---START---{"linear": %.2f, "angular": %.2f}___END___)---", linear, angular);

    // Start_TCP_IP_Connection();
	// send(client_id, msg_cmdvel, strlen(msg_cmdvel), 0);
    // close(client_id);

    cout<<msg_cmdvel<<endl;

}


void Kreisberechnung()
{
    //Wichtig:    int Status_Kreisfahrt=0 global anlegen 
    struct P1_Koordinaten
    {
        double x_koord=2.0;
        double y_koord=2.0;

        double b=-45.0;       //Dehung um die eigene Achse in Grad!
    };

    struct P2_Koordinaten
    {
        double x_koord=4.0;
        double y_koord=0.0;

        double b=90.0;
    };
        
    struct P3_Koordinaten
    {
        double x_koord=-2.0;
        double y_koord=2.0;

        double b=90.0;
    };

    struct P4_Koordinaten
    {
        double x_koord=0.0;     //STartwerte Kreisbahn einlesen
        double y_koord=0.0;     //Startwerte kreisbahn einlesen

        double b=-45.0;
    };

    //aus Struct objekte machen
    struct P1_Koordinaten P1;
    struct P2_Koordinaten P2;
    struct P3_Koordinaten P3;
    struct P4_Koordinaten P4;

    if(Status_Kreisfahrt==1)
    {
        Kreisfahrt(P1.x_koord, P1.y_koord, P1.b);
    }

    if(Status_Kreisfahrt==2)
    {
        Kreisfahrt(P2.x_koord, P2.y_koord, P2.b);
    }

    if(Status_Kreisfahrt==3)
    {
        Kreisfahrt(P3.x_koord, P3.y_koord, P3.b);
    }

    if(Status_Kreisfahrt==4)
    {
        Kreisfahrt(P4.x_koord, P4.y_koord, P4.b);
    }

}

void Kreisfahrt(double X, double Y, double Euler)
{
struct SharedData_odom Odometriedaten;

double dbl_aktuellePosition_x=0.0;
double dbl_aktuellePosition_y=0.0;
double dbl_Eulerwinkel_Z=0.0;

double delta_x=0.0;
double delta_y=0.0;
double roh=0.0;
double alpha = 0.0;
double beta = 0.0;

double kp=0.0;
double k_alpha=0.0;
double k_beta=0.0;
double v=0.0;           //lineare geschwindigkeit
double w=0.0;           //angulare Geschwindigkeit bzw. Winkelgeschwindigkeit

dbl_aktuellePosition_x=stod(Odometriedaten.x_Wert_odom);
dbl_aktuellePosition_y=stod(Odometriedaten.y_Wert_odom);
dbl_Eulerwinkel_Z=stod(Odometriedaten.Eulerwinkel_Z);

//berechnung der Fahrt
delta_x=dbl_aktuellePosition_x-X;
delta_y=dbl_aktuellePosition_y-Y;

roh=sqrt((pow)(delta_x,2)+pow(delta_y,2));
alpha=-dbl_Eulerwinkel_Z+atan2(delta_y, delta_x);
beta=-dbl_Eulerwinkel_Z-alpha;

v=kp*roh;
w=k_alpha*alpha+k_beta*beta;


//Nachricht noch an Turtlebot ausgeben!
 
}

