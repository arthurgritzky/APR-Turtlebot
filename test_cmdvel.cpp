#include <iostream>

using namespace std;


int main()
{
    double linear=0.5;
    double angular=0.0;

    //char* char_linear="";
    //char* char_angular="";

    //original message: ---START---{"linear": 0.0, "angular": 0.00}END
    char* msg_part1=R"---(---START---{"linear": )---";
    char* msg_part2=R"---(, "angular": )---";
    char* msg_part3=R"---(}END)---";

    char msg_cmdvel[100];

    //built message for turtlebot
    // sprintf(char_linear, "%.2f", linear);
    // sprintf(char_angular, "%.2f", angular);

    // strcat(msg_cmdvel, msg_part1);
    // strcat(msg_cmdvel, char_linear);
    // strcat(msg_cmdvel, msg_part2);
    // strcat(msg_cmdvel, char_angular);
    // strcat(msg_cmdvel, msg_part3);

    sprintf(msg_cmdvel,R"---(---START---{"linear": %.2f, "angular": %.2f}END)---", linear, angular);
    //msg_cmdvel=msg_part1+char_linear+msg_part2+char_angular+msg_part3;

    cout<<msg_cmdvel<<endl;

    //cout<<2<<endl;

    return 0;
}