#include "ros/ros.h"
#include "pub_sub_tutorial/Age.h" /*Må ha for å ha datatype Twist fra geometry_msg pakke*/
//Dette er fordi cmd_vel node bruker denne typen data


int years = 1998;
int months = 1;
int days = 18;   

int main(int argc,char **argv){
    ros::init(argc,argv,"age_pub");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/


    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */

    /*Initialiserer publiser til cmd_vel og data type Twist, siden cmd_vel bruker Twist*/ 
    ros::Publisher pub = n.advertise<pub_sub_tutorial::Age>("age",100); 
    //cmd_vel styret hjulene til robotten.

    int count = 0;
    while (ros::ok()){

        //Twist har 6 variabler, linear x,y,z og angular x,y,z.
        /*Trenger ikke kalle melding for msg, kan f.eks
         kalle det move som vi har gjort her*/
        pub_sub_tutorial::Age age;     
        age.years = years;
        age.days = days;
        age.months = months;
        ROS_INFO("%d",age.years);
        pub.publish(age);
        loop_rate.sleep(); /*Passer på at publish rate er sammenhengende med loop rate */

        ros::spinOnce();

        ++count;
    }
    return 0;
}