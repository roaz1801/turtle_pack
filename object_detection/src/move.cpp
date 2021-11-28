#include "ros/ros.h"
#include "geometry_msgs/Twist.h" /*Må ha for å ha datatype Twist fra geometry_msg pakke*/
//Dette er fordi cmd_vel node bruker denne typen data

/*Initialiserer variabler */
float linx = 2; float liny = 0; float linz = 0;
float angx = 0; float angy = 0; float angz = 2;

int main(int argc,char **argv){
    ros::init(argc,argv,"control");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/


    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */

    /*Initialiserer publiser til cmd_vel og data type Twist, siden cmd_vel bruker Twist*/ 
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100); 
    //cmd_vel styret hjulene til robotten.

    int count = 0;
    while (ros::ok()){

        //Twist har 6 variabler, linear x,y,z og angular x,y,z.
        /*Trenger ikke kalle melding for msg, kan f.eks
         kalle det move som vi har gjort her*/
        geometry_msgs::Twist move;

        move.linear.x = linx;
        move.linear.y = liny;
        move.linear.z = linz;
        
        move.angular.x = angx;
        move.angular.y = angy;
        move.angular.z = angz;

        ROS_INFO("%f",move.linear.x);
        pub.publish(move);
        loop_rate.sleep(); /*Passer på at publish rate er sammenhengende med loop rate */

        ros::spinOnce();

        ++count;
    }
    return 0;
}