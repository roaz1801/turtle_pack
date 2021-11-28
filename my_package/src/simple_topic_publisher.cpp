#include "ros/ros.h"
#include "std_msgs/Int32.h" /*Må ha for å ha datatype string*/


int main(int argc,char **argv){
    ros::init(argc,argv,"topic_publisher");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/


    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */

    /*Initialiserer publiser objekt med topic navn counter og data type Int32*/ 
    ros::Publisher pub = n.advertise<std_msgs::Int32>("counter",100); 

    int count = 0;
    while (ros::ok()){

        std_msgs::Int32 msg;
        msg.data = count; /*Setter melding data lik count */

        ROS_INFO("%d",msg);
        pub.publish(msg);
        loop_rate.sleep(); /*Passer på at publish rate er sammenhengende med loop rate */

        ros::spinOnce();

        ++count;

    }

    
    return 0;
}