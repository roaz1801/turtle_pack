#include "ros/ros.h"
#include "nav_msgs/Odometry.h" /*Må ha for å ha datatype string*/

//Callback gjør noe hver gang subscriber leser melding fra topic
void counterCallback(const nav_msgs::Odometry::ConstPtr msg){
    /*Printer ut meldingen, i dette tilfellet, counter */
    ROS_INFO("x position: %f",msg->pose.pose.position.x);
    ROS_INFO("y orientation: %f",msg->pose.pose.orientation.y);
    ROS_INFO("covariance: %f",msg->pose.covariance);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"odom_subscriber");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/


    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */

    /*Initialiserer subscriber objekt som subscriber til topic navn counter og med data type Int32*/ 
    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("odom",100,counterCallback); 
    //Callback indikerer at hver gang subscriber leser fra topic, kalles callback funksjon



    ros::spin();

    
    return 0;
}