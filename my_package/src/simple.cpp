#include "ros/ros.h" /*Må ha for å ha for å bruke ROS*/
#include "std_msgs/String.h" /*Må ha for å ha datatype string*/


int main(int argc,char **argv){
    ros::init(argc,argv,"simple"); /*Initialiserer node med navn*/
    ros::NodeHandle n; /*Håndterer node kommunikasjon*/

    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */

    int count = 0;
    while (ros::ok()){
        /*ROS_INFO er som en print/cout, men for ros noder */
        ROS_INFO("Help me Obi-Wan, you're my only hope, I've been waiting for %d years",count);
        loop_rate.sleep();
        ++count;

    }

    
    return 0;
}