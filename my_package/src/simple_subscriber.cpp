#include "ros/ros.h"
#include "std_msgs/Int32.h" /*Må ha for å ha datatype string*/

//Callback gjør noe hver gang subscriber leser melding fra topic
void counterCallback(const std_msgs::Int32::ConstPtr msg){
    /*Printer ut meldingen, i dette tilfellet, counter */
    ROS_INFO("Count: %d",msg->data);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"topic_subscriber");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/


    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */

    /*Initialiserer subscriber objekt som subscriber til topic navn counter og med data type Int32*/ 
    ros::Subscriber sub = n.subscribe<std_msgs::Int32>("counter",100,counterCallback); 
    //Callback indikerer at hver gang subscriber leser fra topic, kalles callback funksjon



    ros::spin();

    
    return 0;
}