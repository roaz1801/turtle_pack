#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h" /*Må ha for å ha datatype string*/
#include "geometry_msgs/Twist.h" 

/*Initialiserer variabler */
float linx = 0.2; float liny = 0; float linz = 0;
float angx = 0; float angy = 0; float angz = 0;
float min_dist = 1;



using namespace std;
//Callback gjør noe hver gang subscriber leser melding fra topic
void counterCallback(const sensor_msgs::LaserScan::ConstPtr msg){
    /*Printer ut meldingen, i dette tilfellet, counter */
    //ROS_INFO("Laser scan: %f",msg->ranges[0]);

    

    //Angle 0 til robot er rett forran robot, burde ha slik at 30 grader forran robot sjekkes? Eller bare rett forran.
    ROS_INFO("Front:%f",msg->ranges[0]);
    //Når det er objekt rett forran robot, snu
    if(msg->ranges[0] < min_dist){
        linx = 0.1; angz = 1;
    }else{
        linx = 0.2; angz = 0;
    }
   
}

int main(int argc,char **argv){
    ros::init(argc,argv,"object_detector");/*Initialiserer node med navn*/
    
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/  


    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */
    

    /*Initialiserer subscriber objekt som subscriber til topic navn counter og med data type Int32*/ 
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("scan",10,counterCallback); 
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);

    while (ros::ok()){
        geometry_msgs::Twist move;
        move.linear.x = linx;
        move.angular.z = angz;
        ROS_INFO("%f",move.linear.x);
        pub.publish(move);
        //Callback indikerer at hver gang subscriber leser fra topic, kalles callback funksjon
        loop_rate.sleep();
        ros::spinOnce();
    }
  
    return 0;
}