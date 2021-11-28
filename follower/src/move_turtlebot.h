#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h" /*Må ha for å ha datatype string*/
#include "geometry_msgs/Twist.h" 

/*Initialiserer variabler */

class ObjectAvoidance{
    private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    geometry_msgs::Twist move;
    float linx = 4; float liny = 0; float linz = 0;
    float angx = 0; float angy = 0; float angz = 0.5;
    float min_dist = 1;

    public:
    ObjectAvoidance();
    void control();
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

ObjectAvoidance::ObjectAvoidance(){
    pub = n.advertise<geometry_msgs::Twist>("tb3_0/cmd_vel",100);
    sub = n.subscribe<sensor_msgs::LaserScan>("tb3_0/scan",10,&ObjectAvoidance::callback,this); 
};

void ObjectAvoidance::control(){
        move.linear.x = linx;
        move.linear.y = liny;
        move.linear.z = linz;
        
        move.angular.x = angx;
        move.angular.y = angy;
        move.angular.z = angz;
        ROS_INFO("Hello %f",move.angular.z);
        pub.publish(move);
};

void ObjectAvoidance::callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //Angle 0 til robot er rett forran robot, burde ha slik at 30 grader forran robot sjekkes? Eller bare rett forran.
    //ROS_INFO("Front:%f",msg->ranges[0]);
    //Når det er objekt rett forran robot, snu
    if(msg->ranges[0] < min_dist){
        linx = -1; 
    }
    if(msg->ranges[180] < min_dist){
        linx = 1;
    }else{
            float linx = 4; float angz = 0.5;
    }

};