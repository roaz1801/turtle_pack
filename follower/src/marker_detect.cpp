#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "fiducial_msgs/FiducialTransformArray.h"
using namespace std;


void counterCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr msg){
    /*Printer ut meldingen, i dette tilfellet, counter */
    //ROS_INFO("Laser scan: %f",msg->ranges[0]);
    int marker_id;
    
    if(msg->transforms.empty()){
        ROS_INFO("Hello");
    }else{
    marker_id = msg->transforms[0].fiducial_id;
        ROS_INFO("Id: %d",marker_id);
    }
    


    
    //Angle 0 til robot er rett forran robot, burde ha slik at 30 grader forran robot sjekkes? Eller bare rett forran.
    //update_time = msg->transforms.fiducial_id.transform;//.fiducial_id;
    //Når det er objekt rett forran robot, snu
   
}

int main(int argc,char **argv){
    ros::init(argc,argv,"marker_detect");/*Initialiserer node med navn*/
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<fiducial_msgs::FiducialTransformArray>("fiducial_transforms",10,counterCallback);
    fiducial_msgs::FiducialTransformArray h;

    if(n.hasParam("marker_id"))
    {
     ROS_INFO("Has param");
    }else{
     ROS_INFO("Does not have param");
    }


    ros::spin();
    return 0;
}