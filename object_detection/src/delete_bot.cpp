#include "ros/ros.h"
#include "gazebo_msgs/DeleteModel.h" //M"ha med for å definere service.

using namespace std;


int main(int argc,char **argv){
    ros::init(argc,argv,"service_client");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/ 
    
    //ros::service::waitForService("gazebo_msgs/delete_model");
    //Lager service client, har navn og må ha command som brukes i cmd_line for å kalle service (gazebo/delete_model i dette tilfellet)
    ros::ServiceClient deleteModelClient = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model"); 
    gazebo_msgs::DeleteModel deleteModel;
    deleteModel.request.model_name = "turtlebot3_burger"; //Sette model_name til turtlebot3_burger, er et nødvendig argument i denne servicen.
    deleteModelClient.call(deleteModel); //Kaller service
     
    return 0;
}