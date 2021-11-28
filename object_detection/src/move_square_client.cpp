#include "ros/ros.h"
#include "object_detection/custom_service_msg.h"    


using namespace std;



using namespace std;

int main(int argc,char **argv){
    ros::init(argc,argv,"service_client");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/ 
    
    //if(argc!=3){
      //  return 1;
    //}
    //ros::service::waitForService("gazebo_msgs/delete_model");
    //Lager service client, har navn og må ha command som brukes i cmd_line for å kalle service (gazebo/delete_model i dette tilfellet)
    ros::ServiceClient moveBotClient = n.serviceClient<object_detection::custom_service_msg>("/my_service"); 

    object_detection::custom_service_msg moveSquare;
    //deleteModel.request.model_name = "turtlebot3_burger"; //Sette model_name til turtlebot3_burger, er et nødvendig argument i denne servicen.
    

    moveSquare.request.radius = atoll(argv[1]);
    moveSquare.request.repetitions = atoll(argv[2]);
    
    if(moveBotClient.call(moveSquare)){
        ROS_INFO("Called");
    }
    else{
        ROS_INFO("Not called");
        return 1;
    }
    
    return 0;
}