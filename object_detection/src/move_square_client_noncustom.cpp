#include "ros/ros.h"
#include "std_srvs/Empty.h"  

int main(int argc,char **argv){
    ros::init(argc,argv,"service_client");/*Initialiserer node med navn*/
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/ 

    ros::ServiceClient moveBotClient = n.serviceClient<std_srvs::Empty>("/my_service"); 
    std_srvs::Empty moveSquare;
    //deleteModel.request.model_name = "turtlebot3_burger"; //Sette model_name til turtlebot3_burger, er et nødvendig argument i denne servicen.
    moveBotClient.call(moveSquare); //Kaller service
     
    return 0;
}