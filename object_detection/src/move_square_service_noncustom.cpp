#include "ros/ros.h"
#include "std_srvs/Empty.h"  
using namespace std;

bool callback(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res){
    return true;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"service_move_bot");/*Initialiserer node med navn*/
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("my_service",callback); 
    ros::spin();
    return 0;
}