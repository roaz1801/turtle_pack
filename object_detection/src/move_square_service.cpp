#include "ros/ros.h"
//#include "std_srvs/Empty.h"  
#include "object_detection/custom_service_msg.h" //For custom meldinger 
#include "movebot.h" //Importerer klasse

using namespace std;

//For custom meldinger bruker vi f.eks object_detection::custom_service_msg
bool callback(object_detection::custom_service_msg::Request &req, object_detection::custom_service_msg::Request &res){
    MoveBot object;
    //Ser at argumentene i move_square har req. forran seg, kommer fra custom melding og disse gis i client.
    object.move_square(req.radius,req.repetitions);
    return true;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"service_move_bot");/*Initialiserer node med navn*/
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("my_service",callback); 
    ros::spin();
    return 0;
}


