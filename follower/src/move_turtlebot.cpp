#include "move_turtlebot.h" //Importerer klasse
using namespace std;

int main(int argc,char **argv){
    ros::init(argc,argv,"object_detector");/*Initialiserer node med navn*/
    
    ObjectAvoidance object;

    ros::Rate loop_rate(1); /*Bestemmer loop rate hastighet */
    

    while (ros::ok()){
        object.control();
        loop_rate.sleep();
        ros::spinOnce();
    }
  
    return 0;
}