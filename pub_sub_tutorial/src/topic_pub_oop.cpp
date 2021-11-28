#include "ros/ros.h"
#include "geometry_msgs/Twist.h" /*Må ha for å ha datatype Twist fra geometry_msg pakke*/
//Dette er fordi cmd_vel node bruker denne typen data

/*Initialiserer variabler */

class Move{
    private:
    ros::NodeHandle n;
    ros::Publisher pub;
    geometry_msgs::Twist move;
    //ros::Rate loop_rate;

    public:
    Move();
    void control(float linx,float liny, float linz, float angx, float angy, float angz);
    
};

Move::Move(){
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    
}

void Move::control(float linx=0,float liny=0, float linz=0, float angx=0, float angy=0, float angz=0){
        move.linear.x = linx;
        move.linear.y = liny;
        move.linear.z = linz;
        
        move.angular.x = angx;
        move.angular.y = angy;
        move.angular.z = angz;
        ROS_INFO("Hello %f",move.linear.x);
        pub.publish(move);

}

int main(int argc,char **argv){
    ros::init(argc,argv,"control");/*Initialiserer node med navn*/
    Move object;
    
    ros::Rate loop_rate(1);

    while(ros::ok()){
        ros::spinOnce();
        object.control(2,0,0,0,0,2);
        loop_rate.sleep();

    }
    
    return 0;
}