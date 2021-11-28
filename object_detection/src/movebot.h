#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 

/*Header fil slik at klasse kan brukes hvor som helst istedenfor å copy-paste den*/
class MoveBot{
    private:
    /*Initialiserer variabler */
    ros::NodeHandle n;/*Håndterer node kommunikasjon*/  
    ros::Publisher pub;
    float side; 
    int repetition;

    public:
    //Initialiserer metodene
    MoveBot();
    void move(float time,float linx,float angz);
    void stop();
    void move_square(float side,int repetition);   
};

//Constructor metode
MoveBot::MoveBot(){
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
};
void MoveBot::stop(){
    geometry_msgs::Twist move;
    move.linear.x = 0.0;
    move.angular.z= 0.0;
};

void MoveBot::move(float time,float linx=0.2, float angz=0.2){
    geometry_msgs::Twist move;
    move.linear.x = linx;
    move.angular.z = angz;

    /*Viktig å ha denne, når vi ikke skal publishe kontinuerlig.
    Er fordi publisher ikke er klar på første melding, og derfor vil den ikke ta først, så 
    vi publisher bare når den er klar*/ 
    while(pub.getNumSubscribers()<1){
    }
    pub.publish(move);
    sleep(time);
    //loop_rate.sleep();
    ROS_INFO("Hello %f",move.angular.z);
    this->stop();
};

void MoveBot::move_square(float side=0.2,int repetition = 1){
    int i= 0;int j= 0;
    float time_mag = side/0.2;
    while(j<repetition){
    while(i < 4){
        this->move(2.0*time_mag,0.2,0.0);
        this->move(4.0*time_mag,0.0,0.0);
        this->move(4.0*time_mag,0.0,0.2);
        this->move(0.1*time_mag,0.0,0.0);
        
        i+=1;
    }
    j+=1;
    }
};
