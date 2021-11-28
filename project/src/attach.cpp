#include "ros/ros.h"
#include "gazebo_ros_link_attacher/Attach.h" //For custom meldinger 
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Quaternion.h" 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>

using namespace std;

int main(int argc,char **argv){
    ros::init(argc,argv,"spawn_marker");/*Initialiserer node med navn*/
    ros::NodeHandle n;

    //ROS variables
    geometry_msgs::Pose marker_pose;
    gazebo_ros_link_attacher::Attach attacher;
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion marker_quat;
    gazebo_msgs::SpawnModel marker;
    std_srvs::Empty pauseSrv;
    std_srvs::Empty unpauseSrv;
    gazebo_msgs::GetModelState getModelState;
    gazebo_msgs::ModelState modelstate;

    //Vanlige variabler
    ifstream file("/home/roshan/.gazebo/models/marker_2/model.sdf"); //Trenger for å lese xml fil
    string line;
    string modelName = "tb3_0" ;
    string relativeEntityName = "world" ;

    //Initialiserer services
    ros::ServiceClient pauseGazebo= n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    ros::ServiceClient spawn_sdf_model_client = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    ros::ServiceClient get_model = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient attach_client = n.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");

    getModelState.request.model_name = modelName;
    getModelState.request.relative_entity_name = relativeEntityName;
    
    if (get_model.call(getModelState)){
        pauseGazebo.call(pauseSrv); //Pauser Gazebo

        //Setter pose til marker
        marker_pose = getModelState.response.pose;
        marker_pose.position.y -= 0.2;
        marker_pose.position.z = 0.15;


        /*Vanskelig å sette quaternions, bruker derfor
        tf2, tar fra roll/pitch/yaw til quaternions, og så
        transformerer til geometry_msgs*/
        quat_tf.setRPY(0,0,4.7123);
        quat_tf = quat_tf.normalize();
        marker_quat = tf2::toMsg(quat_tf);
        marker_pose.orientation = marker_quat;


        ROS_INFO("Robot X= %f ;",getModelState.response.pose.position.x);
        ROS_INFO("Marker X= %f ;",marker_pose.position.x);

        //Leser xml fil
        while(!file.eof()){
            getline(file,line);
            marker.request.model_xml+=line;
        }
        file.close();

        //Lager marker og setter posen dens og hvilket system den skal være i forhold til
        marker.request.model_name = "marker";
        marker.request.reference_frame="world";
        marker.request.initial_pose = marker_pose; 
        spawn_sdf_model_client.call(marker);

        /*Lager attach variabler, skal attache marker til turtlebot,
        attacher marker til base til turtlebot*/
        attacher.request.model_name_1 = "tb3_0";
        attacher.request.link_name_1 = "base_footprint";
        attacher.request.model_name_2 = "marker";
        attacher.request.link_name_2 = "link";

        attach_client.call(attacher); //Kaller attach
        unpauseGazebo.call(unpauseSrv); //Unpauser
        
    }
    
    return 0;
}
