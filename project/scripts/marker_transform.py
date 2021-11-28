import rospy
import math
import tf2_ros
from gazebo_msgs.srv import GetModelState

def client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException:
        print("Service call failed")

def handle_pose(msg, bot):
    


if __name__ == '__main__':
    rospy.init_node("model_broadcast")
    model_name = "marker"
    relative_entity_name = "tb3_0"
    res = client(model_name,relative_entity_name)
    print(res.pose.position.x)
    print(res.pose.position.y)
    print(res.pose.position.z)




        

