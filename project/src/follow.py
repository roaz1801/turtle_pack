import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class bot:
    def __init__(self):
        self.pub = rospy.Publisher("tb3_0/cmd_vel",Twist,queue_size=100)
        self.sub = rospy.Subscriber("tb3_0/scan",LaserScan,self.callback)
        self.linx = 1
        self.angz = 0.2
    

    def control(self):
        move = Twist()
        L = 0.001
        theta_p_deriver = -c(s)*cos(theta_p)/(1-d*c(s))*v_long
        
        #Longitudinal control
        spacing_error = s_leader-s-l
        curv_error = spacing_error-h*(v-v_truck)

        long_control = -k_a*a + k_v*(v_leader-v) + k_p*curv_error

        #Lateral control
        #theta_p_derivert = ()


        #psi = theta_p_derivert + k_theta*theta_p+k_d*d
        #lat_control = -K*psi-k_theta*theta_p_derivert-k_d*d_derivert

        move.linear.x = self.linx
        move.angular.z = self.angz
        self.pub.publish(move)

    def callback(self,msg):
        min_dist = 1
        print(msg.ranges[0])
        if(msg.ranges[0]<min_dist):
            self.linx = -1
        else:
            self.linx = 1
        """
    if(msg->ranges[0] < min_dist){
        linx = -1; 
    }
    if(msg->ranges[180] < min_dist){
        linx = 1;
    }else{
            linx = 4 
            angz = 0.5
    }
"""

if __name__ == '__main__':
    rospy.init_node("follow")
    rate = rospy.Rate(1)
    obj = move_bot()

    while not rospy.is_shutdown():
        obj.control()
        rate.sleep()


