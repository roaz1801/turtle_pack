import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
import numpy as np



class move_bot:
    def __init__(self):
        self.pub = rospy.Publisher("tb3_1/cmd_vel",Twist,queue_size=1)

    def control(self,d,t):
        
        move = Twist()
        angle_of_view = 62.2 #Horizonal FoV for raspberry pi cam 
        #Placeholder values
        d_con = 2 
        l = 0.5
        K_d = 0.005
        K_beta = 0.001
        rho_d_inf = 0.0625
        rho_beta_inf = 1.15 

        d_desired = 0.75
        d_col = 0.0375
        beta_con = angle_of_view/2 
        M_beta_under = beta_con 
        M_beta_over = beta_con 
        M_d_under = d_desired-d_col 
        M_d_over = d_con-d_desired

        rho_d = (1-rho_d_inf/M_d_over)*np.exp(-l*t)+rho_d_inf/M_d_over
        rho_beta = (1-(rho_d_inf/M_beta_under))*np.exp(-l*t) + rho_d_inf/M_beta_under

        error_d = d-d_desired 
        xi_d = error_d/rho_d


        print("Under:",-M_d_under)
        print("Xi:",xi_d)
        print("Over:",M_d_over)

        epsilon_d = np.log((1+xi_d/M_d_under)/(1-xi_d/M_d_over))

        v = K_d*epsilon_d

        move.linear.x = v
        move.angular.z = 0



if __name__ == '__main__':
    rospy.init_node("decentralized_follow",disable_signals=True)
    rate = rospy.Rate(14)
    obj = move_bot()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    count = 0

    time_start = rospy.get_rostime().secs
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("tb3_1/base_link", "fiducial_0", rospy.Time(0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        try:
            distance = np.sqrt((0-trans.transform.translation.x)**2+(0-trans.transform.translation.y)**2)
            count += 1.0
            time = rospy.get_rostime().secs-time_start

            angle = np.arctan(-trans.transform.translation.y/-trans.transform.translation.x)
            print(angle)
            obj.control(distance,time)


            rate.sleep()

        except KeyboardInterrupt:
            break

        

