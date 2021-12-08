#Import every package and message that is needed
import rospy
import tf_conversions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from fiducial_msgs.msg import FiducialArray
from nav_msgs.msg import Odometry
import math
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler
import csv

"""
-----NODE DESCRIPTION-----
plotter.py node stores the positions of the follower and leader, 
and plots them. Also writes data to files.
"""

class data:
    def __init__(self):
        self.sub1 = rospy.Subscriber("/tb3_0/odom",Odometry,self.tb3_0_callback)
        self.sub2 = rospy.Subscriber("/tb3_1/odom",Odometry,self.tb3_1_callback)


        self.heading_tb0_store = 0
        self.heading_tb1_store = 0

    def store_data(self):

        self.heading_tb0_store = self.heading_tb0
        self.heading_tb1_store = self.heading_tb1

    def tb3_0_callback(self,msg):
        quat = (msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)#msg.pose.pose.orientation
        euler = tf_conversions.transformations.euler_from_quaternion(quat)
        
        self.heading_tb0 = euler[2]

    def tb3_1_callback(self,msg):
        quat = (msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        euler = tf_conversions.transformations.euler_from_quaternion(quat)
        
        self.heading_tb1 = euler[2]


if __name__ == '__main__':
    rospy.init_node("position_plotter",disable_signals=True)
    rate = rospy.Rate(135) #Loop rate

    obj = data()

    #Initialize /tf transform listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #Lists for storing the x and y position of leader and follower
    tb0_xlist = []
    tb0_ylist = []
    tb1_xlist = []
    tb1_ylist = []

    tb0_heading_list = []
    tb1_heading_list = []

    metode = 1

    time_list = []
    #Passer på at tid starter korrekt/tar hånd om race condition
    prevTime = 0
    while not prevTime:
        prevTime = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            #Listen to the /tf topics which give the transforms between frames
            trans_tb0 = tfBuffer.lookup_transform("world", "tb3_0/base_link", rospy.Time(0))
            trans_tb1 = tfBuffer.lookup_transform("world", "tb3_1/base_link", rospy.Time(0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        try:
            
            #rospy.wait_for_message("/tb3_0/odom",Odometry)
            #rospy.wait_for_message("/tb3_1/odom",Odometry)
            currentTime = rospy.Time.now()
            delT = currentTime-prevTime

            time_list.append(delT.to_sec())

            obj.store_data()
            #Get the information about the positions
            tb0_x = trans_tb0.transform.translation.x
            tb0_y = trans_tb0.transform.translation.y 
            tb1_x = trans_tb1.transform.translation.x
            tb1_y = trans_tb1.transform.translation.y 

            #Append data to list for plotting
            tb0_xlist.append(tb0_x) 
            tb0_ylist.append(tb0_y)
            tb1_xlist.append(tb1_x) 
            tb1_ylist.append(tb1_y)

            tb0_heading_list.append(obj.heading_tb0_store)
            tb1_heading_list.append(obj.heading_tb1_store)

            rate.sleep()
            print(delT.to_sec())
        except KeyboardInterrupt:
            break

    if metode == 1:
        delay = 640

        if len(tb0_heading_list) >= delay:

            leader_x = np.array(tb0_xlist[:-delay])
            follower_x = np.array(tb1_xlist[delay:])

            leader_y = np.array(tb0_ylist[:-delay])
            follower_y = np.array(tb1_ylist[delay:])

            leader_heading = np.array(tb0_heading_list[:-delay])
            follower_heading = np.array(tb1_heading_list[delay:])

            print("Leader:",leader_heading)
            print("Follower:",follower_heading)

            heading_error = leader_heading-follower_heading
            for i in range(len(heading_error)):
                leader_heading[i] = math.degrees(leader_heading[i])
                follower_heading[i] = math.degrees(follower_heading[i])
                heading_error[i] = math.degrees(heading_error[i])
            x_error = leader_x-follower_x
            y_error = leader_y-follower_y

            rms = np.sqrt(1/2*(x_error**2+y_error**2))

            plt.title("Heading in angles")
            plt.plot(time_list[delay:],leader_heading,label="Leader heading")
            plt.plot(time_list[delay:],follower_heading,label="Follower heading")
            plt.plot(time_list[delay:],heading_error,label="Heading error")
            plt.xlabel("Time")
            plt.ylabel("Angle")
            plt.legend()
            plt.show()

            plt.title("RMS of position error")
            plt.plot(time_list[delay:],rms,label="RMS")
            plt.xlabel("RMS")
            plt.ylabel("Angle")
            plt.legend()
            plt.show()

    #Code for plotting
    plt.plot(tb0_xlist,tb0_ylist,label="Position of leader")
    plt.plot(tb1_xlist,tb1_ylist,label="Position of follower")
    plt.title("Position of leader and follower")
    plt.xlabel("X position in meters")
    plt.ylabel("Y position in meters")
    plt.axis('equal')
    plt.legend()
    plt.show()


    file_data = [time_list[delay:],
                leader_heading, 
                follower_heading,
                heading_error,
                x_error,
                y_error,
                tb0_xlist,
                tb0_ylist,
                tb1_xlist,
                tb1_ylist,
                ]
    file = open('.csv','w+',newline='')
    with file:
        write = csv.writer(file)
        write.writerows(file_data)
