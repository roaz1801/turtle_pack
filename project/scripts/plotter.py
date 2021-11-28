#Import every package and message that is needed
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from fiducial_msgs.msg import FiducialArray
import math
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt

"""
-----NODE DESCRIPTION-----
plotter.py node stores the positions of the follower and leader, 
and plots them 
"""

if __name__ == '__main__':
    rospy.init_node("plotter",disable_signals=True)
    rate = rospy.Rate(135) #Loop rate

    #Initialize /tf transform listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #Lists for storing the x and y position of leader and follower
    tb0_xlist = []
    tb0_ylist = []
    tb1_xlist = []
    tb1_ylist = []

    while not rospy.is_shutdown():
        try:
            #Listen to the /tf topics which give the transforms between frames
            trans_tb0 = tfBuffer.lookup_transform("world", "tb3_0/base_link", rospy.Time(0))
            trans_tb1 = tfBuffer.lookup_transform("world", "tb3_1/base_link", rospy.Time(0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        try:
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

            rate.sleep()

        except KeyboardInterrupt:
            break
       
    #Code for plotting
    plt.plot(tb0_xlist,tb0_ylist,label="Position of leader")
    plt.plot(tb1_xlist,tb1_ylist,label="Position of follower")
    plt.title("Position of leader and follower")
    plt.xlabel("X position in meters")
    plt.ylabel("Y position in meters")
    plt.axis('equal')
    plt.legend()
    plt.show()