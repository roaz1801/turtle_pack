#Import every package and message that is needed
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

"""
-----NODE DESCRIPTION-----
simple_follow.py node decides the velocity and angular velocity of the leader vehicle.
There are 4 different ways the leader can drive depending on the method variable's value.
 """

class move_bot:
    def __init__(self):
        #Publish to robot wheels
        self.pub = rospy.Publisher("tb3_0/cmd_vel",Twist,queue_size=1)

        #Initialize variables for use and plotting
        self.store_v = 0
        self.store_w = 0
    
    def control(self,time,method):
        """Method where the velocity and angular velocity of the follower is
        set and then published to the wheels. """
        move = Twist()

        if method == 1:
        #Method 1, drive in counter clockwise circle
            move.linear.x = 0.2
            move.angular.z = 0.1 

        if method == 2:
        #Method 2, drive in a figure 8
            move.linear.x = 0.2
            move.angular.z = 0.1 
            if time > 64 and time < 128:
                move.angular.z = -0.1 
            if time > 128 and time < 192:
                move.angular.z = 0.15 
            if time > 192 and time < 256:
                move.angular.z = -0.15
            if time > 256:
                move.angular.z = 0.1

        if method == 3:
        #Method 3, drive in a line with constant velocity
            move.linear.x = 0.5
            move.angular.z = 0 

        if method == 4:
        #Method 4, drive in a line, accelerates up to 0.4 and then down to 0
            if time >= 0 and time <= 20:
                move.linear.x = 0.1
            if time > 20 and time <= 40:
                move.linear.x = 0.2
            if time > 40 and time <= 60:
                move.linear.x = 0.3
            if time > 60 and time <= 80:
                move.linear.x = 0.4
            if time > 80 and time <= 100:
                move.linear.x = 0.3
            if time > 100 and time <= 120:
                move.linear.x = 0.2
            if time > 120 and time <= 140:
                move.linear.x = 0.1
            if time > 140:
                move.linear.x = 0

        #Apply velocities
        self.store_v = move.linear.x
        self.store_w = move.angular.z
        self.pub.publish(move)


if __name__ == '__main__':
    rospy.init_node("simple_follow")
    rate = rospy.Rate(135)
    obj = move_bot()

    #4 methods, 1 is driving in circle, 2 is driving in figure 8, 3 is line with constant speed
    #4 is line with changing speeds.
    method = 1
    
    prevTime = 0

    #Lists for plotting
    v_list = []
    w_list = []
    time_list = []

    #Makes sure that timer starts correctly and avoids race conditions
    while not prevTime:
        prevTime = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            currentTime = rospy.Time.now()
            delT = currentTime-prevTime
            print("Time:",delT.to_sec())

            obj.control(delT.to_sec(),method)

            #Append data to list for plotting
            time_list.append(delT.to_sec())
            v_list.append(obj.store_v)  
            w_list.append(obj.store_w) 

            rate.sleep()
        except KeyboardInterrupt:
            break

    #Code for plotting
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(time_list,v_list)
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Velocity in m/s")
    axis[0].set_title("Velocity of Leader")
    
    axis[1].plot(time_list,w_list)
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Velocity in m/s")
    axis[1].set_title("Angular velocity of Leader")

    plt.show()