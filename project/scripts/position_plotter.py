
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class move_bot:
    def __init__(self):
        self.pub = rospy.Publisher("tb3_0/cmd_vel",Twist,queue_size=1)
        #self.sub = rospy.Subscriber("tb3_0/scan",LaserScan,self.callback)
        self.linx = 0.1
        self.angz = 0
    

    def control(self,time):
        move = Twist()
        move.linear.x = 0.2
        move.angular.z = 0.1 
        print(time)
        #timer = np.cos(time/35)
        if time > 64 and time < 128:
            move.angular.z = -0.1 
        if time > 128 and time < 182:
            move.angular.z = 0.1 
        if time > 182:
            move.angular.z = -0.1 


        
        #if(time>50 and time < 100):
         #   move.linear.x = 0.2
          #  move.angular.z = -0.1
        #if(time>100 and time<120):
         #   move.linear.x = 0.2
          #  move.angular.z = 0
        #if(time>120):
         #   move.linear.x = 0
          #  move.angular.z = 0
        """
        if(time>40):
            move.linear.x = 0
            move.angular.z = 0
        else:
            move.linear.x = 0.5
            move.angular.z = 0.1
        """
        self.pub.publish(move)


if __name__ == '__main__':
    rospy.init_node("simple_follow")
    rate = rospy.Rate(1000)
    obj = move_bot()


    prevTime = 0
    while not prevTime:
        prevTime = rospy.Time.now()

    while not rospy.is_shutdown():

        currentTime = rospy.Time.now()
        delT = currentTime-prevTime
        #print(delT.to_sec())
        obj.control(delT.to_sec())
        rate.sleep()

