#Import every package and message that is needed
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from fiducial_msgs.msg import FiducialArray
import math
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
import csv

"""
-----NODE DESCRIPTION-----
vision_follow.py is a node that detects a marker behind the leader vehicle, detects the 
pixel coordinates of the marker in the camera and then uses this to calculate the velocity and angular
velocity of the follower vehicle, as shown in method 2. 
 """

class move_bot:
    def __init__(self):
        #Publish to robot wheels
        self.pub = rospy.Publisher("tb3_1/cmd_vel",Twist,queue_size=100)

        #Subscribe to aruco detection 
        self.sub = rospy.Subscriber("/fiducial_vertices",FiducialArray,self.callback)

        #Initialize variables for use and plotting
        self.marker_center = 0

        self.store_epsilon_n = 0
        self.store_epsilon_m = 0

        self.store_error_n = 0
        self.store_error_m = 0

        self.store_rho_n = 0
        self.store_rho_m = 0

        self.store_v = 0
        self.store_w = 0

        self.store_lowerboundary_m = 0
        self.store_upperboundary_m = 0

        self.store_lowerboundary_n = 0
        self.store_upperboundary_n = 0

        self.store_m = 0 
        self.store_n = 0


    def control(self,h,t):
        """Method where the velocity and angular velocity of the follower is
        calculated and then published to the wheels. """
        move = Twist()

        #Pixel coordinates
        m = self.marker_center[0]
        n = self.marker_center[1]

        #Boundary and tuning parameters
        rho_inf_m = 60
        rho_inf_n = 30
        k1 = 0.07
        k2 = 0.45
        l = 0.1

        #Center pixels, taken from camera calibration matrix K
        m0 = 320 
        n0 = 240

        #Found through testing
        alpha_m = 616
        alpha_n = 616

        #Calculated from equation depending on desired follower position
        m_desired = 320
        n_desired = 195

        #FOV constrains
        m_max = 620
        m_min = 20
        n_min = 170
        n_max = 235

        #Error
        error_m = m-m_desired
        error_n = n-n_desired

        #print("Error m",error_m)
        #print("Error n:",error_n)

        Cm_under = m_desired-m_min 
        Cm_over = m_max-m_desired 
        Cn_under = n_desired-n_min 
        Cn_over = n_max-n_desired

        #print("Cm under: ",Cm_under)
        #print("Cm over: ",Cm_over)
        #print("Cn under: ",Cn_under)
        #print("Cn over: ",Cn_over)

        #Exponential decay
        rho_m = (1-rho_inf_m/max(Cm_under,Cm_over))*np.exp(-l*t) + rho_inf_m/max(Cm_under,Cm_over)
        rho_n = (1-rho_inf_n/max(Cn_under,Cn_over))*np.exp(-l*t) + rho_inf_n/max(Cn_under,Cn_over)
       
        #print("Rho m:",rho_m)
        #print("Rho n:",rho_n)
        #print("Rho m:",rho_m)
        #print("Cm_under*rho_m",Cm_under*rho_m)
        #print("Cn_under*rho_n",Cn_under*rho_n)

        #Normalized error
        epsilon_m = np.log((error_m+Cm_under*rho_m)/(Cm_over*rho_m-error_m))
        epsilon_n = np.log((error_n+Cn_under*rho_n)/(Cn_over*rho_n-error_n))



            #print("Epsilon m:",epsilon_m)
            #print("Epsilon teller:",error_n+Cn_under*rho_n)
        #print("Epsilon n nevner:",Cn_over*rho_n-error_n)
        #print("Epsilon n:",epsilon_n)

        #print("Epsilon m teller:",error_m+Cm_under*rho_m)
        #print("Epsilon m nevner:",Cm_over*rho_m-error_m)
        #print("Epsilon m:",epsilon_m)
        #print("-------------")

        #Static Gain Controller
        v = k2*epsilon_n*np.cos(epsilon_n)
        w = -k1*epsilon_m
        
        #Store current values so they can be put in a list later
        self.store_epsilon_m = epsilon_m
        self.store_epsilon_n = epsilon_n

        self.store_error_n = error_n
        self.store_error_m = error_m

        self.store_rho_n = rho_n
        self.store_rho_m = rho_m

        self.store_v = v
        self.store_w = w

        self.store_lowerboundary_m = -Cm_under*rho_m
        self.store_upperboundary_m = Cm_over*rho_m

        self.store_lowerboundary_n = -Cn_under*rho_n
        self.store_upperboundary_n = Cn_over*rho_n

        self.store_m = m
        self.store_n = n

        #Apply velocities
        move.linear.x = v
        move.angular.z = w
        self.pub.publish(move)


    def callback(self,msg):
        """Callback function that gives the position of the center of the marker in
          pixel coordinates, calculated from the average of the vertices"""
        if msg.fiducials:
            marker_center_x = (msg.fiducials[0].x0+msg.fiducials[0].x1+msg.fiducials[0].x2+msg.fiducials[0].x3)/4
            marker_center_y = (msg.fiducials[0].y0+msg.fiducials[0].y1+msg.fiducials[0].y2+msg.fiducials[0].y3)/4
            self.marker_center = [marker_center_x,marker_center_y]

if __name__ == '__main__':
    rospy.init_node("vision_follow",disable_signals=True)
    rate = rospy.Rate(10) #Loop rate 
    obj = move_bot()

    #Initialize /tf transform listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #Lists to store data to plot
    time_list = []
    epsilon_n_list = []
    epsilon_m_list = []
    error_m_list = []
    error_n_list = []
    rho_n_list = []
    rho_m_list = []
    m_lower_boundary_list = []
    m_upper_boundary_list = []
    n_lower_boundary_list = []
    n_upper_boundary_list = []
    v_list = []
    w_list = []
    m_list = []
    n_list = []
    distance_list = []
    angle_list = []

    #Wait for first message from callbacks. 
    rospy.wait_for_message("/fiducial_vertices",FiducialArray)

    #Makes sure that timer starts correctly and avoids race conditions
    prevTime = 0
    while not prevTime:
        prevTime = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            #Listen to the /tf topics which give the transforms between frames
            trans = tfBuffer.lookup_transform("tb3_1/base_link", "fiducial_0", rospy.Time(0))
            trans_camera_frame = tfBuffer.lookup_transform("tb3_1/camera_rgb_optical_frame", "fiducial_0", rospy.Time(0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        try:
            currentTime = rospy.Time.now()

            delT = currentTime-prevTime

            #Get information about the position of marker in relation to the camera
            h = -0.06#trans_camera_frame.transform.translation.y 

            #Get information about distance and angle 
            distance = np.sqrt((0-trans.transform.translation.x)**2+(0-trans.transform.translation.y)**2)
            rads = np.arctan(-trans.transform.translation.y/-trans.transform.translation.x)
            angle = math.degrees(rads)

            obj.control(h,delT.to_sec())

            #Append data to list for plotting
            time_list.append(delT.to_sec())
            error_m_list.append(obj.store_error_m)
            error_n_list.append(obj.store_error_n)
            m_lower_boundary_list.append(obj.store_lowerboundary_m)
            m_upper_boundary_list.append(obj.store_upperboundary_m)
            n_lower_boundary_list.append(obj.store_lowerboundary_n)
            n_upper_boundary_list.append(obj.store_upperboundary_n)
            m_list.append(obj.store_m)
            n_list.append(obj.store_n)

            v_list.append(obj.store_v)
            w_list.append(obj.store_w)

            distance_list.append(distance) 
            angle_list.append(angle)



            rate.sleep()

        except KeyboardInterrupt:
            break

    #Code for plotting
    figure, axis = plt.subplots(3, 2)

    axis[0, 0].plot(time_list,v_list)
    axis[0, 0].set_xlabel("Time in seconds")
    axis[0, 0].set_ylabel("Velocity in m/s")
    axis[0, 0].set_title("Velocity")
    
    axis[0, 1].plot(time_list,w_list)
    axis[0, 1].set_xlabel("Time in seconds")
    axis[0, 1].set_ylabel("Angular velocity in m/s")
    axis[0, 1].set_title("Angular velocity")
   
    axis[1, 0].plot(time_list,n_lower_boundary_list,label="Lower bound")
    axis[1, 0].plot(time_list,error_n_list,label="Error")
    axis[1, 0].plot(time_list,n_upper_boundary_list,label="Upper bound")
    axis[1, 0].set_xlabel("Time in seconds")
    axis[1, 0].set_ylabel("Error")
    axis[1, 0].set_title("Boundary")
    axis[1, 0].legend()

    axis[1, 1].plot(time_list,m_lower_boundary_list,label="Lower bound")
    axis[1, 1].plot(time_list,error_m_list,label="Error")
    axis[1, 1].plot(time_list,m_upper_boundary_list,label="Upper bound")
    axis[1, 1].set_xlabel("Time in seconds")
    axis[1, 1].set_ylabel("Error")
    axis[1, 1].set_title("Boundary")
    axis[1, 1].legend()

    axis[2, 0].plot(time_list,n_list)
    axis[2, 0].set_xlabel("Time in seconds")
    axis[2, 0].set_ylabel("Pixel coordinate")
    axis[2, 0].set_title("N")

    axis[2, 1].plot(time_list,m_list)
    axis[2, 1].set_xlabel("Time in seconds")
    axis[2, 1].set_ylabel("Pixel coordinate")
    axis[2, 1].set_title("M")

    figure.suptitle('Summary figure', fontsize=16)

    plt.legend()
    plt.show()

    figure, axis = plt.subplots(1, 2)

    axis[0].plot(time_list,v_list)
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Velocity in m/s")
    axis[0].set_title("Velocity")
    
    axis[1].plot(time_list,w_list)
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Angular velocity in m/s")
    axis[1].set_title("Angular velocity")

    figure.suptitle('Velocity and Angular Velocity of Follower', fontsize=16)

    plt.legend()
    plt.show()

    figure, axis = plt.subplots(1, 2)

    axis[0].plot(time_list,n_lower_boundary_list,label="Lower bound")
    axis[0].plot(time_list,error_n_list,label="Error")
    axis[0].plot(time_list,n_upper_boundary_list,label="Upper bound")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Error")
    axis[0].set_title("Velocity Error and Boundary")
    axis[0].legend()

    axis[1].plot(time_list,m_lower_boundary_list,label="Lower bound")
    axis[1].plot(time_list,error_m_list,label="Error")
    axis[1].plot(time_list,m_upper_boundary_list,label="Upper bound")
    axis[1].set_title("Angular Velocity Error and Boundary")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Error")
    axis[1].legend()
    figure.suptitle('Boundary and Error for Velocity and Angular Velocity Error', fontsize=16)
    plt.show()

    figure, axis = plt.subplots(1, 2)
    axis[0].plot(time_list,n_list)
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Pixel coordinate")
    axis[0].set_title("Vertical pixel coordinate: N")

    axis[1].plot(time_list,m_list)
    axis[1].set_title("Horizontal pixel coordinate: M")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Pixel coordinate")
    figure.suptitle('Pixel coordinates of the marker in followers camera', fontsize=16)
    plt.show()

    figure, axis = plt.subplots(1, 2)
    axis[0].plot(time_list,distance_list)
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Relative distance")
    axis[0].set_title("Distance between leader and follower")

    axis[1].plot(time_list,angle_list)
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Relative angle")
    axis[1].set_title("Angle between leader and follower")
    plt.show()

"""
    file_data = [time_list,
                v_list, 
                w_list,
                n_lower_boundary_list,
                n_upper_boundary_list,
                error_n_list,
                m_lower_boundary_list,
                m_upper_boundary_list,
                error_m_list,
                n_list,
                m_list,
                distance_list,
                angle_list
                ]
    file = open('.csv','w+',newline='')
    with file:
        write = csv.writer(file)
        write.writerows(file_data)
"""