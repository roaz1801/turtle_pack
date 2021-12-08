import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from fiducial_msgs.msg import FiducialArray
import math
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt


class move_bot:
    def __init__(self):
        self.pub = rospy.Publisher("tb3_1/cmd_vel",Twist,queue_size=100)
        #self.pub_fiducial = rospy.Subscriber("/fiducial_vertices",FiducialArray,queue_size=100)
        #self.sub = rospy.Subscriber("tb3_1/scan",LaserScan,self.callback)
        self.sub = rospy.Subscriber("/fiducial_vertices",FiducialArray,self.callback)
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

        self.temp_store_v = []
        self.temp_store_w = []

        self.v = 0
        self.w = 0

    def control(self,h,t):
        move = Twist()
        m = self.marker_center[0]
        n = self.marker_center[1]

        print("M:",m)
        print("N:",n)
        rho_inf_m = 60
        rho_inf_n = 30
        k1 = 0.1
        k2 = 0.45
        l = 0.1

        #Center pixels, taken from camera calibration matrix K
        m0 = 320 
        n0 = 240

        #Found through testing
        alpha_m = 616
        alpha_n = 616

        #Calculated from eq. (5)
        """
        Original, skal teste andre.
        m_desired = 320
        n_desired = 190
        """
        m_desired = 320
        n_desired = 195

        #p = (m-m0)/alpha_m 
        #q = (m-m0)/alpha_n 

        #z = h/q 
       
       #Original FOV constraints. Kommentert ut for å teste andre.
        #m_max = 560
        #m_min = 70
        #n_min = 140
        #n_max = 230

       # """
       # Disse funker bra, men må teste andre
        m_max = 620
        m_min = 20
        n_min = 170
        n_max = 235
       # """
        #m_max = 640
        #m_min = 0
        #n_min = 0
        #n_max = 480

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
        rho_m = (1-rho_inf_m/max(Cm_under,Cm_over))*np.exp(-l*t) + rho_inf_m/max(Cm_under,Cm_over)
        rho_n = (1-rho_inf_n/max(Cn_under,Cn_over))*np.exp(-l*t) + rho_inf_n/max(Cn_under,Cn_over)
       
        #print("Rho m:",rho_m)
        #print("Rho n:",rho_n)
        #print("Rho m:",rho_m)
        #print("Cm_under*rho_m",Cm_under*rho_m)
        #print("Cn_under*rho_n",Cn_under*rho_n)
        #if Cn_over*rho_n > error_n and Cm_over*rho_m > error_m:
            #print("Error too large")
        epsilon_m = np.log((error_m+Cm_under*rho_m)/(Cm_over*rho_m-error_m))
        epsilon_n = np.log((error_n+Cn_under*rho_n)/(Cn_over*rho_n-error_n))

        self.store_epsilon_m = epsilon_m
        self.store_epsilon_n = epsilon_n

        self.store_error_n = error_n
        self.store_error_m = error_m

        self.store_rho_n = rho_n
        self.store_rho_m = rho_m

            #print("Epsilon m:",epsilon_m)
            #print("Epsilon teller:",error_n+Cn_under*rho_n)
        #print("Epsilon n nevner:",Cn_over*rho_n-error_n)
        #print("Epsilon n:",epsilon_n)

        #print("Epsilon m teller:",error_m+Cm_under*rho_m)
        #print("Epsilon m nevner:",Cm_over*rho_m-error_m)
        #print("Epsilon m:",epsilon_m)
        #print("-------------")

        self.store_lowerboundary_m = -Cm_under*rho_m
        self.store_upperboundary_m = Cm_over*rho_m

        self.store_lowerboundary_n = -Cn_under*rho_n
        self.store_upperboundary_n = Cn_over*rho_n

        self.store_m = m
        self.store_n = n

        if self.temp_store_v:
            #Static Gain Controller
            temp_v = k2*epsilon_n*np.cos(epsilon_n)
            temp_w = -k1*epsilon_m
            self.v = 0.3*self.temp_store_v[-1]+0.7*temp_v
            self.w = 0.3*self.temp_store_w[-1]+0.7*temp_w
            #if len(self.temp_store_v) >= 2:
             #   self.v = 0.1*self.temp_store_v[-2]+0.2*self.temp_store_v[-1]+0.7*temp_v
              #  self.w = 0.1*self.temp_store_w[-2]+0.2*self.temp_store_w[-1]+0.7*temp_w
            if len(self.temp_store_v) >= 3:
                del self.temp_store_v[0:-3]
        else: 
            self.v = k2*epsilon_n*np.cos(epsilon_n)
            self.w = -k1*epsilon_m
        
        self.store_v = self.v
        self.store_w = self.w

        self.temp_store_v.append(self.v) 
        self.temp_store_w.append(self.w)

        move.linear.x = self.v
        move.angular.z = self.w
        self.pub.publish(move)


    def callback(self,msg):
        #print(msg.fiducials)
        #print(msg.fiducials)
        if msg.fiducials:
            #print("Has elements")
            marker_center_x = (msg.fiducials[0].x0+msg.fiducials[0].x1+msg.fiducials[0].x2+msg.fiducials[0].x3)/4
            marker_center_y = (msg.fiducials[0].y0+msg.fiducials[0].y1+msg.fiducials[0].y2+msg.fiducials[0].y3)/4
            self.marker_center = [marker_center_x,marker_center_y]
        #if not msg.fiducials:
            #self.marker_center = [marker_center_x,marker_center_y]
            #print("Does not have elements")
            

        #print(self.marker_center)


if __name__ == '__main__':
    rospy.init_node("vision_filter",disable_signals=True)
    rate = rospy.Rate(135) #Loop rate 100Hz
    obj = move_bot()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

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

    #Passer på at tid starter korrekt/tar hånd om race condition
    prevTime = 0
    while not prevTime:
        prevTime = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("tb3_1/base_link", "fiducial_0", rospy.Time(0))
            trans_camera_frame = tfBuffer.lookup_transform("tb3_1/camera_rgb_optical_frame", "fiducial_0", rospy.Time(0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        try:
            currentTime = rospy.Time.now()


            #dt
            delT = currentTime-prevTime

            h = -0.06#trans_camera_frame.transform.translation.y 

            distance = np.sqrt((0-trans.transform.translation.x)**2+(0-trans.transform.translation.y)**2)
            rads = np.arctan(-trans.transform.translation.y/-trans.transform.translation.x)
            angle = math.degrees(rads)
            #print("Distance:",distance)
            #print("Angle:",rads)
            
            #print(h)

            #print(obj.pub_fiducial)
            #Sender distans, angles og tid til kontroll
            obj.control(h,delT.to_sec())


            time_list.append(delT.to_sec())
            epsilon_n_list.append(obj.store_epsilon_n)
            epsilon_m_list.append(obj.store_epsilon_m)
            error_m_list.append(obj.store_error_m)
            error_n_list.append(obj.store_error_n)
            rho_m_list.append(obj.store_rho_m)
            rho_n_list.append(obj.store_rho_n)
            m_lower_boundary_list.append(obj.store_lowerboundary_m)
            m_upper_boundary_list.append(obj.store_upperboundary_m)
            n_lower_boundary_list.append(obj.store_lowerboundary_n)
            n_upper_boundary_list.append(obj.store_upperboundary_n)
            m_list.append(obj.store_m)
            n_list.append(obj.store_n)

            v_list.append(obj.store_v)
            w_list.append(obj.store_w)


            rate.sleep()

        except KeyboardInterrupt:
            break

    figure, axis = plt.subplots(3, 2)

    axis[0, 0].plot(time_list,v_list)
    axis[0, 0].set_xlabel("Time in seconds")
    axis[0, 0].set_ylabel("Velocity in m/s")
    axis[0, 0].set_title("Velocity")
    
    axis[0, 1].plot(time_list,w_list)
    axis[0, 1].set_xlabel("Time in seconds")
    axis[0, 1].set_ylabel("Angular velocity in m/s")
    axis[0, 1].set_title("Angular velocity")
   
    #axis[1, 0].plot(time_list,rho_n_list)
    #axis[1, 0].set_title("Rho n")

    #axis[1, 1].plot(time_list,rho_m_list)
    #axis[1, 1].set_title("Rho m")

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

    #axis[3, 0].plot(time_list,error_n_list)
    #axis[3, 0].set_title("Error n")

    #axis[3, 1].plot(time_list,error_m_list)
    #axis[3, 1].set_title("Error m")

    #axis[4, 0].plot(time_list,epsilon_n_list)
    #axis[4, 0].set_title("Epsilon n")

    #axis[4, 1].plot(time_list,epsilon_m_list)
    #axis[4, 1].set_title("Epsilon m")

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
