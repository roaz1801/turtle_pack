import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
import csv

class move_bot:
    def __init__(self):
        self.pub = rospy.Publisher("tb3_1/cmd_vel",Twist,queue_size=100)
        #self.sub = rospy.Subscriber("tb3_1/scan",LaserScan,self.callback)
        self.store_rho_d = 0
        self.store_rho_beta = 0
    
        self.store_error_d = 0
        self.store_error_beta = 0

        self.store_xi_d = 0
        self.store_xi_beta = 0
        
        self.store_epsilon_d = 0
        self.store_epsilon_beta = 0

        self.store_upperboundary_d = 0
        self.store_lowerboundary_d = 0

        self.store_upperboundary_beta = 0
        self.store_lowerboundary_beta = 0
        
        self.store_v = 0
        self.store_w = 0

    def control(self,d,beta,t):
        move = Twist()
        angle_of_view = 60 #Horizonal FoV til raspberry pi cam 
        #Placeholder values, disse må testes og justeres
        d_con = 2.8 #Lengde der kamera slutter å merke marker, må finne og justere denne.
        l = 0.1
        K_d = 0.3
        K_beta = 0.7
        rho_d_inf = 0.4 #Steady state error på distance
        rho_beta_inf = 15 #Steady state error på vinkel

        #Variables
        d_desired = 0.75
        d_col = 0.2*d_desired
        beta_con = angle_of_view/2 
        M_beta_under = beta_con 
        M_beta_over = beta_con 
        M_d_under = d_desired-d_col 
        M_d_over = d_con-d_desired


        rho_d = (1-rho_d_inf/M_d_over)*np.exp(-l*t)+rho_d_inf/M_d_over
        rho_beta = (1-rho_beta_inf/M_beta_over)*np.exp(-l*t) + rho_beta_inf/M_beta_over

        #Error
        error_d = d-d_desired 
        error_beta = beta

        print("Error d",error_d)
        print("Error beta:",error_beta)

        #Normalized error
        xi_d = error_d/rho_d
        xi_beta = error_beta/rho_beta

        #print("Xi_d:",xi_d)
        #print("Xi_beta:",xi_beta)
        print("Rho d:",rho_d)
        print("Rho beta:",rho_beta)
        #print("Cm_under*rho_m",M_d_under*rho_d)
        #print("Cn_under*rho_n",M_beta_under*rho_beta)

        epsilon_d = np.log((1+xi_d/M_d_under)/(1-xi_d/M_d_over))
        epsilon_beta = np.log((1+xi_beta/M_beta_under)/(1-xi_beta/M_beta_over))

        print("Epsilon_d:",epsilon_d)
        print("Epsilon_beta:",epsilon_beta)

        r_beta = ((1/M_beta_under)+(1/M_beta_over))/(1+(xi_beta/M_beta_under)*(1-(xi_beta/M_beta_over)))

        v = K_d*epsilon_d
        w = K_beta*(1/rho_beta)*r_beta*epsilon_beta
        
        self.store_rho_d = rho_d 
        self.store_rho_beta = rho_beta 
    
        self.store_error_d = error_d 
        self.store_error_beta = error_beta 

        self.store_xi_d = xi_d 
        self.store_xi_beta = xi_beta 
        
        self.store_epsilon_d = epsilon_d 
        self.store_epsilon_beta = epsilon_beta 

        self.store_upperboundary_d = M_d_over*rho_d 
        self.store_lowerboundary_d = -M_d_under*rho_d 

        self.store_upperboundary_beta = M_beta_over*rho_beta
        self.store_lowerboundary_beta = -M_beta_under*rho_beta
        
        self.store_v = v 
        self.store_w = w

        print("-------------")
        move.linear.x = v
        move.angular.z = w

        self.pub.publish(move)

if __name__ == '__main__':
    rospy.init_node("decentralized_follow",disable_signals=True)
    rate = rospy.Rate(135) #Loop rate 100Hz
    obj = move_bot()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    time_list = []
    epsilon_d_list = []
    epsilon_beta_list = []
    error_d_list = []
    error_beta_list = []
    rho_d_list = []
    rho_beta_list = []
    d_lower_boundary_list = []
    d_upper_boundary_list = []
    beta_lower_boundary_list = []
    beta_upper_boundary_list = []
    distance_list = []
    angle_list = []
    v_list = []
    w_list = []


    #Passer på at tid starter korrekt/tar hånd om race condition
    prevTime = 0
    while not prevTime:
        prevTime = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("tb3_1/base_link", "fiducial_0", rospy.Time(0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        try:
            currentTime = rospy.Time.now()

            #dt
            delT = currentTime-prevTime

            #Får distans og vinkel mellom robotter
            distance = np.sqrt((0-trans.transform.translation.x)**2+(0-trans.transform.translation.y)**2)
            rads = np.arctan(-trans.transform.translation.y/-trans.transform.translation.x)
            angle = math.degrees(rads)

            print("Distance:",distance)
            #Sender distans, angles og tid til kontroll
            obj.control(distance,angle,delT.to_sec())

            time_list.append(delT.to_sec())
            epsilon_d_list.append(obj.store_epsilon_d)
            epsilon_beta_list.append(obj.store_epsilon_beta)
            error_d_list.append(obj.store_error_d)
            error_beta_list.append(obj.store_error_beta)
            rho_d_list.append(obj.store_rho_d)
            rho_beta_list.append(obj.store_rho_beta)
            d_lower_boundary_list.append(obj.store_lowerboundary_d)
            d_upper_boundary_list.append(obj.store_upperboundary_d)
            beta_lower_boundary_list.append(obj.store_lowerboundary_beta)
            beta_upper_boundary_list.append(obj.store_upperboundary_beta)
            distance_list.append(distance) 
            angle_list.append(angle)
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
    axis[0, 1].set_ylabel("Velocity in m/s")
    axis[0, 1].set_title("Angular velocity")
   
    axis[1, 0].plot(time_list,d_lower_boundary_list,label="Lower bound")
    axis[1, 0].plot(time_list,error_d_list,label="Error")
    axis[1, 0].plot(time_list,d_upper_boundary_list,label="Upper bound")
    axis[1, 0].set_xlabel("Time in seconds")
    axis[1, 0].set_ylabel("Error")
    axis[1, 0].set_title("Boundary")
    axis[1, 0].legend()

    axis[1, 1].plot(time_list,beta_lower_boundary_list,label="Lower bound")
    axis[1, 1].plot(time_list,error_beta_list,label="Error")
    axis[1, 1].plot(time_list,beta_upper_boundary_list,label="Upper bound")
    axis[1, 1].set_xlabel("Time in seconds")
    axis[1, 1].set_ylabel("Error")
    axis[1, 1].set_title("Boundary")
    axis[1, 1].legend()

    axis[2, 0].plot(time_list,distance_list)
    axis[2, 0].set_xlabel("Time in seconds")
    axis[2, 0].set_ylabel("Relative distance in m")
    axis[2, 0].set_title("Distance between leader and follower")
    axis[2, 0].legend()

    axis[2, 1].plot(time_list,angle_list)
    axis[2, 1].set_xlabel("Time in seconds")
    axis[2, 1].set_ylabel("Relative angle")
    axis[2, 1].set_title("Angle between leader and follower")
    axis[2, 1].legend()

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

    axis[0].plot(time_list,d_lower_boundary_list,label="Lower bound")
    axis[0].plot(time_list,error_d_list,label="Error")
    axis[0].plot(time_list,d_upper_boundary_list,label="Upper bound")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Error")
    axis[0].set_title("Velocity Error and Boundary")
    axis[0].legend()

    axis[1].plot(time_list,beta_lower_boundary_list,label="Lower bound")
    axis[1].plot(time_list,error_beta_list,label="Error")
    axis[1].plot(time_list,beta_upper_boundary_list,label="Upper bound")
    axis[1].set_title("Angular Velocity Error and Boundary")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Error")
    axis[1].legend()
    figure.suptitle('Boundary and Error for Velocity and Angular Velocity Error', fontsize=16)
    plt.show()

    figure, axis = plt.subplots(1, 2)
    axis[0].plot(time_list,distance_list)
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Relative Distance")
    axis[0].set_title("Distance between follower and leader")

    axis[1].plot(time_list,angle_list)
    axis[1].set_title("Angle between robots")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Angle between follower and leader")
    figure.suptitle('Angle and distance between follower and leader', fontsize=16)
    
    plt.show()
  
"""
    file_data = [time_list,
                v_list, 
                w_list,
                d_lower_boundary_list,
                d_upper_boundary_list,
                error_d_list,
                beta_lower_boundary_list,
                beta_upper_boundary_list,
                error_beta_list,
                distance_list,
                angle_list
                ]
    file = open('.csv','w+',newline='')
    with file:
        write = csv.writer(file)
        write.writerows(file_data)
 """