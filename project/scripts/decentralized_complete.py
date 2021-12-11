#Import every package and message that is needed
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
        #Publish to robot wheels
        self.pub = rospy.Publisher("tb3_1/cmd_vel",Twist,queue_size=100)

        #Subscribe to lidar 
        self.sub = rospy.Subscriber("/tb3_1/scan",LaserScan,self.callback)

        #Initialize variables for use and plotting
        self.lidar_dist = 0
        self.lidar_angle = 0

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

        self.v = 0
        self.w = 0

        self.temp_store_v = []
        self.temp_store_w = []

    def control(self,cam_dist,cam_beta,t):
        """Method where the velocity and angular velocity of the follower is
        calculated and then published to the wheels. """
        d = 0.5*self.lidar_dist+0.5*cam_dist 
        beta = 0.5*self.lidar_angle+0.5*cam_beta

        move = Twist()
        angle_of_view = 60 #Horizonal FoV of the Raspberry Pi

        #Boundary and tuning parameters
        d_con = 2.8 #Length where camera stops detecting marker
        l = 0.1
        K_d = 0.3
        K_beta = 0.7
        rho_d_inf = 0.4 #Steady state error on distance
        rho_beta_inf = 15 #Steady state error on angle

        d_desired = 0.75
        d_col = 0.2*d_desired
        beta_con = angle_of_view/2 
        M_beta_under = beta_con 
        M_beta_over = beta_con 
        M_d_under = d_desired-d_col 
        M_d_over = d_con-d_desired

        #Exponential decay function
        rho_d = (1-rho_d_inf/M_d_over)*np.exp(-l*t)+rho_d_inf/M_d_over
        rho_beta = (1-rho_beta_inf/M_beta_over)*np.exp(-l*t) + rho_beta_inf/M_beta_over

        #Error
        error_d = d-d_desired 
        error_beta = beta 

        #Normalized error
        xi_d = error_d/rho_d
        xi_beta = error_beta/rho_beta
    
        epsilon_d = np.log((1+xi_d/M_d_under)/(1-xi_d/M_d_over))
        epsilon_beta = np.log((1+xi_beta/M_beta_under)/(1-xi_beta/M_beta_over))
        #self.store_epsilon = epsilon_d
        #print("Epsilon_d:",epsilon_d)
        #print("Epsilon_beta:",epsilon_beta)

        r_beta = ((1/M_beta_under)+(1/M_beta_over))/(1+(xi_beta/M_beta_under)*(1-(xi_beta/M_beta_over)))

        #Store current values so they can be put in a list later
        self.v = K_d*epsilon_d
        self.w = K_beta*(1/rho_beta)*r_beta*epsilon_beta
        
        self.store_xi_d = xi_d
        self.store_xi_beta = xi_beta
        
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

        #Moving average filter
        if self.temp_store_v:
            temp_v = K_d*epsilon_d
            temp_w = K_beta*(1/rho_beta)*r_beta*epsilon_beta
            self.v = 0.3*self.temp_store_v[-1]+0.7*temp_v
            self.w = 0.3*self.temp_store_w[-1]+0.7*temp_w
            if len(self.temp_store_v) >= 2:
                self.v = 1/3*self.temp_store_v[-2]+1/3*self.temp_store_v[-1]+1/3*temp_v
                self.w = 1/3*self.temp_store_w[-2]+1/3*self.temp_store_w[-1]+1/3*temp_w
            if len(self.temp_store_v) >= 3:
                del self.temp_store_v[0:-3]
        else:
            self.v = K_d*epsilon_d
            self.w = K_beta*(1/rho_beta)*r_beta*epsilon_beta
        
        self.store_v = self.v 
        self.store_w = self.w

        self.temp_store_v.append(self.v) 
        self.temp_store_w.append(self.w)

        #Apply velocities
        move.linear.x = self.v
        move.angular.z = self.w

        self.pub.publish(move)

    def callback(self,msg):
        """Callback function that uses the lidar, detects an object, thresholds to make sure
        we only take the rays that hit the specific objects and gives the angle and distance to the object"""
        detected = []
        #Originally -30 to 30, picked -27 to 28 for better context with the camera
        for i in range(-27,28):
            if(math.isinf(msg.ranges[i])) == False:
                index = i
                length = msg.ranges[i]
                angle = msg.angle_min+(i*msg.angle_increment)
                c = (i,angle,length)
                detected.append(c)

        dist_avg = 0
        rads_avg = 0 
        thresh = 0
        for i in range(len(detected)):
            dist_avg += (detected[i][2]-dist_avg)/(i+1)
            rads_avg += (detected[i][1]-rads_avg)/(i+1)
            thresh = dist_avg+0.5*dist_avg #Threshold
            if detected[i][2] > thresh: #Remove contribution from average if too big
                dist_avg -= (detected[i][2]-dist_avg)/(i+1)
                rads_avg -= (detected[i][1]-rads_avg)/(i+1)
                del detected[i] #Remove from detected, it doesn't fit in with the object.

        if not detected:
            print("No leader detected")
        else:
            self.lidar_dist = dist_avg
            self.lidar_angle = math.degrees(rads_avg)

        
if __name__ == '__main__':
    rospy.init_node("decentralized_complete")
    rate = rospy.Rate(10) #Loop rate 100Hz
    obj = move_bot()

    #Initialize /tf transform listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    transform_detected = 0

    #Lists for plotting
    time_list = []
    #epsilon_d_list = []
    #epsilon_beta_list = []
    error_d_list = []
    error_beta_list = []
    #rho_d_list = []
    #rho_beta_list = []
    d_lower_boundary_list = []
    d_upper_boundary_list = []
    beta_lower_boundary_list = []
    beta_upper_boundary_list = []
    distance_list = []
    angle_list = []
    v_list = []
    w_list = []

    #Wait for first message from callbacks. 
    rospy.wait_for_message("/tb3_1/scan",LaserScan)


    #Makes sure that timer starts correctly and avoids race conditions
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

            #Wait for first message from callbacks. 
            #rospy.wait_for_message("/tb3_1/scan",LaserScan)

            currentTime = rospy.Time.now()
            delT = currentTime-prevTime

            #Get information about distance and angle 
            distance = np.sqrt((0-trans.transform.translation.x)**2+(0-trans.transform.translation.y)**2)
            rads = np.arctan(-trans.transform.translation.y/-trans.transform.translation.x)
            angle = math.degrees(rads)

            #print("Distance:",distance)
            obj.control(distance,angle,delT.to_sec())


            #Append data to list for plotting
            time_list.append(delT.to_sec())
            #epsilon_d_list.append(obj.store_epsilon_d)
            #epsilon_beta_list.append(obj.store_epsilon_beta)
            error_d_list.append(obj.store_error_d)
            error_beta_list.append(obj.store_error_beta)
            #rho_d_list.append(obj.store_rho_d)
            #rho_beta_list.append(obj.store_rho_beta)
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

    #Code for plotting
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
 