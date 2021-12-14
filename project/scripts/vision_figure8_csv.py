import csv 
import matplotlib.pyplot as plt 
import numpy as np
import math

if __name__ == '__main__':

    """
    For this, the indices mean:
    0:time_list,
    1:v_list
    2:w_list
    """
    with open("csv_files/leader_figure8.csv") as f:
        leader = list(csv.reader(f,delimiter=","))

    for i in range(len(leader[1])):
        leader[0][i] = np.float(leader[0][i])
        leader[1][i] = np.float(leader[1][i])
        leader[2][i] = np.float(leader[2][i])

    """
    For these, the indices mean:
    0:time_list,
    1:v_list, 
    2:w_list, 
    3:n_lower_boundary_list,
    4:n_upper_boundary_list,
    5:error_n_list,
    6:m_lower_boundary_list,
    7:m_upper_boundary_list,
    8:error_m_list,
    9:n_list,
    10:m_list,
    11:distance_list,
    12:angle_list
    """
    with open("csv_files/k1_0.07_k2_0.45_vision_follow_figure8.csv") as f:
        exp1 = list(csv.reader(f,delimiter=","))

    with open("csv_files/k1_0.07_k2_0.45_vision_lidar_only_figure8.csv") as f:
        exp2 = list(csv.reader(f,delimiter=","))

    with open("csv_files/k1_0.07_k2_0.45_vision_filtered_figure8.csv") as f:
        exp3 = list(csv.reader(f,delimiter=","))

    with open("csv_files/k1_0.07_k2_0.45_vision_complete_figure8.csv") as f:
        exp4 = list(csv.reader(f,delimiter=","))

    
    """
    For these, indices mean:
    0:time_list[delay:],
    1:leader_heading, 
    2:follower_heading,
    3:heading_error,
    4:x_error,
    5:y_error,
    6:tb0_xlist,
    7:tb0_ylist,
    8:tb1_xlist,
    9:tb1_ylist,
    """
    with open("csv_files/k1_0.07_k2_0.45_vision_follow_figure8_position.csv") as f:
        exp1_pos = list(csv.reader(f,delimiter=","))

    with open("csv_files/k1_0.07_k2_0.45_vision_lidar_only_figure8_position.csv") as f:
        exp2_pos = list(csv.reader(f,delimiter=","))

    with open("csv_files/k1_0.07_k2_0.45_vision_filtered_figure8_position.csv") as f:
        exp3_pos = list(csv.reader(f,delimiter=","))

    with open("csv_files/k1_0.07_k2_0.45_vision_complete_figure8_position.csv") as f:
        exp4_pos = list(csv.reader(f,delimiter=","))

        
    for i in range(len(exp1)):
       exp1[i] = np.float_(exp1[i])
       exp2[i] = np.float_(exp2[i])
       exp3[i] = np.float_(exp3[i])
       exp4[i] = np.float_(exp4[i])

    for i in range(len(exp1_pos)):
       exp1_pos[i] = np.float_(exp1_pos[i])
       exp2_pos[i] = np.float_(exp2_pos[i])
       exp3_pos[i] = np.float_(exp3_pos[i])
       exp4_pos[i] = np.float_(exp4_pos[i])

    vel_error1 = np.zeros(1280)
    vel_error2 = np.zeros(1280)
    vel_error3 = np.zeros(1280)
    vel_error4 = np.zeros(1280)

    ang_vel_error1 = np.zeros(1280)
    ang_vel_error2 = np.zeros(1280)
    ang_vel_error3 = np.zeros(1280)
    ang_vel_error4 = np.zeros(1280)
    for i in range(len(exp1_pos[3][:1280])):
        vel_error1[i] = exp1[1][i]-leader[1][i]
        vel_error2[i] = exp2[1][i]-leader[1][i]
        vel_error3[i] = exp3[1][i]-leader[1][i]
        vel_error4[i] = exp4[1][i]-leader[1][i]

        ang_vel_error1[i] = exp1[2][i]-leader[2][i]
        ang_vel_error2[i] = exp2[2][i]-leader[2][i]
        ang_vel_error3[i] = exp3[2][i]-leader[2][i]
        ang_vel_error4[i] = exp4[2][i]-leader[2][i]

    pos_error_rms1 = np.sqrt(1/2*(exp1_pos[4]**2+exp1_pos[5]**2))
    pos_error_rms2 = np.sqrt(1/2*(exp2_pos[4]**2+exp2_pos[5]**2))
    pos_error_rms3 = np.sqrt(1/2*(exp3_pos[4]**2+exp3_pos[5]**2))
    pos_error_rms4 = np.sqrt(1/2*(exp4_pos[4]**2+exp4_pos[5]**2))


 #Starting at 200, this is 20 seconds. Take the mean and std.dev
    #from here to avoid transient

    #velocity mean and stdev
    print("Velocity mean 1:",np.mean(exp1[1][200:1280]))
    print("Velocity std.dev 1:",np.std(exp1[1][200:1280]))

    print("Velocity mean 2:",np.mean(exp2[1][200:1280]))
    print("Velocity std.dev 2:",np.std(exp2[1][200:1280]))

    print("Velocity mean 3:",np.mean(exp3[1][200:1280]))
    print("Velocity std.dev 3:",np.std(exp3[1][200:1280]))

    print("Velocity mean 4:",np.mean(exp4[1][200:1280]))
    print("Velocity std.dev 4:",np.std(exp4[1][200:1280]))

    #Angular velocity mean and stdev
    print("Angular Velocity mean 1:",np.mean(exp1[2][200:1280]))
    print("Angular Velocity std.dev 1:",np.std(exp1[2][200:1280]))

    print("Angular Velocity mean 2:",np.mean(exp2[2][200:1280]))
    print("Angular Velocity std.dev 2:",np.std(exp2[2][200:1280]))

    print("Angular Velocity mean 3:",np.mean(exp3[2][200:1280]))
    print("Angular Velocity std.dev 3:",np.std(exp3[2][200:1280]))

    print("Angular Velocity mean 4:",np.mean(exp4[2][200:1280]))
    print("Angular Velocity std.dev 4:",np.std(exp4[2][200:1280]))   

    #velocity error mean and stdev
    print("Velocity error mean 1:",np.mean(vel_error1[200:1280]))
    print("Velocity error std.dev 1:",np.std(vel_error1[200:1280]))

    print("Velocity error mean 2:",np.mean(vel_error2[200:1280]))
    print("Velocity error std.dev 2:",np.std(vel_error2[200:1280]))

    print("Velocity error mean 3:",np.mean(vel_error3[200:1280]))
    print("Velocity error std.dev 3:",np.std(vel_error3[200:1280]))

    print("Velocity error mean 4:",np.mean(vel_error4[200:1280]))
    print("Velocity error std.dev 4:",np.std(vel_error4[200:1280]))

    #Angular velocity error mean and stdev
    print("Angular velocity error mean 1:",np.mean(ang_vel_error1[200:1280]))
    print("Angular velocity error std.dev 1:",np.std(ang_vel_error1[200:1280]))

    print("Angular Velocity error mean 2:",np.mean(ang_vel_error2[200:1280]))
    print("Angular Velocity error std.dev 2:",np.std(ang_vel_error2[200:1280]))

    print("Angular Velocity error mean 3:",np.mean(ang_vel_error3[200:1280]))
    print("Angular velocity error std.dev 3:",np.std(ang_vel_error3[200:1280]))

    print("Angular velocity error mean 4:",np.mean(ang_vel_error4[200:1280]))
    print("Angular velocity error std.dev 4:",np.std(ang_vel_error4[200:1280]))   

    #Distance error mean and std dev
    print("Image coordinate n error mean 1:",np.mean(exp1[5][200:1280]))
    print("Image coordinate n error std.dev 1:",np.std(exp1[5][200:1280]))

    print("Image coordinate n error mean 2:",np.mean(exp2[5][200:1280]))
    print("Image coordinate n error std.dev 2:",np.std(exp2[5][200:1280]))

    print("Image coordinate n error mean 3:",np.mean(exp3[5][200:1280]))
    print("Image coordinate n error std.dev 3:",np.std(exp3[5][200:1280]))

    print("Image coordinate n error mean 4:",np.mean(exp4[5][200:1280]))
    print("Image coordinate n error std.dev 4:",np.std(exp4[5][200:1280]))

    #Angular error mean and std dev
    print("Image coordinate m error mean 1:",np.mean(exp1[8][200:1280]))
    print("Image coordinate m error std.dev 1:",np.std(exp1[8][200:1280]))

    print("Image coordinate m error mean 2:",np.mean(exp2[8][200:1280]))
    print("Image coordinate m error std.dev 2:",np.std(exp2[8][200:1280]))

    print("Image coordinate m error mean 3:",np.mean(exp3[8][200:1280]))
    print("Image coordinate m error std.dev 3:",np.std(exp3[8][200:1280]))

    print("Image coordinate m error mean 4:",np.mean(exp4[8][200:1280]))
    print("Image coordinate m error std.dev 4:",np.std(exp4[8][200:1280]))   

    #Distance mean and std. dev
    print("Distance mean 1:",np.mean(exp1[11][200:1280]))
    print("Distance std.dev 1:",np.std(exp1[11][200:1280]))

    print("Distance mean 2:",np.mean(exp2[11][200:1280]))
    print("Distance std.dev 2:",np.std(exp2[11][200:1280]))

    print("Distance mean 3:",np.mean(exp3[11][200:1280]))
    print("Distance std.dev 3:",np.std(exp3[11][200:1280]))

    print("Distance mean 4:",np.mean(exp4[11][200:1280]))
    print("Distance std.dev 4:",np.std(exp4[11][200:1280]))

    #Angle mean and std. dev
    print("Angle mean 1:",np.mean(exp1[12][200:1280]))
    print("Angle std.dev 1:",np.std(exp1[12][200:1280]))

    print("Angle mean 2:",np.mean(exp2[12][200:1280]))
    print("Angle std.dev 2:",np.std(exp2[12][200:1280]))

    print("Angle mean 3:",np.mean(exp3[12][200:1280]))
    print("Angle std.dev 3:",np.std(exp3[12][200:1280]))

    print("Angle mean 4:",np.mean(exp4[12][200:1280]))
    print("Angle std.dev 4:",np.std(exp4[12][200:1280]))

    #position error RMS error, mean and std. dev
    print("Pos_error RMS mean 1:",np.mean(pos_error_rms1[200:1280]))
    print("Pos_error RMS std.dev 1:",np.std(pos_error_rms1[200:1280]))

    print("Pos_error RMS mean 2:",np.mean(pos_error_rms2[200:1280]))
    print("Pos_error RMS std.dev 2:",np.std(pos_error_rms2[200:1280]))

    print("Pos_error RMS mean 3:",np.mean(pos_error_rms3[200:1280]))
    print("Pos_error RMS std.dev 3:",np.std(pos_error_rms3[200:1280]))

    print("Pos_error RMS mean 4:",np.mean(pos_error_rms4[200:1280]))
    print("Pos_error RMS std.dev 4:",np.std(pos_error_rms4[200:1280]))


    #Plotting velocity and angular velocity
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:1280],exp1[1][:1280],label="Camera-only",color = "blue")
    axis[0].plot(exp2[0][:1280],exp2[1][:1280],label="Lidar-only",color = "red")
    axis[0].plot(exp3[0][:1280],exp3[1][:1280],label="Filtered",color = "green")
    axis[0].plot(exp4[0][:1280],exp4[1][:1280],label="Complete",color = "orange")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Velocity in m/s")
    axis[0].set_title("Velocity")
    axis[0].legend()
    
    axis[1].plot(exp1[0][:1280],exp1[2][:1280],label="Camera-only",color = "blue")
    axis[1].plot(exp2[0][:1280],exp2[2][:1280],label="Lidar only",color = "red")
    axis[1].plot(exp3[0][:1280],exp3[2][:1280],label="Filtered",color = "green")
    axis[1].plot(exp4[0][:1280],exp4[2][:1280],label="Complete",color = "orange")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Angular velocity in m/s")
    axis[1].set_title("Angular velocity")
    axis[1].legend()
    plt.show()

    #Plotting boundary and errors
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:1280],exp1[3][:1280],label="Lower boundary")
    axis[0].plot(exp1[0][:1280],exp1[5][:1280],label="Camera-only",color = "blue")
    axis[0].plot(exp2[0][:1280],exp2[5][:1280],label="Lidar-only",color = "red")
    axis[0].plot(exp3[0][:1280],exp3[5][:1280],label="Filtered",color = "green")
    axis[0].plot(exp1[0][:1280],exp4[5][:1280],label="Complete",color = "orange")
    axis[0].plot(exp4[0][:1280],exp1[4][:1280],label="Upper boundary")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Image coordinate (n) error ")
    axis[0].set_title("Image coordinate n Boundary and Error")
    axis[0].legend()

    axis[1].plot(exp1[0][:1280],exp1[6][:1280],label="Upper boundary")
    axis[1].plot(exp1[0][:1280],exp1[8][:1280],label="Camera-only",color = "blue")
    axis[1].plot(exp2[0][:1280],exp2[8][:1280],label="Lidar-only",color = "red")
    axis[1].plot(exp3[0][:1280],exp3[8][:1280],label="Filtered",color = "green")
    axis[1].plot(exp1[0][:1280],exp4[8][:1280],label="Complete",color = "orange")
    axis[1].plot(exp4[0][:1280],exp1[7][:1280],label="Lower boundary")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Image coordinate (m) Error")
    axis[1].set_title("Image coordinate m Boundary and Error")
    axis[1].legend()
    plt.show()

    #Plotting distance and angle
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:1280],exp1[11][:1280],label="Camera-only",color = "blue")
    axis[0].plot(exp2[0][:1280],exp2[11][:1280],label="Lidar-only",color = "red")
    axis[0].plot(exp3[0][:1280],exp3[11][:1280],label="Filtered",color = "green")
    axis[0].plot(exp4[0][:1280],exp4[11][:1280],label="Complete",color = "orange")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Relative distance in m")
    axis[0].set_title("Distance between leader and follower")
    axis[0].legend()

    axis[1].plot(exp1[0][:1280],exp1[12][:1280],label="Camera-only",color = "blue")
    axis[1].plot(exp2[0][:1280],exp2[12][:1280],label="Lidar-only",color = "red")
    axis[1].plot(exp3[0][:1280],exp3[12][:1280],label="Filtered",color = "green")
    axis[1].plot(exp4[0][:1280],exp4[12][:1280],label="Complete",color = "orange")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Relative angle")
    axis[1].set_title("Angle between leader and follower")
    axis[1].legend()
    plt.show()

    #Plotting position
    plt.plot(exp1_pos[6][:1280],exp1_pos[7][:1280],label="Leader")
    plt.plot(exp1_pos[8][:1280],exp1_pos[9][:1280],label="Camera-only",color = "blue")
    plt.plot(exp2_pos[8][:1280],exp2_pos[9][:1280],label="Lidar-only",color = "red")
    plt.plot(exp3_pos[8][:1280],exp3_pos[9][:1280],label="Filtered",color = "green")
    plt.plot(exp4_pos[8][:1280],exp4_pos[9][:1280],label="Complete",color = "orange")
    plt.title("Position of leader and follower")
    plt.xlabel("X position in meters")
    plt.ylabel("Y position in meters")
    plt.axis('equal')
    plt.legend()
    plt.show()

    #Plotting position error
    plt.plot(exp1_pos[0][:1280],pos_error_rms1[:1280],label="Camera-only",color = "blue")
    plt.plot(exp2_pos[0][:1280],pos_error_rms2[:1280],label="Lidar-only",color = "red")
    plt.plot(exp3_pos[0][:1280],pos_error_rms3[:1280],label="Filtered",color = "green")
    plt.plot(exp4_pos[0][:1280],pos_error_rms4[:1280],label="Complete",color = "orange")
    plt.title("RMS of error in x and y position")
    plt.xlabel("Time")
    plt.ylabel("RMS")
    plt.legend()
    plt.show()

