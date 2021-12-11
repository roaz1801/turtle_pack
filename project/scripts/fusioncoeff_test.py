import csv 
import matplotlib.pyplot as plt 
import numpy as np
import math
"""
Plotte de 3 versjonene av metodene inn i plotter, så 
camera_only, lidar_only, filtered, complete.
Må legges inn etter at data har blitt skaffet.

"""

if __name__ == '__main__':
    """
    For these, the indices mean:
    0:time_list,
    1:v_list, 
    2:w_list,
    3:d_lower_boundary_list,
    4:d_upper_boundary_list,
    5:error_d_list,
    6:beta_lower_boundary_list,
    7:beta_upper_boundary_list,
    8:error_beta_list,
    9:distance_list,
    10:angle_list
    """
    
    with open("decentcomplete_weighttest_0.5_0.5.csv") as f:
        exp1 = list(csv.reader(f,delimiter=","))

    with open("decentcomplete_weighttest_0.3_0.7.csv") as f:
        exp2 = list(csv.reader(f,delimiter=","))

    with open("decentcomplete_weighttest_0.7_0.3.csv") as f:
        exp3 = list(csv.reader(f,delimiter=","))

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
    with open("decentcomplete_pos_weighttest_0.5_0.5.csv") as f:
        exp1_pos = list(csv.reader(f,delimiter=","))

    with open("decentcomplete_pos_weighttest_0.3_0.7.csv") as f:
        exp2_pos = list(csv.reader(f,delimiter=","))

    with open("decentcomplete_pos_weighttest_0.7_0.3.csv") as f:
        exp3_pos = list(csv.reader(f,delimiter=","))

    for i in range(len(exp1)):
       exp1[i] = np.float_(exp1[i])
       exp2[i] = np.float_(exp2[i])
       exp3[i] = np.float_(exp3[i])

    for i in range(len(exp1_pos)):
       exp1_pos[i] = np.float_(exp1_pos[i])
       exp2_pos[i] = np.float_(exp2_pos[i])
       exp3_pos[i] = np.float_(exp3_pos[i])

    for i in range(len(exp1_pos[3][:3000])):
        exp1_pos[3][i] = math.degrees(math.atan2(np.sin(exp1_pos[1][i]-exp1_pos[2][i]),np.cos(exp1_pos[1][i]-exp1_pos[2][i])))
        exp2_pos[3][i] = math.degrees(math.atan2(np.sin(exp2_pos[1][i]-exp2_pos[2][i]),np.cos(exp2_pos[1][i]-exp2_pos[2][i])))
        exp3_pos[3][i] = math.degrees(math.atan2(np.sin(exp3_pos[1][i]-exp3_pos[2][i]),np.cos(exp3_pos[1][i]-exp3_pos[2][i])))

    pos_error_rms1 = np.sqrt(1/2*(exp1_pos[4]**2+exp1_pos[5]**2))
    pos_error_rms2 = np.sqrt(1/2*(exp2_pos[4]**2+exp2_pos[5]**2))
    pos_error_rms3 = np.sqrt(1/2*(exp3_pos[4]**2+exp3_pos[5]**2))

    #Starting at 200, this is 20 seconds. Take the mean and std.dev
    #from here to avoid transient

    #velocity mean and stdev
    print("Velocity mean 1:",np.mean(exp1[1][200:3000]))
    print("Velocity std.dev 1:",np.std(exp1[1][200:3000]))

    print("Velocity mean 2:",np.mean(exp2[1][200:3000]))
    print("Velocity std.dev 2:",np.std(exp2[1][200:3000]))

    print("Velocity mean 3:",np.mean(exp3[1][200:3000]))
    print("Velocity std.dev 3:",np.std(exp3[1][200:3000]))

    #Angular velocity mean and stdev
    print("Angular Velocity mean 1:",np.mean(exp1[2][200:3000]))
    print("Angular Velocity std.dev 1:",np.std(exp1[2][200:3000]))

    print("Angular Velocity mean 2:",np.mean(exp2[2][200:3000]))
    print("Angular Velocity std.dev 2:",np.std(exp2[2][200:3000]))

    print("Angular Velocity mean 3:",np.mean(exp3[2][200:3000]))
    print("Angular Velocity std.dev 3:",np.std(exp3[2][200:3000]))

    #velocity error mean and std dev
    print("Distance error mean 1:",np.mean(exp1[5][200:3000]))
    print("Distance error std.dev 1:",np.std(exp1[5][200:3000]))

    print("Distance error mean 2:",np.mean(exp2[5][200:3000]))
    print("Distance error std.dev 2:",np.std(exp2[5][200:3000]))

    print("Distance error mean 3:",np.mean(exp3[5][200:3000]))
    print("Distance error std.dev 3:",np.std(exp3[5][200:3000]))


    #Angular velocity error mean and std dev
    print("Angular error mean 1:",np.mean(exp1[8][200:3000]))
    print("Angular error 1 std.dev:",np.std(exp1[8][200:3000]))

    print("Angular error mean 2:",np.mean(exp2[8][200:3000]))
    print("Angular error std.dev 2:",np.std(exp2[8][200:3000]))

    print("Angular error mean 3:",np.mean(exp3[8][200:3000]))
    print("Angular error std.dev 3:",np.std(exp3[8][200:3000]))  

    #Distance mean and std. dev
    print("Distance mean 1:",np.mean(exp1[9][200:3000]))
    print("Distance std.dev 1:",np.std(exp1[9][200:3000]))

    print("Distance mean 2:",np.mean(exp2[9][200:3000]))
    print("Distance std.dev 2:",np.std(exp2[9][200:3000]))

    print("Distance mean 3:",np.mean(exp3[9][200:3000]))
    print("Distance std.dev 3:",np.std(exp3[9][200:3000]))

    #Angle mean and std. dev
    print("Angle mean 1:",np.mean(exp1[10][200:3000]))
    print("Angle std.dev 1:",np.std(exp1[10][200:3000]))

    print("Angle mean 2:",np.mean(exp2[10][200:3000]))
    print("Angle std.dev 2:",np.std(exp2[10][200:3000]))

    print("Angle mean 3:",np.mean(exp3[10][200:3000]))
    print("Angle std.dev 3:",np.std(exp3[10][200:3000]))

    #position error RMS error, mean and std. dev
    print("Pos_error RMS mean 1:",np.mean(pos_error_rms1[200:3000]))
    print("Pos_error RMS std.dev 1:",np.std(pos_error_rms1[200:3000]))

    print("Pos_error RMS mean 2:",np.mean(pos_error_rms2[200:3000]))
    print("Pos_error RMS std.dev 2:",np.std(pos_error_rms2[200:3000]))

    print("Pos_error RMS mean 3:",np.mean(pos_error_rms3[200:3000]))
    print("Pos_error RMS std.dev 3:",np.std(pos_error_rms3[200:3000]))

    #Heading error mean and std. dev
    print("Heading error mean 1:",np.mean(exp1_pos[3][200:3000]))
    print("Heading error std.dev 1:",np.std(exp1_pos[3][200:3000]))

    print("Heading error mean 2:",np.mean(exp2_pos[3][200:3000]))
    print("Heading error std.dev 2:",np.std(exp2_pos[3][200:3000]))

    print("Heading error mean 3:",np.mean(exp3_pos[3][200:3000]))
    print("Heading error std.dev 3:",np.std(exp3_pos[3][200:3000]))

    #Stopping index at 3000 because that is 300 seconds
    #Plotting velocity and angular velocity
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:3000],exp1[1][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[0].plot(exp2[0][:3000],exp2[1][:3000],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[0].plot(exp3[0][:3000],exp3[1][:3000],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Velocity in m/s")
    axis[0].set_title("Velocity")
    axis[0].legend()
    
    axis[1].plot(exp1[0][:3000],exp1[2][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[1].plot(exp2[0][:3000],exp2[2][:3000],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[1].plot(exp3[0][:3000],exp3[2][:3000],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Angular velocity in m/s")
    axis[1].set_title("Angular velocity")
    axis[1].legend()
    plt.show()

    #Plotting boundary and errors
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:3000],exp1[3][:3000],label="Lower boundary")
    axis[0].plot(exp1[0][:3000],exp1[5][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[0].plot(exp2[0][:3000],exp2[5][:3000],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[0].plot(exp3[0][:3000],exp3[5][:3000],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[0].plot(exp1[0][:3000],exp1[4][:3000],label="Upper boundary")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Distance Error")
    axis[0].set_title("Distance Boundary and Error")
    axis[0].legend()

    axis[1].plot(exp1[0][:3000],exp1[6][:3000],label="Upper boundary")
    axis[1].plot(exp1[0][:3000],exp1[8][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[1].plot(exp2[0][:3000],exp2[8][:3000],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[1].plot(exp3[0][:3000],exp3[8][:3000],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[1].plot(exp1[0][:3000],exp1[7][:3000],label="Lower boundary")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Angle Error")
    axis[1].set_title("Angle Boundary and Error")
    axis[1].legend()
    plt.show()

    #Plotting distance and angle
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:3000],exp1[9][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[0].plot(exp2[0][:3000],exp2[9][:3000],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[0].plot(exp3[0][:3000],exp3[9][:3000],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Relative distance in m")
    axis[0].set_title("Distance between leader and follower")
    axis[0].legend()

    axis[1].plot(exp1[0][:3000],exp1[10][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[1].plot(exp2[0][:3000],exp2[10][:3000],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[1].plot(exp3[0][:3000],exp3[10][:3000],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Relative angle")
    axis[1].set_title("Angle between leader and follower")
    axis[1].legend()
    plt.show()

    #Plotting position
    plt.plot(exp1_pos[6][:3000],exp1_pos[7][:3000],label="Leader")
    plt.plot(exp1_pos[8][:3000],exp1_pos[9][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    plt.plot(exp2_pos[8][:3000],exp2_pos[9][:3000],label="(k1,k2)=(0.3,0.7)",color = "red")
    plt.plot(exp3_pos[8][:3000],exp3_pos[9][:3000],label="(k1,k2)=(0.5,0.7)",color = "green")
    plt.title("Position of leader and follower")
    plt.xlabel("X position in meters")
    plt.ylabel("Y position in meters")
    plt.axis('equal')
    plt.legend()
    plt.show()

    #Plotting position error
    plt.plot(exp1_pos[0][:3000],pos_error_rms1[:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    plt.plot(exp2_pos[0][:3000],pos_error_rms2[:3000],label="(k1,k2)=(0.3,0.7)",color = "green")
    plt.plot(exp3_pos[0][:3000],pos_error_rms3[:3000],label="(k1,k2)=(0.5,0.7)",color = "red")
    plt.title("RMS of error in x and y position")
    plt.xlabel("Time")
    plt.ylabel("RMS")
    plt.legend()
    plt.show()

    #Plotting position heading error
    plt.plot(exp1_pos[0][:3000],exp1_pos[3][:3000],label="(k1,k2)=(0.3,0.5)",color = "blue")
    plt.plot(exp2_pos[0][:3000],exp2_pos[3][:3000],label="(k1,k2)=(0.3,0.7)",color = "green")
    plt.plot(exp3_pos[0][:3000],exp3_pos[3][:3000],label="(k1,k2)=(0.5,0.7)",color = "red")
    plt.title("Error in heading")
    plt.xlabel("Time")
    plt.ylabel("Heading angle")
    plt.legend()
    plt.show()
