import csv 
import matplotlib.pyplot as plt 
import numpy as np

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
    with open("decent_k1_0.3_k2_0.5.csv") as f:
        exp1 = list(csv.reader(f,delimiter=","))

    with open("decent_k1_0.3_k2_0.7.csv") as f:
        exp2 = list(csv.reader(f,delimiter=","))

    with open("decent_k1_0.5_k2_0.7.csv") as f:
        exp3 = list(csv.reader(f,delimiter=","))

    with open("decent_k1_0.5_k2_1.csv") as f:
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
    with open("decentralized_pos_k1_0.3_k2_0.5.csv") as f:
        exp1_pos = list(csv.reader(f,delimiter=","))

    with open("decentralized_pos_k1_0.3_k2_0.7.csv") as f:
        exp2_pos = list(csv.reader(f,delimiter=","))

    with open("decentralized_pos_k1_0.5_k2_0.7.csv") as f:
        exp3_pos = list(csv.reader(f,delimiter=","))

    with open("decentralized_pos_k1_0.5_k2_1.csv") as f:
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


    #Plotting velocity and angular velocity
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:1500],exp1[1][:1500],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[0].plot(exp2[0][:1500],exp2[1][:1500],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[0].plot(exp3[0][:1500],exp3[1][:1500],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[0].plot(exp4[0][:1500],exp4[1][:1500],label="(k1,k2)=(0.5,1)",color = "orange")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Velocity in m/s")
    axis[0].set_title("Velocity")
    axis[0].legend()
    
    axis[1].plot(exp1[0][:1500],exp1[2][:1500],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[1].plot(exp2[0][:1500],exp2[2][:1500],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[1].plot(exp3[0][:1500],exp3[2][:1500],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[1].plot(exp4[0][:1500],exp4[2][:1500],label="(k1,k2)=(0.5,1)",color = "orange")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Angular velocity in m/s")
    axis[1].set_title("Angular velocity")
    axis[1].legend()
    plt.show()

    #Plotting boundary and errors
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:1500],exp1[3][:1500],label="Lower boundary")
    axis[0].plot(exp1[0][:1500],exp1[5][:1500],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[0].plot(exp2[0][:1500],exp2[5][:1500],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[0].plot(exp3[0][:1500],exp3[5][:1500],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[0].plot(exp1[0][:1500],exp4[5][:1500],label="(k1,k2)=(0.5,1)",color = "orange")
    axis[0].plot(exp4[0][:1500],exp1[4][:1500],label="Upper boundary")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Distance Error")
    axis[0].set_title("Distance Boundary and Error")
    axis[0].legend()

    axis[1].plot(exp1[0][:1500],exp1[6][:1500],label="Upper boundary")
    axis[1].plot(exp1[0][:1500],exp1[8][:1500],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[1].plot(exp2[0][:1500],exp2[8][:1500],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[1].plot(exp3[0][:1500],exp3[8][:1500],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[1].plot(exp1[0][:1500],exp4[8][:1500],label="(k1,k2)=(0.5,1)",color = "orange")
    axis[1].plot(exp4[0][:1500],exp1[7][:1500],label="Lower boundary")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Angle Error")
    axis[1].set_title("Angle Boundary and Error")
    axis[1].legend()
    plt.show()

    #Plotting distance and angle
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(exp1[0][:1500],exp1[9][:1500],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[0].plot(exp2[0][:1500],exp2[9][:1500],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[0].plot(exp3[0][:1500],exp3[9][:1500],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[0].plot(exp4[0][:1500],exp4[9][:1500],label="(k1,k2)=(0.5,1)",color = "orange")
    axis[0].set_xlabel("Time in seconds")
    axis[0].set_ylabel("Relative distance in m")
    axis[0].set_title("Distance between leader and follower")
    axis[0].legend()

    axis[1].plot(exp1[0][:1500],exp1[10][:1500],label="(k1,k2)=(0.3,0.5)",color = "blue")
    axis[1].plot(exp2[0][:1500],exp2[10][:1500],label="(k1,k2)=(0.3,0.7)",color = "red")
    axis[1].plot(exp3[0][:1500],exp3[10][:1500],label="(k1,k2)=(0.5,0.7)",color = "green")
    axis[1].plot(exp4[0][:1500],exp4[10][:1500],label="(k1,k2)=(0.5,1)",color = "orange")
    axis[1].set_xlabel("Time in seconds")
    axis[1].set_ylabel("Relative angle")
    axis[1].set_title("Angle between leader and follower")
    axis[1].legend()
    plt.show()

    #Plotting position
    plt.plot(exp1_pos[6],exp1_pos[7],label="Leader")
    plt.plot(exp1_pos[8],exp1_pos[9],label="(k1,k2)=(0.3,0.5)",color = "blue")
    plt.plot(exp2_pos[8],exp2_pos[9],label="(k1,k2)=(0.3,0.7)",color = "red")
    plt.plot(exp3_pos[8],exp3_pos[9],label="(k1,k2)=(0.5,0.7)",color = "green")
    plt.plot(exp4_pos[8],exp4_pos[9],label="(k1,k2)=(0.5,1)",color = "orange")
    plt.title("Position of leader and follower")
    plt.xlabel("X position in meters")
    plt.ylabel("Y position in meters")
    plt.axis('equal')
    plt.legend()
    plt.show()

    #Plotting position error
    plt.plot(exp1_pos[0],exp1_pos[4],label="(k1,k2)=(0.3,0.5)",color = "blue")
    plt.plot(exp1_pos[0],exp1_pos[5],label="(k1,k2)=(0.3,0.5)",color = "blue")
    plt.plot(exp2_pos[0],exp2_pos[4],label="(k1,k2)=(0.3,0.7)",color = "green")
    plt.plot(exp2_pos[0],exp2_pos[5],label="(k1,k2)=(0.3,0.7)",color = "green")
    plt.plot(exp3_pos[0],exp3_pos[4],label="(k1,k2)=(0.5,0.7)",color = "red")
    plt.plot(exp3_pos[0],exp3_pos[5],label="(k1,k2)=(0.5,0.7)",color = "red")
    plt.plot(exp4_pos[0],exp4_pos[4],label="(k1,k2)=(0.5,1)",color = "orange")
    plt.plot(exp4_pos[0],exp4_pos[5],label="(k1,k2)=(0.5,1)",color = "orange")
    plt.title("Error in x and y position")
    plt.xlabel("Time")
    plt.ylabel("Error in meters")
    plt.legend()
    plt.show()
