import numpy as np
import serial
import time
#Can remove this, added in for readability when debugging
np.set_printoptions(precision=4, suppress=True)

# Create D-H frame transformation matrix from parameters
def create_dh(t_i, a_i, r_i, d_i):
    ct = np.cos(t_i)
    st = np.sin(t_i)
    ca= np.cos(a_i)
    sa = np.sin(a_i)
    return np.array([
        [ct,   -st*ca, st*sa,  r_i*ct],
        [st,   ct*ca,  -ct*sa, r_i*st],
        [0,    sa,     ca,     d_i],
        [0,    0,      0,      1]])

# From completed D-H table, calculate the forward kinematics for specific angles
def fk(j0, j1, j2, j3, j4):
    T0, T1, T2, T3, T4, T5 = (
        create_dh(j0, -np.pi/2, 0, 70),
        create_dh(j1-np.pi/2, 0, 125, 0),
        create_dh(j2+np.pi/2, np.pi/2, 0, 18.7),
        create_dh(j3+np.pi/2, -np.pi/2, 0, 105),
        create_dh(j4, np.pi/2, 0, 50),
        create_dh(0, 0, 0, 150),
    )
    return (T0 @ T1 @ T2 @ T3 @ T4 @ T5)

# Generate the Jacobian for Damped Least Squares inverse kinematics calculations
def form_jacobian(j0, j1, j2, j3, j4):
    T = [np.eye(4)]
    rm = np.array([[0], [0], [1]])
    for m in [create_dh(j0, -np.pi/2, 0, 70),
              create_dh(j1-np.pi/2, 0, 125, 0),
              create_dh(j2+np.pi/2, np.pi/2, 0, 18.7),
              create_dh(j3+np.pi/2, -np.pi/2, 0, 105),
              create_dh(j4, np.pi/2, 0, 50),
              create_dh(0, 0, 0, 50),]:
        T.append(T[-1] @ m)
    D06 = T[-1][:3,3]
    j_matrix = np.zeros((6,6))
    for i in range(6):
        R0i = T[i][:3, :3]
        Zi = T[i][:3, 3]
        j_matrix[:3, i] = (np.cross(np.transpose(R0i @ rm), (D06-Zi)))
        j_matrix[3:, i] = np.transpose(R0i @ rm)
    return j_matrix

# Solve inverse kinematics for a desired x,yz position.
# If position_only is set to true, the solver will find any solution that satisfies convergence criteria for the desired position
# If position_only is set to false, the solver will bias itself to find solutions with the wrist level with the surface (horizontal approach)
def find_ik(x_d, y_d, z_d, j0, j1, j2, j3, j4, position_only):
    desired_location = np.array([x_d, y_d, z_d])
    desired_orientation = np.array([1, 0, 0])
    iterations = 0
    
    if not position_only:
        n_rows = 6
    else:
        n_rows = 3
        
    #Alpha and lambda for DLS IK method, adjusted for more consistent convergence
    alpha = 1
    lam = 0.03

    joint_limits = [
        (-np.pi*135/180, np.pi*135/180),
        (-np.pi/2, np.pi/2),
        (-np.pi*135/180, np.pi*135/180),
        (-np.pi/2, np.pi/2),
        (-np.pi/2, np.pi/2),
        ]
    # Wraps resultant angles to limits of the servo (if beyond capabilities)
    def legal_angle(theta, num):
        return max(joint_limits[num][0], min(theta, joint_limits[num][1]))
    
    # Iteration maximum of 1000 if convergence criteria not met earlier
    while iterations < 1000:
        H06 = fk(j0, j1, j2, j3, j4)
        R06 = H06[:3, :3]
        curr_loc = H06[:3, 3]
        error_vector = desired_location - curr_loc
        
        if not position_only:
            curr_ori = R06[2,:]
            ori_error = np.cross(curr_ori, desired_orientation)
            delta_v = np.concatenate([error_vector, ori_error * 1])
        else:
            delta_v = np.concatenate([error_vector])
            ori_error = np.array([0,0,0])
        
        
        J = form_jacobian(j0, j1, j2, j3, j4)
        J = J[:n_rows,:]
        dt = np.linalg.solve(J.T @ J + lam**2 *np.eye(6), J.T @ delta_v)
        
        j0 = legal_angle(j0 + alpha*dt[0], 0)
        j1 = legal_angle(j1+ alpha*dt[1], 1)
        j2 = legal_angle(j2+ alpha*dt[2], 2)
        j3 = legal_angle(j3+ alpha*dt[3], 3)
        j4 = legal_angle(j4+ alpha*dt[4], 4)
        
        iterations += 1
        
        # Convergence update every 100 iterations
        if (iterations) % 100 == 0:
            print(f"Iteration {iterations} | XYZ Error: {np.linalg.norm(error_vector):.4f} | Rotation Error: {np.linalg.norm(ori_error):.4f}")
        
        if np.linalg.norm(error_vector) < 1 and np.linalg.norm(ori_error) < 0.01:
            print(f"Convergence achieved (Iteration {iterations}) | XYZ Error: {np.linalg.norm(error_vector):.4f} | Rotation Error: {np.linalg.norm(ori_error):.4f}")
            break
    return np.array([j0, j1, j2, j3, j4])
        
# Serial settings for communication with Arduino, edit COM port based on what the Arduino/microcontroller is connected to
seri = serial.Serial('COM6', 9600)
time.sleep(15)
# Boots the robot and waits to receive the "BOOTED" message from the robot 
A_UNO = seri.readline().decode().strip().upper()
print(f"Arduinio Responds: {A_UNO}")

# Send angles (IN DEGREES) to the serial port
def sendToArduino(angles):
    info_msg = ','.join([str(x) for x in angles]) + '\n'
    seri.write(info_msg.encode())
    unoR = seri.readline().decode().strip().upper()
    print(f"Arduinio Responds: {unoR}")

# Safe guard function, can bypass and just use sendToArduino
# sConf calculates FK and displays the angles the robot will move towards to prevent
# sending illegal angles to the robot (when debugging/testing) or collisions    
def sConf(ang_deg, ang_rad):
    print(f"\n -----ANGLES-----\n J0: {ang_deg[0]} \n J1: {ang_deg[1]} \n J2: {ang_deg[2]} \n J3: {ang_deg[3]} \n J4: {ang_deg[4]}")
    H = fk(*ang_rad)
    print(f"FK: \n X: {H[0,3]} \n Y: {H[1,3]} \n Z: {H[2,3]}")
    
    conf = input("\n Would you like to move the robot this location (Y/N)?")
    if (conf.upper() == "Y"):
        print("\n Sending angles to the robot, be prepared for movement.")
        sendToArduino(ang_deg)
    else:
        print("\n CANCELLED")
    



    

    





