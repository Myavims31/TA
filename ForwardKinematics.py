import numpy as np
import math

# Define DH parameters for each link
a1 = 0.012
a2 = 0.128
a3 = 0.148
a4 = 0.126
d1 = 0.077
d2 = 0
d3 = 0
d4 = 0
rad1 = float(input("rad1="))
rad2 = float(input("rad2="))
rad3 = float(input("rad3="))
rad4 = float(input("rad4="))
theta1 = rad1 #*(180/np.pi)
theta2 = rad2 + np.deg2rad(90)
theta3 = rad3 - np.deg2rad(90)
# theta2 = rad2-((np.pi/180)*11)#*(180/np.pi)
# theta3 = rad3+((np.pi/180)*11) #*(180/np.pi)
theta4 = rad4 #*(180/np.pi)
alpha1 = np.pi/2
alpha2 = alpha3 = alpha4 = 0

# Define transformation matrix for each link
T1 = np.array([[np.cos(theta1), -np.sin(theta1)*np.cos(alpha1), np.sin(theta1)*np.sin(alpha1), a1*np.cos(theta1)],
               [np.sin(theta1), np.cos(theta1)*np.cos(alpha1), -np.cos(theta1)*np.sin(alpha1), a1*np.sin(theta1)],
               [0, np.sin(alpha1), np.cos(alpha1), d1],
               [0, 0, 0, 1]])

T2 = np.array([[np.cos(theta2), -np.sin(theta2)*np.cos(alpha2), np.sin(theta2)*np.sin(alpha2), a2*np.cos(theta2)],
               [np.sin(theta2), np.cos(theta2)*np.cos(alpha2), -np.cos(theta2)*np.sin(alpha2), a2*np.sin(theta2)],
               [0, np.sin(alpha2), np.cos(alpha2), d2],
               [0, 0, 0, 1]])

T3 = np.array([[np.cos(theta3), -np.sin(theta3)*np.cos(alpha3), np.sin(theta3)*np.sin(alpha3), a3*np.cos(theta3)],
               [np.sin(theta3), np.cos(theta3)*np.cos(alpha3), -np.cos(theta3)*np.sin(alpha3), a3*np.sin(theta3)],
               [0, np.sin(alpha3), np.cos(alpha3), d3],
               [0, 0, 0, 1]])

T4 = np.array([[np.cos(theta4), -np.sin(theta4)*np.cos(alpha4), np.sin(theta4)*np.sin(alpha4), a4*np.cos(theta4)],
               [np.sin(theta4), np.cos(theta4)*np.cos(alpha4), -np.cos(theta4)*np.sin(alpha4), a4*np.sin(theta4)],
               [0, np.sin(alpha4), np.cos(alpha4), d4],
               [0, 0, 0, 1]])

# Compute overall transformation matrix from base to end effector
T = np.matmul(np.matmul(T1, T2),np.matmul(T3, T4))
# Extract position and orientation of end effector
position = T[:3, 3]
position[0] = round(float(position[0]),3)
position[1] = round(float(position[1]),3)
position[2] = round(float(position[2]),3)

orientation = T[:3, :3]

def rotation_matrix_to_euler_angles(R):
    # Menghitung roll (rotasi sepanjang sumbu X)
    roll = math.atan2(R[2][1], R[2][2])

    # Menghitung pitch (rotasi sepanjang sumbu Y)
    pitch = math.atan2(-R[2][0], math.sqrt(R[2][1]**2 + R[2][2]**2))

    # Menghitung yaw (rotasi sepanjang sumbu Z)
    yaw = math.atan2(R[1][0], R[0][0])

    # Mengonversi sudut dari radian menjadi derajat
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    return roll_deg, pitch_deg, yaw_deg

# Contoh penggunaan
rotation_matrix = T[:3, :3]

roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

# print(T)

print("Position:\n", position)
print("Orientation:\n", orientation)
print("OrientationEuler:\n", [roll, pitch, yaw])