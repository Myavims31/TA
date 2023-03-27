import numpy as np

# Define DH parameters for each link
a1 = 0
a2 = 0.13
a3 = 0.124
a4 = 0.028
d1 = 0.077
rad1 = input("rad1=")
theta1 = float(rad1)*(np.pi/4)
theta2 = np.pi/3
theta3 = np.pi/4
theta4 = np.pi/3

# Define transformation matrix for each link
T1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, a1*np.cos(theta1)],
               [np.sin(theta1), np.cos(theta1), 0, a1*np.sin(theta1)],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

T2 = np.array([[np.cos(theta2), -np.sin(theta2), 0, a2*np.cos(theta2)],
               [np.sin(theta2), np.cos(theta2), 0, a2*np.sin(theta2)],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

T3 = np.array([[np.cos(theta3), -np.sin(theta3), 0, a3*np.cos(theta3)],
               [np.sin(theta3), np.cos(theta3), 0, a3*np.sin(theta3)],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

T4 = np.array([[np.cos(theta4), -np.sin(theta4), 0, a4*np.cos(theta4)],
               [np.sin(theta4), np.cos(theta4), 0, a4*np.sin(theta4)],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

# Compute overall transformation matrix from base to end effector
T = np.dot(np.dot(T1, T2),np.dot(T3, T4))
# Extract position and orientation of end effector
position = T[:3, 3]
orientation = T[:3, :3]

print("Position:", position)
print("Orientation:", orientation)
