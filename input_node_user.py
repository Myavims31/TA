import rospy
from std_msgs.msg import Float32MultiArray
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

#Import Library For Machine Learning NN
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.utils import to_categorical
from tensorflow.keras import layers
from sklearn.model_selection import train_test_split
import keras
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Activation
from keras.layers import Dropout
from keras.layers import LSTM
from tensorflow.keras import backend as K
from tensorflow.keras.callbacks import Callback
import matplotlib.pyplot as plt

def lms(y_true,y_pred): #by simon haykin book's
  return (K.square(y_pred-y_true))/2

def input_node():
    #ML
    loaded_model = tf.keras.models.load_model("ModelNN.h5", custom_objects={"lms": lms })


    pub = rospy.Publisher('pos_robot', Float32MultiArray, queue_size=10)
    rospy.init_node('input_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    x = float(input("Enter x: "))
    y = float(input("Enter y: "))
    z = float(input("Enter z: "))
    roll = float(input("Enter roll: "))
    pitch = float(input("Enter pitch: "))
    yaw = float(input("Enter yaw: "))

    data = {
    'x': x,
    'y': y,
    'z': z,
    'roll': roll,
    'pitch': pitch,
    'yaw': yaw
}
    df = pd.DataFrame(data, index=[0])
    Y_prediksi = loaded_model.predict(df)

    print(Y_prediksi)

    theta1 = Y_prediksi[0][0]
    theta2 = Y_prediksi[0][1]
    theta3 = Y_prediksi[0][2]
    theta4 = Y_prediksi[0][3]

    float_array_msg = Float32MultiArray()
    float_array_msg.data = [theta1, theta2, theta3, theta4]
    print(float_array_msg)

    joint_position = JointPosition()
    joint_position.joint_name = [f"joint{i+1}" for i in range(4)]
    if not args.trt:
        joint_position.position = [float_array_msg[i].item() for i in range(4)]
    else:
        joint_position.position = [float_array_msg[0][i].item() for i in range(4)]
    request = SetJointPosition.Request()
    request.joint_position = joint_position
    request.path_time = 4.0
    

    while not rospy.is_shutdown():
        pub.publish(float_array_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        input_node()
    except rospy.ROSInterruptException:
        pass