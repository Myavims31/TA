import rospy
from std_msgs.msg import Float32MultiArray

def input_node():
    pub = rospy.Publisher('pos_robot', Float32MultiArray, queue_size=10)
    rospy.init_node('input_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    x = float(input("Enter x: "))
    y = float(input("Enter y: "))
    z = float(input("Enter z: "))
    roll = float(input("Enter roll: "))
    pitch = float(input("Enter pitch: "))
    yaw = float(input("Enter yaw: "))

    y_prediksi = [1, 2, 3, 4]

    print(y_prediksi)

    theta1 = y_prediksi[0]
    theta2 = y_prediksi[1]
    theta3 = y_prediksi[2]
    theta4 = y_prediksi[3]

    float_array_msg = Float32MultiArray()
    float_array_msg.data = [theta1, theta2, theta3, theta4]

    

    while not rospy.is_shutdown():
        pub.publish(float_array_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        input_node()
    except rospy.ROSInterruptException:
        pass