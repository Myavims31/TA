#include <iostream>
#include <cmath>
#include <iomanip>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Geometry>
#include "sensor_msgs/JointState.h"
#include "open_manipulator_msgs/KinematicsPose.h"

using namespace std;

float joint_1, prev_joint_1 = 0;
float joint_2, prev_joint_2 = 0;
float joint_3, prev_joint_3 = 0;
float joint_4, prev_joint_4 = 0;

float pos_x, prev_pos_x = 0;
float pos_y, prev_pos_y = 0;
float pos_z, prev_pos_z = 0;

float orient_x, prev_orient_x = 0;
float orient_y, prev_orient_y = 0;
float orient_z, prev_orient_z = 0;
float orient_w, prev_orient_w = 0;

void jointsCallback(const sensor_msgs::JointState& msg)
{
    // Untuk UI
    joint_1 = std::round(msg.position.at(2)*1000)/1000;
    joint_2 = std::round(msg.position.at(3)*1000)/1000;
    joint_3 = std::round(msg.position.at(4)*1000)/1000;
    joint_4 = std::round(msg.position.at(5)*1000)/1000;

    // printf("%.4f,%.4f,%.4f,%.4f\n", joint_1, joint_2, joint_3, joint_4);
}

void gripperCallback(const open_manipulator_msgs::KinematicsPose& msg)
{
    pos_x = std::round(msg.pose.position.x*1000)/1000;
    pos_y = std::round(msg.pose.position.y*1000)/1000;
    pos_z = std::round(msg.pose.position.z*1000)/1000;
    
    orient_x = std::round(msg.pose.orientation.x*1000)/1000;
    orient_y = std::round(msg.pose.orientation.y*1000)/1000;
    orient_z = std::round(msg.pose.orientation.z*1000)/1000;
    orient_w = std::round(msg.pose.orientation.w*1000)/1000;

    //atan2(2*(orient_w*orient_x+orient_y*orient_z),1-2 *((orient_x*orient_x)+(orient_y*orient_y)));

    // printf("%.4f,%.4f,%.4f\n",pos_x, pos_y, pos_z);
}

float round_function(float var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensors");
  ros::NodeHandle n;
  ros::Rate rate(100); //ROS rate at Hz (T=1/f)
  ros::Subscriber sub_joint = n.subscribe("/joint_states", 100, jointsCallback);
  ros::Subscriber sub_gripper = n.subscribe("/gripper/kinematics_pose", 100, gripperCallback);

  while(ros::ok()){

    // Fungsi agar melakukan printing apabila terdapat value yang berbeda
    if(abs(joint_1) != abs(prev_joint_1) || abs(joint_2) != abs(prev_joint_2) || abs(joint_3) != abs(prev_joint_3) || abs(joint_4) != abs(prev_joint_4) 
    || abs(pos_x) != abs(prev_pos_x) || abs(pos_y) != abs(prev_pos_y) || abs(pos_z) != (prev_pos_z) 
    || abs(orient_x) != (prev_orient_x) || abs(orient_y) != (prev_orient_y) || abs(orient_z) != (prev_orient_z) || abs(orient_w) != (prev_orient_w))
    {
        Eigen::Quaterniond quat(orient_w, orient_x, orient_y, orient_z);
        quat.normalize();
        Eigen::Matrix3d rot_matrix = quat.toRotationMatrix();
        Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(2, 1, 0);
        Eigen::Vector3d euler_angles_deg = euler_angles;
        double roll = abs(euler_angles_deg(2));    // z-axis rotation
        double pitch = abs(euler_angles_deg(1));   // Y-axis rotation
        double yaw = abs(euler_angles_deg(0));     // x-axis rotation
        //std::cout << roll << std::endl;
        //std::cout << pitch << std::endl;
        //std::cout << yaw << std::endl;    
    
        printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", 
        joint_1, joint_2, joint_3, joint_4, pos_x, pos_y, pos_z, 
        orient_x, orient_y, orient_z, orient_w, roll, pitch, yaw);

    }

    // variable previous = variable sekarang
    prev_joint_1 = joint_1;
    prev_joint_2 = joint_2;
    prev_joint_3 = joint_3;
    prev_joint_4 = joint_4;
    prev_pos_x = pos_x;
    prev_pos_y = pos_y;
    prev_pos_z = pos_z;
    prev_orient_x = orient_x;
    prev_orient_y = orient_y;
    prev_orient_z = orient_z;
    prev_orient_w = orient_w;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}