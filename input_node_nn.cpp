#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <open_manipulator_msgs/SetJointPosition.h>

ros::Publisher pub_theta;
ros::Subscriber sub_theta;

std_msgs::Float32MultiArray theta_msg;
std::vector<std::string> joint_name_global;
std::vector<double> joint_angle_global;


void sensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    theta_msg.data = msg->data;
    std::cout << theta_msg << std::endl;
}

bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "input_node_controller");
    ros::NodeHandle nh;
    
    goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");

    double path_time = 2.0;
    joint_name_global.push_back("joint1"); joint_angle_global.push_back(theta_msg.data[0]);
    joint_name_global.push_back("joint2"); joint_angle_global.push_back(theta_msg.data[1]);
    joint_name_global.push_back("joint3"); joint_angle_global.push_back(theta_msg.data[2]);
    joint_name_global.push_back("joint4"); joint_angle_global.push_back(theta_msg.data[3]);

    setJointSpacePath(joint_name_global, joint_angle_global, path_time);

    pub_theta = nh.advertise<std_msgs::Float32MultiArray>("theta", 1000);
    sub_theta = nh.subscribe("pos_robot", 10, sensorCallback);

    ros::spin();
    return 0;
}
