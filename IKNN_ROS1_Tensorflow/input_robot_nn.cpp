#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <open_manipulator_msgs/SetJointPosition.h>


//variable for ROS
ros::Subscriber sub_theta;
ros::ServiceClient goal_joint_space_path_client_;
open_manipulator_msgs::SetJointPosition srv;


// variable for tuning
double joint_1, joint_2, joint_3, joint_4;
std::vector<std::string> joint_name_global;
std::vector<double> joint_angle_global;
std_msgs::Float32MultiArray theta_msg;

bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
void sensorCallback(const std_msgs::Float32MultiArray& msg);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "input_node_controller");
    ros::NodeHandle nh;
    ros::Rate rate(500);
    sub_theta = nh.subscribe("pos_robot", 10, sensorCallback);


    goal_joint_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    std::cout << joint_1 << " " << joint_2 << " " << joint_3 << " " << joint_4 << std::endl;

    ros::Duration delay_duration(5.0);
    ROS_INFO("TUNGGU SEK 5 DETIK...");

    delay_duration.sleep();
    
    std::cout << joint_1 << " " << joint_2 << " " << joint_3 << " " << joint_4 << std::endl;



    double path_time = 4.0;
    joint_name_global.push_back("joint1"); 
    joint_name_global.push_back("joint2"); 
    joint_name_global.push_back("joint3"); 
    joint_name_global.push_back("joint4"); 

    joint_angle_global.push_back(joint_1);
    joint_angle_global.push_back(joint_2);
    joint_angle_global.push_back(joint_3);
    joint_angle_global.push_back(joint_4);
    

    setJointSpacePath(joint_name_global, joint_angle_global, path_time);

    return 0;
}


bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
//   open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;
  

  std::cout<<"test1"<<std::endl;

  if(goal_joint_space_path_client_.call(srv))
  {
    ROS_INFO("OKE BAGUS LANJUTKAN");
    return srv.response.is_planned;
  }
    std::cout<<"test4"<<std::endl;

  return false;
}


void sensorCallback(const std_msgs::Float32MultiArray& msg)
{
    theta_msg.data = msg.data;

    joint_1 = theta_msg.data[0];
    joint_2 = theta_msg.data[1];
    joint_3 = theta_msg.data[2];
    joint_4 = theta_msg.data[3];

    double path_time = 4.0;
    joint_name_global.push_back("joint1"); 
    joint_name_global.push_back("joint2"); 
    joint_name_global.push_back("joint3"); 
    joint_name_global.push_back("joint4"); 

    joint_angle_global.push_back(joint_1);
    joint_angle_global.push_back(joint_2);
    joint_angle_global.push_back(joint_3);
    joint_angle_global.push_back(joint_4);
      
    std::cout << theta_msg << std::endl;
    
    std::cout<<"test2"<<std::endl;
    
    setJointSpacePath(joint_name_global, joint_angle_global, path_time);



}